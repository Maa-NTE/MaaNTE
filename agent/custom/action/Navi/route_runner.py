import threading
import time
from typing import Any, Callable

from maa.context import Context

from .coordinate_position import CoordinatePositionProvider
from .map_locator import MapLocator
from .waypoint_navigator import WaypointNavigator
from .route_model import RouteSession, SourceSize, Waypoint

_ROUTE_START_POSITION_TIMEOUT = 1.0


class RouteRunner:
    """按顺序执行可变路线会话中的路径点。"""

    def __init__(
        self,
        context: Context,
        route: RouteSession,
        *,
        angle_backend: str = "auto",
        position_backend: str = "map",
        tolerance: float = 80.0,
        frame_interval: float = 0.1,
        debug: bool = False,
        on_frame: Callable[[Any, Any], None] | None = None,
        max_duration: float | None = None,
        periodic_jump: bool = False,
    ) -> None:
        self.context = context
        self.route = route
        self.angle_backend = angle_backend
        self.position_backend = position_backend
        self.tolerance = tolerance
        self.frame_interval = max(0.05, float(frame_interval))
        self.debug = debug
        self.on_frame = on_frame
        self.max_duration = max_duration
        self.periodic_jump = periodic_jump
        self.navigator: WaypointNavigator | None = None
        self._position_providers: dict[str, CoordinatePositionProvider] = {}
        self._current_point: Waypoint | None = None
        self._moving_target: tuple[int, Waypoint] | None = None
        self._lock = threading.Lock()

    def configure(
        self,
        *,
        position_backend: str,
        angle_backend: str,
        debug: bool,
        tolerance: float,
        frame_interval: float,
        max_duration: float | None,
        periodic_jump: bool,
    ) -> None:
        self.position_backend = position_backend
        self.angle_backend = angle_backend
        self.debug = debug
        self.tolerance = tolerance
        self.frame_interval = max(0.05, frame_interval)
        self.max_duration = max_duration
        self.periodic_jump = periodic_jump
        if self.navigator is not None:
            self.navigator.tolerance = tolerance
            self.navigator.max_duration = max_duration
            self.navigator.configure_positioning(
                self.get_position_provider(position_backend),
                angle_backend,
                frame_interval,
                debug,
            )

    def get_position_provider(self, backend: str) -> CoordinatePositionProvider:
        backend = backend.strip().lower()
        if backend not in self._position_providers:
            provider = CoordinatePositionProvider(backend, debug=self.debug)
            self._position_providers[backend] = provider
            if not provider.uses_visual_positioning():
                # 两种模式共享已启动的抓包。中间切换到视觉定位也保留到任务结束。
                self._position_providers.update(auto=provider, coordinate=provider)
        return self._position_providers[backend]

    def start(self) -> None:
        if self.navigator is not None:
            return
        self.navigator = WaypointNavigator(
            self.context,
            angle_backend=self.angle_backend,
            position_provider=self.get_position_provider(self.position_backend),
            tolerance=self.tolerance,
            frame_interval=self.frame_interval,
            debug=self.debug,
            on_frame=self._on_frame,
            should_cancel=self._should_cancel,
            max_duration=self.max_duration,
        )

    def run_until_stopped(
        self,
        *,
        on_tick: Callable[[], None] | None = None,
        stop_when_route_done: bool = False,
        jump_when_stuck: bool = False,
    ) -> str:
        self.start()
        assert self.navigator is not None
        self.navigator.reset_jumps()

        while not self.context.tasker.stopping:
            if on_tick is not None:
                on_tick()
            payload = self.route.payload()
            if not payload["active"]:
                if stop_when_route_done and payload["status"] in {
                    "arrived",
                    "cleared",
                    "empty",
                    "stopped",
                    "timeout",
                }:
                    return str(payload["status"])
                started = time.perf_counter()
                self.navigator.update()
                self.navigator.sleep_remaining(started)
                continue

            with self.route.lock:
                current_index = self.route.current_index
                waypoints = list(self.route.waypoints)
            if current_index >= len(waypoints):
                with self.route.lock:
                    self.route.active = False
                    self.route.status = "arrived"
                continue

            # 关闭末段的周期跳跃，让角色正常走到拾取位置。
            if self.periodic_jump:
                total = len(waypoints)
                if total > 3:
                    jump_on = current_index < total - 2
                else:
                    jump_on = False
                self.navigator.set_jump_enabled(jump_on)
            else:
                self.navigator.set_jump_enabled(False)

            target = waypoints[current_index]
            with self._lock:
                self._moving_target = (current_index, target)
            try:
                arrived = self.navigator.move_to(target, jump_when_stuck=jump_when_stuck)
            except Exception:
                self.navigator.release()
                raise
            finally:
                with self._lock:
                    self._moving_target = None
            if arrived:
                self.route.advance()
                continue

            reason = self.navigator.last_stop_reason or "timeout"
            if reason == "timeout":
                # 寻路超时：记录状态并结束本次运行，避免对同一个路径点无限重试
                with self.route.lock:
                    self.route.active = False
                    self.route.status = "timeout"
                if stop_when_route_done:
                    return "timeout"
            # 其余情况（被取消、或任务正在停止）不改动路线状态：回到循环顶部
            # 重新读取，让长期运行的在线寻路继续服务后续指令；一次性运行则由
            # 循环顶部的 active/status 判断正常退出。
            continue

        return "stopped"

    def close(self) -> None:
        try:
            if self.navigator is not None:
                self.navigator.close()
                self.navigator = None
        finally:
            for provider in set(self._position_providers.values()):
                provider.close()
            self._position_providers.clear()

    def update_current_frame(self, *, since: float | None = None) -> Waypoint | None:
        self.start()
        assert self.navigator is not None
        provider = self.navigator.position_provider
        assert provider is not None
        deadline = time.monotonic()
        if since is not None and not provider.uses_visual_positioning():
            # 传送前采集的包可能延迟送达。起点选择和后续走位都只用本段的新包。
            provider.sample_not_before = since
            deadline += _ROUTE_START_POSITION_TIMEOUT

        while not self.context.tasker.stopping:
            result = self.navigator.update()
            if result is not None:
                location, _ = result
                if location.found and location.point is not None:
                    return location.point
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(self.navigator.frame_interval, remaining))
        return None

    def source_size(self) -> SourceSize:
        navigator = self.navigator
        return navigator.map_size if navigator is not None else MapLocator.MAP_SIZE

    def current_point(self) -> Waypoint | None:
        with self._lock:
            return self._current_point

    def _on_frame(self, location: Any, angle: Any) -> None:
        if location.found and location.point is not None:
            with self._lock:
                self._current_point = location.point
        if self.on_frame is not None:
            self.on_frame(location, angle)

    def _should_cancel(self) -> bool:
        with self._lock:
            target = self._moving_target
        if target is None:
            return False
        target_index, target_point = target
        with self.route.lock:
            return (
                not self.route.active
                or self.route.current_index != target_index
                or target_index >= len(self.route.waypoints)
                or self.route.waypoints[target_index] != target_point
            )
