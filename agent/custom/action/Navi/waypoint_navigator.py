import json
import math
import time
from dataclasses import dataclass
from typing import Any, Callable

from maa.context import Context

from utils.maafocus import PrintT

from ..Common.logger import get_logger
from .angle_predictor import AnglePredictionResult, AnglePredictor
from .coordinate_position import CoordinatePositionProvider
from .debug_windows import close_debug_windows, pump_debug_windows
from .map_locator import MapLocator

logger = get_logger(__name__)

_KEY_W = 87
_KEY_SPACE = 0x20
_NETWORK_FRAME_INTERVAL = 1.0 / 60.0

# 单次截图的最长等待时间：截图卡住时也必须回到循环顶部，让寻路超时和
# 任务停止信号有机会生效，避免整个路径点被一次卡死的截图拖住。
_CAPTURE_TIMEOUT = 10.0
_CAPTURE_POLL_INTERVAL = 0.02

# 空格键按住的时长，用于「按下」与「松开」之间留出一点游戏输入间隔。
_JUMP_HOLD = 0.05
# 以导航底图像素计，进入到达范围前留出一段落地距离。
_JUMP_APPROACH_MARGIN = 12.0
# 前进时允许坐标在 2 个底图像素内抖动，持续 2 秒才判定卡住。
_STALL_RADIUS = 2.0
_STALL_SECONDS = 2.0


@dataclass
class AnglePidController:
    """PID 输出角速度，按采样间隔换算成鼠标本次需要转过的角度。"""

    kp: float = 2.5
    ki: float = 0.15
    kd: float = 0.15
    output_limit: float = 35.0
    max_speed: float = 220.0  # 度/秒
    integral_limit: float = 60.0
    deadband: float = 4.0
    restart_margin: float = 2.0
    sample_interval: float = 0.12
    max_dt: float = 0.25
    derivative_filter_time: float = 0.18
    _integral: float = 0.0
    _last_error: float | None = None
    _last_heading: float | None = None
    _angular_speed: float = 0.0
    _last_time: float | None = None

    def reset(self) -> None:
        self._integral = 0.0
        self._last_error = None
        self._last_heading = None
        self._angular_speed = 0.0
        self._last_time = None

    def update(
        self, error: float, now: float, *, heading: float | None = None
    ) -> float:
        threshold = self.deadband
        if self._last_time is None:
            threshold += self.restart_margin
        if abs(error) <= threshold:
            self.reset()
            return 0.0

        elapsed = (
            self.sample_interval if self._last_time is None else now - self._last_time
        )
        if elapsed <= 0:
            return 0.0
        dt = min(elapsed, self.max_dt)

        # 目标在正后方时，细微噪声即可使 +179° 变成 -179°。沿已选方向掉头。
        if self._last_error is not None and error * self._last_error < 0:
            if abs(error) >= 170.0 and abs(self._last_error) >= 170.0:
                error += math.copysign(360.0, self._last_error)
            else:
                self._integral = 0.0

        # 只对实测朝向求导，路点或整数坐标引起的目标方向突变不会产生微分冲击。
        if heading is not None and self._last_heading is not None:
            rotation = (heading - self._last_heading + 180.0) % 360.0 - 180.0
            weight = elapsed / (self.derivative_filter_time + elapsed)
            self._angular_speed += weight * (rotation / elapsed - self._angular_speed)
        else:
            self._angular_speed = 0.0

        integral = max(
            -self.integral_limit, min(self.integral_limit, self._integral + error * dt)
        )
        speed = self.kp * error + self.ki * integral - self.kd * self._angular_speed
        direction = math.copysign(1.0, error)
        requested = direction * speed * dt
        # 给下一帧朝向反馈留出余量，制动时可停下等待，单帧不反打鼠标。
        limit = min(self.output_limit, self.max_speed * dt, abs(error) * 0.3)
        if 0.0 < requested < limit:
            self._integral = integral
        self._last_error = error
        self._last_heading = heading
        self._last_time = now
        return direction * max(0.0, min(requested, limit))


class WaypointNavigator:
    """根据实时位置和朝向，将角色移动到单个地图路径点。"""

    def __init__(
        self,
        context: Context,
        *,
        angle_backend: str = "auto",
        position_backend: str = "map",
        position_provider: CoordinatePositionProvider | None = None,
        tolerance: float = 80.0,
        max_duration: float | None = None,
        frame_interval: float = 0.1,
        debug: bool = False,
        on_frame: Callable[[Any, Any], None] | None = None,
        should_cancel: Callable[[], bool] | None = None,
    ) -> None:
        self.context = context
        # Maa Agent 每次从同一个 Tasker 取 controller 都会替换原生代理。
        # 用独立的 Context 持有导航控制器，避免传送或拾取截图使它失效。
        self._controller_context = context.clone()
        self.controller = self._controller_context.tasker.controller
        self.tolerance = tolerance
        self.max_duration = max_duration
        self.debug = debug
        self.on_frame = on_frame
        self.should_cancel = should_cancel
        self.frame_interval = max(0.05, float(frame_interval))
        self.turn_pixels_per_degree = 10.0
        self.max_turn_degrees = 35.0
        self.resume_forward_degrees = 25.0

        self.align_threshold = 4.0
        self.move_pulse = 0.12
        self.key_refresh_interval = 0.5
        self.turn_pid = AnglePidController(
            output_limit=self.max_turn_degrees,
            deadband=self.align_threshold,
            sample_interval=self.move_pulse,
        )
        self.w_down = False
        self.space_down = False
        self._last_w_down_at = 0.0

        # 周期性跳跃参数
        self.jump_interval = 3.0
        self.jump_hold = _JUMP_HOLD
        self.jump_enabled = False
        self._next_jump_at: float | None = None

        # move_to() 结束原因：arrived / timeout / cancelled / stopped。
        # RouteRunner 依据它区分「寻路超时」与「路线被停止、清空或替换」。
        self.last_stop_reason: str | None = None

        self.current_point: tuple[int, int] | None = None
        self.locator: MapLocator | None = None
        self.position_provider: CoordinatePositionProvider | None = None
        self._owned_position_provider: CoordinatePositionProvider | None = None
        self.predictor: AnglePredictor | None = None
        self._visual_settings: tuple[str, bool] | None = None
        self._network_position_reported = False
        try:
            if position_provider is None:
                self._owned_position_provider = CoordinatePositionProvider(
                    position_backend, debug=debug
                )
                position_provider = self._owned_position_provider
            self.configure_positioning(
                position_provider, angle_backend, frame_interval, debug
            )
        except Exception:
            self.close()
            raise

    def configure_positioning(
        self,
        provider: CoordinatePositionProvider,
        angle_backend: str,
        frame_interval: float,
        debug: bool,
    ) -> None:
        previous_provider = self.position_provider
        self.position_provider = provider
        self.debug = debug
        provider.debug = debug
        if not provider.uses_visual_positioning():
            self.frame_interval = _NETWORK_FRAME_INTERVAL
            self.map_size = MapLocator.MAP_SIZE
            if provider is not previous_provider:
                logger.debug(
                    "Navi network pose active: visual angle predictor disabled, "
                    "control_rate=60Hz",
                    extra={"file_only": True},
                )
            return

        self.frame_interval = max(0.05, float(frame_interval))
        if self.locator is None or self.locator.debug != debug:
            if self.locator is not None:
                self.locator.close()
                self.locator = None
            self.locator = MapLocator(debug=debug)
        self.map_size = self.locator.origin_w, self.locator.origin_h
        if self._visual_settings != (angle_backend, debug):
            if self.predictor is not None:
                self.predictor.close()
                self.predictor = None
            self._visual_settings = None
            self.predictor = AnglePredictor(
                backend=angle_backend, threshold=0.0, debug=debug
            )
            self._visual_settings = (angle_backend, debug)

    def reset_jumps(self) -> None:
        self._next_jump_at = None

    def set_jump_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if enabled != self.jump_enabled:
            # 切换跳跃开关时重置计时。否则复用同一个导航器时，关闭跳跃期间
            # 已经过期的计时会让下一次开启立刻起跳。
            self._next_jump_at = None
        self.jump_enabled = enabled

    def jump(self) -> None:
        """按一次空格跳跃。

        只要按下过空格，就一定会补发松开按键，避免异常路径把空格键留在按下
        状态，导致角色持续起跳。
        """
        self.space_down = True
        try:
            self.controller.post_key_down(_KEY_SPACE).wait()
            time.sleep(self.jump_hold)
        finally:
            try:
                self.controller.post_key_up(_KEY_SPACE).wait()
            finally:
                self.space_down = False

    def _screencap(self, deadline: float | None = None):
        """截取一帧，等待过程受超时与任务停止信号约束。

        框架的 Job.wait() 没有超时参数，因此这里改为轮询作业状态；一旦截图长
        时间不返回，就放弃本次截图，让调用方回到循环顶部重新检查截止时间。
        """
        job = self.controller.post_screencap()
        poll_deadline = time.monotonic() + _CAPTURE_TIMEOUT
        if deadline is not None:
            poll_deadline = min(poll_deadline, deadline)
        while not job.done:
            if self.context.tasker.stopping:
                return None
            if time.monotonic() >= poll_deadline:
                logger.warning(
                    "WaypointNavigator screencap stalled, dropping this frame"
                )
                return None
            time.sleep(_CAPTURE_POLL_INTERVAL)
        return job.get()

    def update(self, deadline: float | None = None) -> tuple[Any, Any] | None:
        assert self.position_provider is not None
        if self.position_provider.uses_visual_positioning():
            frame = self._screencap(deadline)
            if frame is None:
                if self.debug:
                    pump_debug_windows()
                return None
            assert self.predictor is not None
            location = self.position_provider.locate(self.locator, frame)
            angle = self.predictor.predict(frame)
        else:
            location = self.position_provider.locate(None, None)
            if location.found and not self._network_position_reported:
                PrintT(self.context, "navi.network_positioning_active")
                self._network_position_reported = True
            angle = AnglePredictionResult(
                found=location.found and location.camera_heading is not None,
                angle=location.camera_heading,
                confidence=1.0 if location.camera_heading is not None else 0.0,
            )
        if location.found and location.point is not None:
            self.current_point = location.point
        if self.on_frame is not None:
            self.on_frame(location, angle)
        return location, angle

    def move_to(
        self, target: tuple[int, int], *, jump_when_stuck: bool = False
    ) -> bool:
        target_x, target_y = target
        deadline = (
            time.monotonic() + self.max_duration
            if self.max_duration is not None
            else None
        )
        last_log_time = 0.0
        turn_in_place = False
        stall_anchor: tuple[int, int] | None = None
        stall_since = 0.0
        self.turn_pid.reset()
        self.last_stop_reason = None

        while not self.context.tasker.stopping:
            if deadline is not None and time.monotonic() >= deadline:
                logger.info("WaypointNavigator timeout: target=%s", target)
                self.release()
                self.last_stop_reason = "timeout"
                return False
            if self.should_cancel is not None and self.should_cancel():
                # 路线被停止、清空或被替换属于取消，不是寻路超时
                self.release()
                self.last_stop_reason = "cancelled"
                return False

            started = time.perf_counter()
            result = self.update(deadline)

            if result is None:
                stall_anchor = None
                self.sleep_remaining(started)
                continue
            location, angle = result
            if (
                not location.found
                or location.point is None
                or not angle.found
                or angle.angle is None
            ):
                stall_anchor = None
                self.turn_pid.reset()
                self.release()
                self.sleep_remaining(started)
                continue

            current_x, current_y = location.point
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx, dy)

            if deadline is not None and time.monotonic() >= deadline:
                logger.info(
                    "WaypointNavigator timeout after localization: target=%s "
                    "current=%s distance=%.1f",
                    target,
                    location.point,
                    distance,
                )
                self.release()
                self.last_stop_reason = "timeout"
                return False
            if distance <= self.tolerance:
                logger.debug(
                    "WaypointNavigator arrived: target=%s current=%s distance=%.1f",
                    target,
                    location.point,
                    distance,
                )
                self.release()
                self.last_stop_reason = "arrived"
                return True

            desired_angle = math.degrees(math.atan2(dx, -dy)) % 360.0
            angle_delta = (desired_angle - angle.angle + 540.0) % 360.0 - 180.0
            # 原地转身后多对准一点再恢复前进，避免在 35° 附近反复松按 W。
            move_threshold = (
                self.resume_forward_degrees if turn_in_place else self.max_turn_degrees
            )
            facing_target = abs(angle_delta) <= move_threshold
            turn_in_place = location.camera_heading is not None and not facing_target
            if turn_in_place and self.w_down:
                self.release()
            turn_degrees = self.turn_pid.update(
                angle_delta, time.monotonic(), heading=angle.angle
            )
            turn_dx = int(round(turn_degrees * self.turn_pixels_per_degree))
            if turn_dx != 0:
                self.controller.post_relative_move(turn_dx, 0).wait()
            if not turn_in_place:
                self.press_forward()

            now = time.monotonic()
            near_target = distance <= self.tolerance + _JUMP_APPROACH_MARGIN
            if (
                jump_when_stuck
                and near_target
                and facing_target
                and location.camera_heading is not None
            ):
                if (
                    stall_anchor is None
                    or math.dist(location.point, stall_anchor) > _STALL_RADIUS
                ):
                    stall_anchor = location.point
                    stall_since = now
                elif now - stall_since >= _STALL_SECONDS:
                    logger.info(
                        "WaypointNavigator stall jump: target=%s current=%s distance=%.1f",
                        target,
                        location.point,
                        distance,
                    )
                    self.jump()
                    jump_when_stuck = False
                    self._next_jump_at = time.monotonic() + self.jump_interval
            else:
                stall_anchor = None

            if self.jump_enabled:
                if self._next_jump_at is None:
                    self._next_jump_at = now + self.jump_interval
                elif (
                    now >= self._next_jump_at
                    and facing_target
                    and not near_target
                ):
                    self.jump()
                    self._next_jump_at = time.monotonic() + self.jump_interval

            if not self.sleep_interruptible(self.move_pulse):
                self.release()
                self.last_stop_reason = "stopped"
                return False

            now = time.monotonic()
            if now - last_log_time >= 2.0:
                logger.debug(
                    "WaypointNavigator moving: target=%s current=%s distance=%.1f "
                    "angle_delta=%.1f turn=%.1f forward=%s",
                    target,
                    location.point,
                    distance,
                    angle_delta,
                    turn_degrees,
                    self.w_down,
                )
                last_log_time = now

            self.sleep_remaining(started)

        self.release()
        self.last_stop_reason = "stopped"
        return False

    def press_forward(self) -> None:
        now = time.monotonic()
        if self.w_down and now - self._last_w_down_at < self.key_refresh_interval:
            return
        self.controller.post_key_down(_KEY_W).wait()
        self.w_down = True
        self._last_w_down_at = now

    def release(self) -> None:
        self.turn_pid.reset()
        self._last_w_down_at = 0.0
        # 两个按键都要尽力松开：先空格后 W，任何一个失败都不影响另一个。
        try:
            if self.space_down:
                self.controller.post_key_up(_KEY_SPACE).wait()
        finally:
            self.space_down = False
            if self.w_down:
                try:
                    self.controller.post_key_up(_KEY_W).wait()
                finally:
                    self.w_down = False

    def close(self) -> None:
        try:
            self.release()
        finally:
            try:
                if self._owned_position_provider is not None:
                    self._owned_position_provider.close()
                    self._owned_position_provider = None
                self.position_provider = None
            finally:
                try:
                    if self.locator is not None:
                        self.locator.close()
                        self.locator = None
                finally:
                    try:
                        if self.predictor is not None:
                            self.predictor.close()
                            self.predictor = None
                    finally:
                        if self.debug:
                            close_debug_windows()

    def sleep_remaining(self, started: float) -> None:
        sleep_time = self.frame_interval - (time.perf_counter() - started)
        if sleep_time <= 0:
            return
        if not self.debug:
            time.sleep(sleep_time)
            return

        deadline = time.monotonic() + sleep_time
        while time.monotonic() < deadline:
            pump_debug_windows()
            time.sleep(min(0.05, abs(deadline - time.monotonic())))

    def sleep_interruptible(self, duration: float) -> bool:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self.context.tasker.stopping:
                return False
            if self.debug:
                pump_debug_windows()
            time.sleep(min(0.05, abs(deadline - time.monotonic())))
        return True


def load_params(custom_action_param: Any) -> dict[str, Any]:
    if not custom_action_param:
        return {}
    if isinstance(custom_action_param, dict):
        return custom_action_param
    try:
        params = json.loads(custom_action_param)
    except Exception as exc:
        logger.warning("Parse custom_action_param failed: %s", exc)
        return {}
    return params if isinstance(params, dict) else {}
