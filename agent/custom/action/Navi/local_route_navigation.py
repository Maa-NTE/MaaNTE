import json
import math
import time
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path
from typing import TYPE_CHECKING, Any

from maa.agent.agent_server import AgentServer
from maa.custom_action import CustomAction

if TYPE_CHECKING:
    from maa.context import Context

from ..Common.logger import get_logger
from .resource_paths import resource_base_path
from .route_model import (
    RouteSession,
    Waypoint,
    parse_route_segment_from_json_data,
    parse_waypoints_from_json_data,
)
from .waypoint_navigator import load_params

logger = get_logger(__name__)
DEFAULT_MAP_SIZE = (11264, 11264)
DEFAULT_TOLERANCE = 5.0
DEFAULT_FRAME_INTERVAL = 0.1
__all__ = [
    "LocalRouteNavigation",
    "LocalRouteNavigationAction",
    "RunNavigationTaskAction",
    "LocalRouteNavigationUnitTestAction",
    "parse_optional_bool",
    "parse_optional_duration",
    "parse_route_waypoints",
    "resolve_route_json_path",
]


RouteJson = str | Path | dict[str, Any] | list[Any]


def resolve_route_json_path(json_path: str | Path) -> Path:
    path = Path(json_path).expanduser()
    if path.exists():
        return path

    routes_dir = resource_base_path().parent / "routes"
    file_name = path.name
    if not file_name:
        raise ValueError("route json path is empty")
    if not file_name.lower().endswith(".json"):
        file_name = f"{file_name}.json"

    candidate = routes_dir / file_name
    if candidate.exists():
        return candidate

    raise FileNotFoundError(f"route json not found: {json_path}")


def parse_route_waypoints(
    route_json: Any,
    *,
    route_name: str = "",
    segment_index: int = 1,
    source_size: tuple[int, int] = DEFAULT_MAP_SIZE,
    target_size: tuple[int, int] = DEFAULT_MAP_SIZE,
) -> list[Waypoint]:
    if isinstance(route_json, dict):
        return parse_route_segment_from_json_data(
            route_json,
            route_name,
            segment_index,
            source_size,
            target_size,
        )
    return parse_waypoints_from_json_data(route_json, source_size, target_size)


def parse_optional_bool(value: Any, default: bool = False) -> bool:
    """把 Pipeline 参数解析成布尔值。

    bool("false") 的结果是 True，因此不能直接对字符串使用 bool()：否则在
    Pipeline 中写 "periodic_jump": "false" 反而会开启跳跃。
    """
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    text = str(value).strip().lower()
    if not text:
        return default
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
    raise ValueError(f"invalid boolean value: {value!r}")


def parse_optional_duration(value: Any) -> float | None:
    """把 Pipeline 参数解析成可选的超时秒数。

    接受 None 或空串表示「不限制时间」。NaN / inf 会让截止时间比较永远不成立、
    从而静默失去超时保护，因此这里直接拒绝；非正值同样视为非法。
    """
    if value is None:
        return None
    if isinstance(value, str) and not value.strip():
        return None
    duration = float(value)
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError(f"max_duration must be a positive finite number: {value!r}")
    return duration


class LocalRouteNavigation:
    """本地路线寻路器。

    一个实例持有一套 RouteRunner、定位器和方向推理器。需要在同一个函数中连续
    执行多个路线段时，先构造实例并加载整份路线 JSON，再多次调用 run_route()
    指定不同 segment，避免每段路线都重复初始化模型和定位资源。

    构造参数：
        tolerance: 到达判定距离阈值，默认 DEFAULT_TOLERANCE。
        angle_backend: 朝向推理后端，默认 "auto"。
        position_backend: 位置来源后端，默认 "map"。
        frame_interval: 每帧最小间隔（秒），默认 DEFAULT_FRAME_INTERVAL。
        debug: 是否开启调试窗口，默认 False。
        max_duration: 单个路径点的最长寻路时间（秒），默认 None 即不限时，
            与原行为一致；必须是有限的正常数。
        periodic_jump: 是否在寻路途中周期性跳跃（每 3 秒一次），默认 False，
            路径点少于 4 个或位于最后两个点时停用周期跳跃。

    Pipeline 写法（可选参数不填即保持原行为）：
        "LocalRouteNavigationDemo": {
            "recognition": "DirectHit",
            "action": "Custom",
            "custom_action": "local_route_navigation",
            "custom_action_param": {
                "json_path": "penquan",
                "route_name": "penquan",
                "segment_index": 1,
                "tolerance": 5.0,
                "frame_interval": 0.1,
                "max_duration": 30,
                "periodic_jump": true
            }
        }
    """

    def __init__(
        self,
        context: "Context",
        route_json: RouteJson | None = None,
        *,
        tolerance: float = DEFAULT_TOLERANCE,
        angle_backend: str = "auto",
        position_backend: str = "map",
        frame_interval: float = DEFAULT_FRAME_INTERVAL,
        debug: bool = False,
        max_duration: float | None = None,
        periodic_jump: bool = False,
    ) -> None:
        from .route_runner import RouteRunner

        self.context = context
        self.route = RouteSession()
        self.runner = RouteRunner(
            context,
            self.route,
            angle_backend=angle_backend,
            position_backend=position_backend,
            tolerance=tolerance,
            frame_interval=frame_interval,
            debug=debug,
            max_duration=max_duration,
            periodic_jump=periodic_jump,
        )
        self._route_json: Any | None = None
        self._route_json_path: Path | None = None
        self._closed = False

        if route_json is not None:
            self.load_route_json(route_json)

    def load_route_json(self, route_json: RouteJson) -> None:
        if isinstance(route_json, (dict, list)):
            self._route_json = route_json
            self._route_json_path = None
            return

        route_json_path = resolve_route_json_path(route_json)
        with route_json_path.open("r", encoding="utf-8") as file:
            self._route_json = json.load(file)
        self._route_json_path = route_json_path

    def select_route_file(self, json_path: str | Path) -> None:
        path = resolve_route_json_path(json_path)
        if self._route_json_path != path:
            self.load_route_json(path)

    def run_route(
        self,
        route_json: RouteJson | None = None,
        *,
        route_name: str = "",
        segment_index: int = 1,
        jump_when_stuck: bool = False,
    ) -> bool:
        """执行路线段。jump_when_stuck 允许在坐标定位确认近距离卡住时补跳一次。"""
        if self._closed:
            raise RuntimeError("local route navigation is closed")

        segment_started_at = time.time()
        self.runner.start()
        data, route_label = self._route_json_for_run(route_json)
        if data is None:
            raise ValueError("route json is not loaded")

        waypoints = parse_route_waypoints(
            data,
            route_name=route_name,
            segment_index=segment_index,
            source_size=DEFAULT_MAP_SIZE,
            target_size=self.runner.source_size(),
        )
        if not waypoints:
            logger.warning(
                "OnlineMapNavigation local route is empty: path=%s route=%s segment=%s",
                route_label,
                route_name,
                segment_index,
            )
            return False

        current_point = self.runner.update_current_frame(since=segment_started_at)
        if self.context.tasker.stopping:
            return False
        self.route.reset(waypoints, True, current_point)
        logger.info(
            "OnlineMapNavigation local route loaded: path=%s route=%s segment=%s "
            "waypoints=%s current=%s start_index=%s",
            route_label,
            route_name,
            segment_index,
            len(waypoints),
            current_point,
            self.route.payload()["currentIndex"],
        )
        status = self.runner.run_until_stopped(
            stop_when_route_done=True, jump_when_stuck=jump_when_stuck
        )
        return status == "arrived"

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.runner.close()

    def __enter__(self) -> "LocalRouteNavigation":
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def _route_json_for_run(
        self,
        route_json: RouteJson | None,
    ) -> tuple[Any | None, str | Path]:
        if route_json is None:
            return self._route_json, self._route_json_path or "<memory>"
        if isinstance(route_json, (dict, list)):
            return route_json, "<memory>"
        route_path = resolve_route_json_path(route_json)
        with route_path.open("r", encoding="utf-8") as file:
            return json.load(file), route_path


# MaaFramework 在进程内统一分配任务号，嵌套 run_task 的动作回调沿用外层任务号。
_task_navigators: dict[int, LocalRouteNavigation] = {}


def get_task_navigation(task_id: int) -> LocalRouteNavigation | None:
    return _task_navigators.get(task_id)


@contextmanager
def _navigation_task(
    context: "Context", task_id: int
) -> Iterator[LocalRouteNavigation]:
    navigator = get_task_navigation(task_id)
    if navigator is not None:
        yield navigator
        return

    # run_task 是同步调用。由外层回调持有 Context，内层节点返回后它仍然有效。
    with LocalRouteNavigation(context) as navigator:
        _task_navigators[task_id] = navigator
        try:
            yield navigator
        finally:
            del _task_navigators[task_id]


@AgentServer.custom_action("run_navigation_task")
class RunNavigationTaskAction(CustomAction):
    def run(
        self, context: "Context", argv: CustomAction.RunArg
    ) -> CustomAction.RunResult:
        try:
            entry = load_params(argv.custom_action_param).get("entry")
            if not isinstance(entry, str) or not entry or entry == argv.node_name:
                raise ValueError("entry must reference a different pipeline node")
            with _navigation_task(context, argv.task_detail.task_id):
                result = context.run_task(entry)
                return CustomAction.RunResult(
                    success=result is not None and result.status.succeeded
                )
        except Exception:
            logger.exception("Navigation task failed: %s", argv.node_name)
            return CustomAction.RunResult(success=False)


@AgentServer.custom_action("local_route_navigation")
class LocalRouteNavigationAction(CustomAction):
    @staticmethod
    def route_params(custom_action_param: Any) -> dict[str, Any]:
        return load_params(custom_action_param)

    def run(
        self, context: "Context", argv: CustomAction.RunArg
    ) -> CustomAction.RunResult:
        try:
            params = self.route_params(argv.custom_action_param)
            json_path = params.get("json_path")
            if not json_path:
                raise ValueError("json_path is required")

            route_name = str(params.get("route_name", "")).strip()
            segment_index = int(params.get("segment_index", 1))
            tolerance = float(params.get("tolerance", DEFAULT_TOLERANCE))
            frame_interval = max(
                0.05,
                float(params.get("frame_interval", DEFAULT_FRAME_INTERVAL)),
            )
            angle_backend = str(params.get("angle_backend", "auto")).strip() or "auto"
            position_backend = (
                str(params.get("position_backend", "map")).strip() or "map"
            )
            debug = bool(params.get("debug", False))
            max_duration = parse_optional_duration(params.get("max_duration"))
            periodic_jump = parse_optional_bool(params.get("periodic_jump"))

        except (TypeError, ValueError) as exc:
            logger.error("LocalRouteNavigation param invalid: %s", exc)
            return CustomAction.RunResult(success=False)

        try:
            with _navigation_task(context, argv.task_detail.task_id) as navigator:
                navigator.select_route_file(json_path)
                navigator.runner.configure(
                    tolerance=tolerance,
                    frame_interval=frame_interval,
                    angle_backend=angle_backend,
                    position_backend=position_backend,
                    debug=debug,
                    max_duration=max_duration,
                    periodic_jump=periodic_jump,
                )
                success = navigator.run_route(
                    route_name=route_name,
                    segment_index=segment_index,
                )
            return CustomAction.RunResult(success=success)
        except Exception as exc:
            logger.error("LocalRouteNavigation failed: %s", exc)
            return CustomAction.RunResult(success=False)


@AgentServer.custom_action("local_route_navigation_unit_test")
class LocalRouteNavigationUnitTestAction(LocalRouteNavigationAction):
    @staticmethod
    def route_params(custom_action_param: Any) -> dict[str, Any]:
        return {
            **load_params(custom_action_param),
            "json_path": "penquan",
            "route_name": "penquan",
            "segment_index": 1,
            "max_duration": None,
            "periodic_jump": False,
        }
