import json
import time
from typing import Any

from maa.agent.agent_server import AgentServer
from maa.context import Context
from maa.custom_action import CustomAction
from maa.pipeline import JOCR, JRecognitionType

from utils.logger import logger

from .Navi import LocalRouteNavigation
from .Navi.resource_paths import resource_base_path
from .MapTeleport.teleport_to_point import run_map_teleport_flow

# ---------------- 常量 ----------------
ROUTE_FILE = "wallet"           # 路线文件名
VK_F = 0x46                     # F 键的 Windows 虚拟键码
VK_ESC = 27                     # Esc 键的 Windows 虚拟键码

# OCR 参数
OCR_ROI = (734, 263, 316, 254)                  # 拾取提示文字的 ROI（1280×720 基准）
POSITIVES = ["团三郎", "咕比", "劲爽", "再来一口", "馈赠", "一箱", "猎人", "心猎骑士"]   # 正期望词表
NEGATIVES = ["售货机", "坐下", "驾驶", "乘坐", "搭乘", "安魂曲"]                    # 负期望词表
OCR_THRESHOLD = 0.5                              # OCR 命中阈值
OCR_SLOW_SECONDS = 2.0                           # 超过此耗时记录单帧识别延迟

# 拾取节奏
PRESS_DELAY = 0.5               # F 键模式下，相邻两次按 F 之间的间隔
WAIT_AFTER_ROUND1 = 1.5         # 第一轮结束到第二轮开始之间的等待
WAIT_AFTER_ALT_CLICK = 1.5      # alt_click 模式下，相邻两次点击间隔

# 寻路后等待
WAIT_AFTER_NAVIGATE = 1.5       # 寻路成功后到第一次识别之间的等待

# 传送等待
WAIT_WORLD_TIMEOUT = 40.0       # 传送后等待回到大世界的最长时间
WAIT_WORLD_INTERVAL = 1.0       # InWorld 轮询间隔

# 截图等待：Maa 的 Job.wait() 没有超时参数，直接阻塞会连任务停止信号都收不到，
# 因此改为轮询作业状态，超时即放弃这一帧交给调用方重试。
SCREENCAP_TIMEOUT = 10.0        # 单次截图的最长等待
SCREENCAP_POLL_INTERVAL = 0.02  # 截图作业状态轮询间隔

# 失败后的界面恢复
WAIT_IN_WORLD_PROBE = 3.0       # 单次 InWorld 探测的最长等待
WAIT_AFTER_MAP_CLOSE = 0.8      # 按下 Esc 后等待界面切换
MAP_CLOSE_RETRIES = 3           # 最多按几次 Esc

# 寻路参数
NAV_MAX_DURATION = 30           # 每个路径点的寻路超时（秒）
NAV_TOLERANCE = 2               # 到达判定的距离阈值

# 路线文件校验
MAX_ROUTE_PROBLEMS = 50         # 校验失败时最多打印多少条问题


# ============================================================
# 资源读取
# ============================================================
def _load_route_json() -> dict:
    routes_dir = resource_base_path().parent / "routes"
    file_path = routes_dir / f"{ROUTE_FILE}.json"
    with file_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _load_teleport_name(teleport_id: str) -> str:
    points_file = resource_base_path() / "map_teleport" / "teleport_points.json"
    try:
        with points_file.open("r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return teleport_id
    for p in data.get("teleport_points", []):
        if p.get("id") == teleport_id:
            return p.get("name", teleport_id)
    return teleport_id


def _load_teleport_ids() -> set[str]:
    """读取 teleport_points.json 中的全部传送点 id，用于校验路线引用。"""
    points_file = resource_base_path() / "map_teleport" / "teleport_points.json"
    with points_file.open("r", encoding="utf-8") as f:
        data = json.load(f)
    points = data.get("teleport_points") if isinstance(data, dict) else None
    if not isinstance(points, list):
        return set()
    return {str(p.get("id")) for p in points if isinstance(p, dict) and p.get("id")}


_POINT_COORD_KEYS = ("x", "y", "pixelX", "pixelY", "lat", "lng", "target_x", "target_y")


def _validate_routes(routes: list[dict]) -> list[str]:
    """校验路线文件结构，返回问题列表；空列表表示通过。

    寻路组件按 name/id 取第一条匹配的路线，且 name 为空时会回退到 routes[0]，
    所以「重名」和「空名」都不会抛异常，只会安静地跑错路线。这里提前拦下来，
    并在日志里指明是第几条路线、第几个点位。
    """
    problems: list[str] = []
    try:
        known_tp: set[str] | None = _load_teleport_ids()
    except Exception:
        logger.warning(
            "[PickWallet] 读取 teleport_points.json 失败，跳过传送点引用校验",
            exc_info=True,
        )
        known_tp = None

    names: dict[str, int] = {}
    ids: dict[str, int] = {}
    for r_idx, route in enumerate(routes, 1):
        if not isinstance(route, dict):
            problems.append("R%s 不是对象" % r_idx)
            continue

        name = str(route.get("name") or "").strip()
        route_id = str(route.get("id") or "").strip()
        if not name:
            problems.append("R%s 缺少 name（会导致导航回退到第 1 条路线）" % r_idx)
        elif name in names:
            problems.append(
                "R%s 的 name=%r 与 R%s 重复（只会命中第一条）"
                % (r_idx, name, names[name])
            )
        else:
            names[name] = r_idx
        if route_id:
            if route_id in ids:
                problems.append(
                    "R%s 的 id=%r 与 R%s 重复" % (r_idx, route_id, ids[route_id])
                )
            else:
                ids[route_id] = r_idx

        segments = route.get("segments")
        if not isinstance(segments, list) or not segments:
            problems.append(
                "R%s(%s) 的 segments 为空" % (r_idx, name or route_id or "无名字")
            )
            continue

        for p_idx, seg in enumerate(segments, 1):
            if not isinstance(seg, dict):
                problems.append("R%sP%s 不是对象" % (r_idx, p_idx))
                continue

            points = seg.get("points")
            if not isinstance(points, list) or not points:
                problems.append(
                    "R%sP%s 的 points 为空，寻路会直接失败" % (r_idx, p_idx)
                )
            else:
                for k_idx, point in enumerate(points, 1):
                    if not isinstance(point, dict) or not any(
                        key in point for key in _POINT_COORD_KEYS
                    ):
                        problems.append(
                            "R%sP%s 第 %s 个路径点没有坐标字段"
                            % (r_idx, p_idx, k_idx)
                        )
                        break

            if seg.get("need_teleport"):
                tp_id = str(seg.get("teleport_point_id") or "").strip()
                if not tp_id:
                    problems.append(
                        "R%sP%s 标记了 need_teleport 但没有 teleport_point_id"
                        % (r_idx, p_idx)
                    )
                elif known_tp is not None and tp_id not in known_tp:
                    problems.append(
                        "R%sP%s 引用了不存在的传送点 %r" % (r_idx, p_idx, tp_id)
                    )

    for token in sorted(set(names) & set(ids)):
        if names[token] != ids[token]:
            problems.append(
                "%r 同时是 R%s 的 name 和 R%s 的 id，会互相抢占匹配"
                % (token, names[token], ids[token])
            )

    return problems


# ============================================================
# 通用工具
# ============================================================
def _box_to_xywh(box: Any):
    if box is None:
        return None
    if isinstance(box, (list, tuple)) and len(box) >= 4:
        return int(box[0]), int(box[1]), int(box[2]), int(box[3])
    if all(hasattr(box, a) for a in ("x", "y", "w", "h")):
        return int(box.x), int(box.y), int(box.w), int(box.h)
    return None


def _screencap(context: Context, timeout: float = SCREENCAP_TIMEOUT):
    """截取一帧，等待时间受限，截图卡死时返回 None。

    框架的 Job.wait() 会一直阻塞到作业结束，截图请求卡住时既无法响应任务停止
    信号，会让上层所有超时判断失效，所以这里改成轮询作业状态。
    """
    # 截图作业通过同一个远程控制器代理查询状态和读取图像。
    controller = context.tasker.controller
    job = controller.post_screencap()
    deadline = time.monotonic() + max(timeout, 0.0)
    while not job.done:
        if context.tasker.stopping:
            return None
        if time.monotonic() >= deadline:
            logger.warning("截图超时，放弃本次截图")
            return None
        time.sleep(SCREENCAP_POLL_INTERVAL)
    return job.get()


def _sleep_until(deadline: float, interval: float) -> None:
    """睡一小步，但不超过剩余时间。"""
    time.sleep(max(0.0, min(interval, deadline - time.monotonic())))


def _wait_in_world(context: Context, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if context.tasker.stopping:
            return False
        remaining = deadline - time.monotonic()
        if remaining < SCREENCAP_POLL_INTERVAL:
            break
        # 单次截图既不能超过剩余的轮询时间，也不能超过截图自身的上限，
        # 否则一次卡死的截图就会吃掉整个等待预算。
        frame = _screencap(context, min(SCREENCAP_TIMEOUT, remaining))
        if frame is None:
            _sleep_until(deadline, WAIT_WORLD_INTERVAL)
            continue
        try:
            result = context.run_recognition("InWorld", frame)
        except Exception:
            logger.warning("InWorld 识别异常", exc_info=True)
            _sleep_until(deadline, WAIT_WORLD_INTERVAL)
            continue
        if result and getattr(result, "hit", False):
            return True
        _sleep_until(deadline, WAIT_WORLD_INTERVAL)
    return False


def _restore_world_ui(context: Context) -> bool:
    """尽量把游戏界面还原回大世界。

    传送流程会在「地图」「地图索引」等中间界面失败，此时若直接结束任务，游戏会
    停在半开的界面里。这里每按一次 Esc 就探测一次是否已回到大世界，避免多按把
    暂停菜单按出来。
    """
    if context.tasker.stopping:
        return False
    if _wait_in_world(context, WAIT_IN_WORLD_PROBE):
        return True
    for _ in range(MAP_CLOSE_RETRIES):
        if context.tasker.stopping:
            return False
        try:
            _press_key(context, VK_ESC)
        except Exception:
            logger.warning("恢复界面时按下 Esc 失败", exc_info=True)
            return False
        time.sleep(WAIT_AFTER_MAP_CLOSE)
        if _wait_in_world(context, WAIT_IN_WORLD_PROBE):
            return True
    logger.warning("未能自动恢复到大世界界面，请手动确认游戏画面")
    return False


# ============================================================
# OCR
# ============================================================
def _read_pickup_prompts(
    context: Context, frame
) -> tuple[bool, list[tuple[int, int, int, int]]]:
    """从同一帧读出其他交互提示和可拾取目标的点击框。"""
    started = time.monotonic()
    detail = context.run_recognition_direct(
        JRecognitionType.OCR,
        JOCR(expected=POSITIVES + NEGATIVES, roi=OCR_ROI, threshold=OCR_THRESHOLD),
        frame,
    )
    elapsed = time.monotonic() - started
    if elapsed >= OCR_SLOW_SECONDS:
        logger.warning("[PickWallet] 拾取提示单帧 OCR 耗时 %.2f 秒", elapsed)
    if detail is None or not detail.hit:
        return False, []

    has_negative = False
    boxes = []
    for item in detail.filtered_results or []:
        text = getattr(item, "text", "") or ""
        if any(word in text for word in NEGATIVES):
            has_negative = True
        if any(word in text for word in POSITIVES):
            box = _box_to_xywh(getattr(item, "box", None))
            if box is not None and box not in boxes:
                boxes.append(box)
    return has_negative, boxes


# ============================================================
# 按键 / alt_click
# ============================================================
def _press_key(context: Context, key: int) -> None:
    node = "__PickWalletClickKey"
    context.run_action(
        node,
        pipeline_override={
            node: {
                "action": {"type": "ClickKey", "param": {"key": [int(key)]}},
                "pre_delay": 0, "post_delay": 0, "rate_limit": 0,
            }
        },
    )


def _press_f(context: Context) -> None:
    _press_key(context, VK_F)


def _alt_click_box(context: Context, box: tuple[int, int, int, int]) -> None:
    context.run_action("__AltClickAltKeyDownAction")
    try:
        context.run_action(
            "__AltClickMouseClickAction",
            pipeline_override={
                "__AltClickMouseClickAction": {
                    "action": {"param": {"target": list(box)}}
                },
            },
        )
    finally:
        # 点击失败也必须松开 Alt，否则后续所有输入都会带上 Alt 修饰
        context.run_action("__AltClickAltKeyUpAction")


# ============================================================
# 拾取流程
# ============================================================
def _do_pick_round(context: Context, round_name: str) -> bool:
    frame = _screencap(context)
    if frame is None:
        return False
    has_negative, boxes = _read_pickup_prompts(context, frame)

    logger.info(
        "%s识别，正期望%s个，负期望%s个", round_name, len(boxes), int(has_negative)
    )

    if not boxes:
        logger.info("无正期望，该点位结束")
        return False

    if has_negative:
        logger.info("alt_click模式，依次点击拾取")
        for _ in range(len(boxes)):
            if context.tasker.stopping:
                return False
            frame = _screencap(context)
            if frame is None:
                break
            _, current_boxes = _read_pickup_prompts(context, frame)
            if not current_boxes:
                logger.info("未识别到正期望，提前结束 alt_click")
                break
            if context.tasker.stopping:
                return False
            _alt_click_box(context, current_boxes[0])
            time.sleep(WAIT_AFTER_ALT_CLICK)
        return False

    logger.info("F键模式，按下%s次", len(boxes))
    for _ in range(len(boxes)):
        if context.tasker.stopping:
            return False
        _press_f(context)
        time.sleep(PRESS_DELAY)
    return True


def _pick_wallet(context: Context) -> None:
    if not _do_pick_round(context, "第一轮"):
        return
    time.sleep(WAIT_AFTER_ROUND1)
    _do_pick_round(context, "第二轮")


# ============================================================
# 传送
# ============================================================
def _do_teleport(context: Context, tp_id: str) -> bool:
    try:
        ok = bool(run_map_teleport_flow(context, tp_id))
    except Exception:
        logger.error("传送异常", exc_info=True)
        ok = False
    if ok:
        return True
    # 失败时流程可能停在地图 / 地图索引等中间界面，先还原界面再交给上层处理
    if not context.tasker.stopping:
        _restore_world_ui(context)
    return False


# ============================================================
# 回溯 / 跳点目标查找
# ============================================================
def _find_backtrack_target(segments: list[dict], current_p: int) -> int:
    """向前找（含当前点）最近的传送点"""
    for p in range(current_p, 0, -1):
        if segments[p - 1].get("need_teleport", False):
            return p
    return 1


def _find_forward_teleport_target(segments: list[dict], current_p: int) -> int | None:
    """向后找（不含当前点）最近的传送点"""
    total = len(segments)
    for p in range(current_p + 1, total + 1):
        if segments[p - 1].get("need_teleport", False):
            return p
    return None


# ============================================================
# 主循环
# ============================================================
@AgentServer.custom_action("pick_wallet_run_all")
class PickWalletRunAll(CustomAction):
    def __init__(self):
        super().__init__()

    def run(self, context: Context, argv: CustomAction.RunArg) -> CustomAction.RunResult:
        try:
            route_json = _load_route_json()
        except Exception:
            logger.error("[PickWallet] 读取 wallet.json 失败", exc_info=True)
            return CustomAction.RunResult(success=False)

        routes = route_json.get("routes") if isinstance(route_json, dict) else None
        if not isinstance(routes, list) or not routes:
            logger.error("[PickWallet] wallet.json 中没有任何路线")
            return CustomAction.RunResult(success=False)

        try:
            with LocalRouteNavigation(
                context,
                route_json,
                tolerance=NAV_TOLERANCE,
                angle_backend="auto",
                position_backend="coordinate",
                frame_interval=0.1,
                max_duration=NAV_MAX_DURATION,
                periodic_jump=True,
            ) as navigator:
                return self._run_task(context, routes, navigator)
        except Exception:
            logger.error("[PickWallet] 任务异常终止", exc_info=True)
            return CustomAction.RunResult(success=False)

    def _run_task(
        self, context: Context, routes: list[dict], navigator: LocalRouteNavigation
    ) -> CustomAction.RunResult:
        # 使用 attach 字段绕过 Maa 的字段校验，杜绝 UI 覆盖丢失的问题。
        start_route = int(context.get_node_data("PickWalletCfg_StartRoute")["attach"]["val"])
        limit_count = bool(context.get_node_data("PickWalletCfg_LimitCount")["attach"]["val"])
        enable_backtrack = bool(context.get_node_data("PickWalletCfg_EnableBacktrack")["attach"]["val"])

        # route_count 是「本次要跑几条路线」，只在开启「限制执行路线数量」时生效；
        # 路线总数一律用 len(routes) 动态读取，所以 else 里的 1 只是占位值。
        if limit_count:
            route_count = int(context.get_node_data("PickWalletCfg_RouteCount")["attach"]["val"])
        else:
            route_count = 1

        # 先校验结构：重名 / 空名 / 缺坐标 / 引用不存在的传送点都会让寻路静默跑错，
        # 必须在开始传送之前拦住。
        problems = _validate_routes(routes)
        if problems:
            logger.error(
                "[PickWallet] 路线文件校验失败，共 %s 处问题：", len(problems)
            )
            for item in problems[:MAX_ROUTE_PROBLEMS]:
                logger.error("[PickWallet]   - %s", item)
            if len(problems) > MAX_ROUTE_PROBLEMS:
                logger.error(
                    "[PickWallet]   ... 其余 %s 处未显示",
                    len(problems) - MAX_ROUTE_PROBLEMS,
                )
            return CustomAction.RunResult(success=False)

        route_start_globals = []
        acc = 0
        for route in routes:
            route_start_globals.append(acc + 1)
            acc += len(route.get("segments", []))
        total_segments = acc

        logger.info(
            "[PickWallet] 已加载 %s 条路线，共 %s 个点位", len(routes), total_segments
        )
        if limit_count:
            logger.info(
                "[PickWallet] 限制条件：从 R%s 开始，往后执行 %s 条路线",
                start_route,
                route_count,
            )
        else:
            logger.info(
                "[PickWallet] 执行条件：从 R%s 开始，一直跑到最后一条路线", start_route
            )
        logger.info(
            "[PickWallet] 寻路失败回溯功能：%s", "开启" if enable_backtrack else "关闭"
        )

        # 起点非法时走正常失败返回，不抛异常中断动作
        if start_route < 1 or start_route > len(routes):
            logger.error(
                "[PickWallet] start_route=%s 非法（应在 1~%s 之间）",
                start_route,
                len(routes),
            )
            return CustomAction.RunResult(success=False)

        # 被寻路失败跳过的点位；只要非空就说明路线不完整，任务不能报成功。
        skipped_points: list[int] = []
        processed_routes = 0  # 记录本次任务已开始处理的路线数量

        # ---- 遍历路线 ----
        for r_idx in range(start_route, len(routes) + 1):
            if context.tasker.stopping:
                return CustomAction.RunResult(success=True)

            # 检查“限制执行路线数量”开关
            if limit_count and processed_routes >= route_count:
                logger.info(
                    "[PickWallet] 已达到设定的执行路线总数 (%s)，任务结束。",
                    route_count,
                )
                break

            route = routes[r_idx - 1]
            segments = route.get("segments", [])
            if not segments:
                continue

            processed_routes += 1  # 只要开始处理（非空路线），计数就 +1

            route_name = route.get("name", "")
            route_start_global = route_start_globals[r_idx - 1]
            backtracked = False
            retry_target_p = None
            p_idx = 1

            while p_idx <= len(segments):
                if context.tasker.stopping:
                    return CustomAction.RunResult(success=True)

                seg = segments[p_idx - 1]
                need_tp = bool(seg.get("need_teleport", False))
                tp_id = seg.get("teleport_point_id", "") if need_tp else ""
                global_idx = route_start_global + p_idx - 1

                logger.info("第%s路线第%s点位，总第%s点位", r_idx, p_idx, global_idx)

                # ---------- 传送 ----------
                if need_tp:
                    tp_name = _load_teleport_name(tp_id)
                    logger.info("准备使用地图传送，%s", tp_name)

                    if not _do_teleport(context, tp_id):
                        if context.tasker.stopping:
                            return CustomAction.RunResult(success=True)
                        logger.error("R%sP%s 传送失败，任务结束", r_idx, p_idx)
                        return CustomAction.RunResult(success=False)

                    if not _wait_in_world(context, WAIT_WORLD_TIMEOUT):
                        if context.tasker.stopping:
                            return CustomAction.RunResult(success=True)
                        logger.error(
                            "R%sP%s 传送后未回到大世界，任务结束", r_idx, p_idx
                        )
                        return CustomAction.RunResult(success=False)

                    logger.info("传送完成，开始寻路")
                else:
                    logger.info("开始寻路")

                # ---------- 寻路 ----------
                try:
                    navigate_ok = navigator.run_route(
                        route_name=route_name, segment_index=p_idx, jump_when_stuck=True
                    )
                except Exception:
                    # 寻路异常不能当成普通超时继续跳点，否则路线不完整还会报成功
                    logger.error("R%sP%s 寻路异常", r_idx, p_idx, exc_info=True)
                    return CustomAction.RunResult(success=False)

                if not navigate_ok:
                    if context.tasker.stopping:
                        return CustomAction.RunResult(success=True)

                    # 1. 如果开启了回溯，且本路线未回溯过，执行向前回溯
                    if enable_backtrack and not backtracked:
                        target_p = _find_backtrack_target(segments, p_idx)
                        backtracked = True
                        target_global = route_start_global + target_p - 1
                        logger.info(
                            "寻路超时，回溯到当前路线第%s点位，总第%s点位",
                            target_p,
                            target_global,
                        )
                        retry_target_p = p_idx
                        p_idx = target_p
                        continue

                    # 2. 其他情况（未开启回溯，或已回溯过，或向前回溯失败），进入向后跳点逻辑
                    retry_target_p = None  # 强制清空追赶状态
                    forward_tp = _find_forward_teleport_target(segments, p_idx)

                    if forward_tp is not None:
                        target_global = route_start_global + forward_tp - 1
                        skipped_points.extend(
                            route_start_global + i - 1
                            for i in range(p_idx, forward_tp)
                        )
                        logger.info(
                            "寻路超时，放弃中间点位，跳至下一个传送点："
                            "第%s点位，总第%s点位",
                            forward_tp,
                            target_global,
                        )
                        p_idx = forward_tp
                        continue

                    skipped_points.extend(
                        route_start_global + i - 1
                        for i in range(p_idx, len(segments) + 1)
                    )
                    logger.warning("后续无可用传送点，放弃当前路线剩余点位")
                    break

                # ---------- 寻路成功，判断是否拾取 ----------
                should_pickup = (retry_target_p is None) or (p_idx >= retry_target_p)
                if should_pickup:
                    time.sleep(WAIT_AFTER_NAVIGATE)
                    try:
                        _pick_wallet(context)
                    except Exception:
                        logger.error("R%sP%s 拾取失败", r_idx, p_idx, exc_info=True)
                        return CustomAction.RunResult(success=False)
                    if retry_target_p is not None and p_idx >= retry_target_p:
                        retry_target_p = None

                p_idx += 1

        if skipped_points:
            logger.warning(
                "[PickWallet] 有 %s 个点位因寻路失败未处理，任务按未完成处理",
                len(skipped_points),
            )
            return CustomAction.RunResult(success=False)

        logger.info("[PickWallet] 全部点位处理完成，任务结束")
        return CustomAction.RunResult(success=True)
