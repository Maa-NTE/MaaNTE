import sys
import time
import unittest
from contextlib import ExitStack
from importlib import import_module
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, call, patch, sentinel


def load_wallet():
    # 单独加载钱包动作。导入 maa.agent 会切换整个进程的原生库。
    agent_dir = Path(__file__).resolve().parents[1] / "agent"
    package = ModuleType("_wallet_actions")
    package.__path__ = [str(agent_dir / "custom" / "action")]
    sys.modules[package.__name__] = package
    module_name = "maa.agent.agent_server"
    stub = ModuleType(module_name)
    stub.AgentServer = SimpleNamespace(custom_action=lambda name: lambda cls: cls)
    previous = sys.modules.get(module_name)
    sys.modules[module_name] = stub
    try:
        with patch.object(sys, "path", [str(agent_dir), *sys.path]):
            return import_module("_wallet_actions.PickWalletRunAll")
    finally:
        if previous is None:
            del sys.modules[module_name]
        else:
            sys.modules[module_name] = previous


wallet = load_wallet()


def recognition(*prompts):
    results = [SimpleNamespace(text=text, box=box) for text, box in prompts]
    return SimpleNamespace(hit=bool(results), filtered_results=results)


class WalletPickupTest(unittest.TestCase):
    def setUp(self):
        self.context = Mock()
        self.context.tasker.stopping = False
        self.ocr = self.context.run_recognition_direct
        patches = ExitStack()
        self.addCleanup(patches.close)
        for name in ("_screencap", "_press_f", "_alt_click_box", "logger"):
            patches.enter_context(patch.object(wallet, name))
        patches.enter_context(patch.object(wallet.time, "sleep"))

    def test_empty_frame_runs_one_ocr_for_both_word_lists(self):
        wallet._screencap.return_value = sentinel.frame
        self.ocr.return_value = recognition()
        self.assertFalse(wallet._do_pick_round(self.context, "第一轮"))
        self.ocr.assert_called_once()
        _, params, frame = self.ocr.call_args.args
        self.assertIs(frame, sentinel.frame)
        self.assertEqual(set(params.expected), set(wallet.POSITIVES + wallet.NEGATIVES))
        wallet._press_f.assert_not_called()
        wallet._alt_click_box.assert_not_called()

    def test_duplicate_wallet_box_only_presses_f_once(self):
        box = (798, 383, 103, 16)
        self.ocr.return_value = recognition(("团三郎的恶作剧", box), ("团三郎", box))
        self.assertTrue(wallet._do_pick_round(self.context, "第一轮"))
        self.ocr.assert_called_once()
        wallet._press_f.assert_called_once_with(self.context)

    def test_mixed_prompts_click_current_wallet_boxes_only(self):
        seat = ("坐下", (800, 420, 40, 18))
        first = ("团三郎", (798, 380, 90, 18))
        second = ("咕比", (798, 400, 75, 18))
        moved = ("咕比", (798, 383, 75, 18))
        self.ocr.side_effect = [
            recognition(seat, first, second),
            recognition(seat, first, second),
            recognition(seat, moved),
        ]
        self.assertFalse(wallet._do_pick_round(self.context, "第一轮"))
        self.assertEqual(self.ocr.call_count, wallet._screencap.call_count)
        self.assertEqual(
            wallet._alt_click_box.call_args_list,
            [call(self.context, first[1]), call(self.context, moved[1])],
        )
        wallet._press_f.assert_not_called()


class WalletRoutePositionTest(unittest.TestCase):
    def setUp(self):
        core = import_module("_wallet_actions.Navi.nte_coordinate_api")
        positioning = import_module("_wallet_actions.Navi.coordinate_position")
        steering = import_module("_wallet_actions.Navi.waypoint_navigator")
        self.capture = core.CoordinateCapture(refresh_rate=0)
        self.capture._decoder = Mock()
        self.enterContext(patch.object(core.CoordinateCapture, "start"))
        self.create_capture = self.enterContext(
            patch.object(positioning, "_create_capture", return_value=self.capture)
        )
        self.enterContext(patch.object(steering, "PrintT"))
        self.now = 1000.0
        self.enterContext(patch.object(time, "time", lambda: self.now))
        self.enterContext(patch.object(time, "monotonic", lambda: self.now))
        self.enterContext(patch.object(time, "perf_counter", lambda: self.now))
        self.context = Mock()
        self.context.tasker.stopping = False
        self.navigation = wallet.LocalRouteNavigation(
            self.context,
            [{"x": x, "y": 0} for x in (0, 1000, 2000)],
            position_backend="coordinate",
            max_duration=0.1,
        )
        self.addCleanup(self.navigation.close)

    def publish_pose(self, x, captured_at):
        self.capture._decoder.decode.return_value = (x, 0, 0, 0, 90)
        flow = ("127.0.0.1", 1234, "127.0.0.2", 5678, "udp")
        self.capture._accept_packet(b"pose", captured_at, flow)

    def test_selects_nearest_waypoint_from_post_teleport_sample(self):
        before_teleport = self.now
        self.publish_pose(2000, before_teleport)
        self.enterContext(
            patch.object(
                self.navigation.runner, "run_until_stopped", return_value="arrived"
            )
        )
        with patch.object(time, "sleep", side_effect=AssertionError("已有本段坐标")):
            self.assertTrue(self.navigation.run_route())
        self.assertEqual(self.navigation.route.current_index, 2)

        self.now += 0.1
        samples = iter((2000, 1000))

        def deliver_sample(seconds):
            self.now += seconds
            x = next(samples)
            self.publish_pose(x, before_teleport if x == 2000 else self.now)

        with patch.object(time, "sleep", side_effect=deliver_sample) as waits:
            self.assertTrue(self.navigation.run_route())
        self.assertEqual(waits.call_count, 2)
        self.assertEqual(self.navigation.route.current_index, 1)
        self.create_capture.assert_called_once()

    def test_only_delayed_packets_keep_route_at_start_without_steering(self):
        before_teleport = self.now - 0.1
        self.publish_pose(2000, before_teleport)
        self.assertIsNotNone(self.navigation.runner.update_current_frame())

        def deliver_old_sample(seconds):
            self.now += seconds
            self.publish_pose(2000, before_teleport)

        with patch.object(time, "sleep", side_effect=deliver_old_sample):
            self.assertFalse(self.navigation.run_route())
        self.assertEqual(self.navigation.route.current_index, 0)
        self.assertEqual(self.navigation.route.status, "timeout")
        controller = self.context.clone.return_value.tasker.controller
        controller.post_key_down.assert_not_called()
        controller.post_relative_move.assert_not_called()

    def test_stop_interrupts_wait_for_first_coordinate(self):
        def stop(_seconds):
            self.context.tasker.stopping = True

        with patch.object(time, "sleep", side_effect=stop) as wait:
            self.assertFalse(self.navigation.run_route())
        wait.assert_called_once()
        self.assertFalse(self.navigation.route.active)
        self.context.clone.return_value.tasker.controller.post_key_down.assert_not_called()

    def test_teleport_check_uses_visual_map_when_shared_capture_is_stale(self):
        teleport = import_module("_wallet_actions.MapTeleport.check_teleport_required")
        runner = self.navigation.runner
        runner.start()
        coordinate = runner.get_position_provider("auto")
        visual = runner.get_position_provider("map")
        self.assertIs(coordinate, runner.get_position_provider("coordinate"))
        self.assertIsNot(coordinate, visual)
        self.assertIsNone(runner.navigator.locator)
        self.publish_pose(2000, self.now)
        self.now += 2.0
        get_frame = Mock(return_value=sentinel.frame)
        locator = Mock()
        locator.return_value.locate.return_value = SimpleNamespace(
            found=True, point=(20, 30)
        )
        with (
            patch.object(teleport, "load_map_locator_class", return_value=locator),
            patch.object(visual, "close", wraps=visual.close) as close_visual,
            patch.object(type(self.capture), "close", autospec=True) as close_capture,
        ):
            result = teleport.locate_current_position(
                get_frame=get_frame,
                position_backend="auto",
                coordinate_timeout=0,
                runner=runner,
            )
            self.assertEqual(result.point, (20, 30))
            get_frame.assert_called_once_with()
            locator.return_value.locate.assert_called_once_with(sentinel.frame)
            locator.return_value.close.assert_called_once_with()
            close_capture.assert_not_called()
            self.navigation.close()
            close_capture.assert_called_once_with(self.capture)
            close_visual.assert_called_once_with()
        self.create_capture.assert_called_once()
