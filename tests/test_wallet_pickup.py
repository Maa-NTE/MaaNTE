import sys
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
