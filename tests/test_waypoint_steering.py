import sys
import unittest
from importlib import import_module
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, call, patch


def load_navigator():
    # 只加载走位模块。导入整个 custom.action 会注册原生 AgentServer 动作。
    agent_dir = Path(__file__).resolve().parents[1] / "agent"
    actions = agent_dir / "custom" / "action"
    for name, directory in (
        ("_steering_actions", actions),
        ("_steering_actions.Navi", actions / "Navi"),
    ):
        package = ModuleType(name)
        package.__path__ = [str(directory)]
        sys.modules[name] = package
    with patch.object(sys, "path", [str(agent_dir), *sys.path]):
        return import_module(
            "_steering_actions.Navi.waypoint_navigator"
        ).WaypointNavigator


WaypointNavigator = load_navigator()


def pose(point, heading, *, network=True):
    return (
        SimpleNamespace(
            found=True, point=point, camera_heading=heading if network else None
        ),
        SimpleNamespace(found=True, angle=heading),
    )


class WaypointSteeringTest(unittest.TestCase):
    def setUp(self):
        context = Mock()
        context.tasker.stopping = False
        provider = Mock()
        provider.uses_visual_positioning.return_value = False
        self.navigator = WaypointNavigator(
            context, position_provider=provider, tolerance=2
        )
        self.addCleanup(self.navigator.close)
        self.controller = self.navigator.controller
        self.forward = []
        self.navigator.sleep_remaining = Mock()
        self.navigator.sleep_interruptible = self.record_forward
        self.navigator.set_jump_enabled(True)
        self.navigator.jump_interval = 0

    def record_forward(self, duration):
        self.forward.append(self.navigator.w_down)
        return True

    def test_wallet_corner_stops_forward_until_camera_faces_target(self):
        # 钱包 R1P1 的第二个路径点。前两帧取自出现绕圈时的位置和相机朝向。
        self.navigator.update = Mock(
            side_effect=[
                pose((4804, 5976), 60.4),
                pose((4808, 5972), 354.9),
                pose((4808, 5972), 182.0),
                pose((4808, 5975), 182.0),
            ]
        )
        self.assertTrue(self.navigator.move_to((4808, 5976)))
        self.assertEqual(self.forward, [True, False, True])
        self.assertFalse(self.navigator.w_down)
        self.assertEqual(
            self.controller.post_key_up.call_args_list, [call(87), call(87)]
        )
        self.assertNotIn(call(32), self.controller.post_key_down.call_args_list)
        turns = self.controller.post_relative_move.call_args_list
        self.assertTrue(any(turn.args[0] < 0 for turn in turns))

    def test_periodic_jump_remains_available_on_a_long_aligned_leg(self):
        self.navigator.update = Mock(
            side_effect=[
                pose((0, 0), 90.0),
                pose((5, 0), 90.0),
                pose((79, 0), 90.0),
            ]
        )
        self.assertTrue(self.navigator.move_to((80, 0)))
        self.assertEqual(self.forward, [True, True])
        self.assertIn(call(32), self.controller.post_key_down.call_args_list)
        self.assertIn(call(32), self.controller.post_key_up.call_args_list)

    def test_visual_arrow_can_turn_with_forward_movement(self):
        self.navigator.update = Mock(
            side_effect=[
                pose((0, 0), 270.0, network=False),
                pose((19, 0), 90.0, network=False),
            ]
        )
        self.assertTrue(self.navigator.move_to((20, 0)))
        self.assertEqual(self.forward, [True])


if __name__ == "__main__":
    unittest.main()
