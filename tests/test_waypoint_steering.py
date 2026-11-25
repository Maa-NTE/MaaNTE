import sys
import unittest
from importlib import import_module
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock, call, patch


def load_steering_module():
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
        return import_module("_steering_actions.Navi.waypoint_navigator")


steering = load_steering_module()
WaypointNavigator = steering.WaypointNavigator


def pose(point, heading, *, network=True):
    return (
        SimpleNamespace(
            found=True, point=point, camera_heading=heading if network else None
        ),
        SimpleNamespace(found=True, angle=heading),
    )


class AnglePidTest(unittest.TestCase):
    def test_wallet_straight_leg_converges_with_delayed_heading(self):
        # R3P3 初始夹角 28.8°，该段两次鼠标指令的间隔中位数为 0.126 秒。
        for gain, lag in ((0.9, 0), (1.0, 1), (1.3, 2)):
            with self.subTest(gain=gain, lag=lag):
                pid = steering.AnglePidController()
                heading = 0.0
                history = []
                commands = []
                for i in range(60):
                    history.append(heading)
                    measured = history[max(0, i - lag)] + (-1.0, 1.0, 0.0)[i % 3]
                    delta = (28.8 - measured + 180.0) % 360.0 - 180.0
                    turn = pid.update(delta, i * 0.126, heading=measured % 360.0)
                    dx = round(turn * 10)
                    commands.append(dx)
                    heading += gain * dx / 10
                nonzero = [dx for dx in commands if dx]
                reversals = sum(a * b < 0 for a, b in zip(nonzero, nonzero[1:]))
                self.assertLessEqual(reversals, 3)
                self.assertLess(abs(heading - 28.8), 6.0)
                self.assertFalse(any(commands[-20:]))

    def test_heading_noise_at_alignment_does_not_restart_turn(self):
        pid = steering.AnglePidController()
        for i, error in enumerate((2.0, 4.1, -5.8, 5.4, -4.5)):
            self.assertEqual(pid.update(error, i * 0.12, heading=-error), 0.0)
        self.assertGreater(pid.update(7.0, 0.60, heading=353.0), 0.0)
        self.assertGreater(pid.update(5.0, 0.72, heading=355.0), 0.0)
        self.assertEqual(pid.update(3.0, 0.84, heading=357.0), 0.0)

    def test_u_turn_keeps_its_direction_across_the_angle_wrap(self):
        pid = steering.AnglePidController()
        for i, error in enumerate((179.0, -179.0, 178.0, -178.0, 165.0)):
            turn = pid.update(error, i * 0.12, heading=(180.0 - error) % 360.0)
            self.assertGreater(turn, 0.0)
            self.assertLessEqual(turn, pid.output_limit)
        pid.reset()
        self.assertLess(pid.update(-179.0, 2.0, heading=359.0), 0.0)


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
        self.turns_per_frame = []
        self.navigator.sleep_remaining = Mock()
        self.navigator.sleep_interruptible = self.record_forward
        self.navigator.set_jump_enabled(True)
        self.navigator.jump_interval = 0

    def record_forward(self, duration):
        self.forward.append(self.navigator.w_down)
        mouse = self.controller.post_relative_move
        self.turns_per_frame.append([command.args for command in mouse.call_args_list])
        mouse.reset_mock()
        return True

    def assert_horizontal_turn(self, frame, direction):
        commands = self.turns_per_frame[frame]
        self.assertEqual(len(commands), 1, f"frame {frame}: {commands}")
        dx, dy = commands[0]
        self.assertGreater(dx * direction, 0, f"frame {frame}: dx={dx}")
        self.assertEqual(dy, 0)

    def replay_approach(self, target, samples, *, jump_when_stuck=True):
        samples = iter(samples)
        now = 0.0
        jumps = []

        def update(_deadline):
            nonlocal now
            now, result = next(samples)
            return result

        self.navigator.update = update
        self.navigator.jump = lambda: jumps.append(now)
        with patch("time.monotonic", new=lambda: now):
            arrived = self.navigator.move_to(target, jump_when_stuck=jump_when_stuck)
        return arrived, jumps

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
        self.assert_horizontal_turn(0, direction=1)
        self.assert_horizontal_turn(1, direction=-1)
        self.assertEqual(self.turns_per_frame[2], [])

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
                pose((0, 0), 20.0, network=False),
                pose((19, 0), 90.0, network=False),
            ]
        )
        self.assertTrue(self.navigator.move_to((20, 0)))
        self.assertEqual(self.forward, [True])
        self.assert_horizontal_turn(0, direction=1)

    def test_network_turn_resumes_forward_after_alignment(self):
        self.navigator.set_jump_enabled(False)
        self.navigator.update = Mock(
            side_effect=[
                *(pose((0, 0), angle) for angle in (0, 36, 34, 36, 27, 24)),
                pose((0, -49), 0),
            ]
        )
        self.assertTrue(self.navigator.move_to((0, -50)))
        self.assertEqual(self.forward, [True, False, False, False, False, True])

    def test_wallet_penultimate_waypoint_jumps_once_after_stalling(self):
        # R17P10 倒数第二个路点的实测坐标，周期跳跃已关闭。
        self.navigator.set_jump_enabled(False)
        target = (4187, 4291)
        samples = [
            (0.0, pose((4195, 4291), 270.0)),
            (1.0, pose((4195, 4292), 276.0)),
            (2.0, pose((4195, 4293), 282.0)),
            (5.0, pose((4195, 4292), 277.0)),
            (6.0, pose((4191, 4292), 280.0)),
            (7.0, pose((4188, 4291), 270.0)),
        ]
        self.assertEqual(self.replay_approach(target, samples), (True, [2.0]))

    def test_turns_lost_position_and_progress_restart_stall_observation(self):
        target = (0, 0)
        stale_location, stale_angle = pose((8, 0), 270.0)
        stale_location.found = False
        samples = [
            (0.0, pose((12, 0), 270.0)),
            (1.5, pose((12, 0), 90.0)),
            (2.5, pose((12, 0), 270.0)),
            (4.0, pose((8, 0), 270.0)),
            (5.0, (stale_location, stale_angle)),
            (7.0, pose((8, 0), 270.0)),
            (8.0, pose((4, 0), 270.0)),
            (9.5, pose((2, 0), 270.0)),
        ]
        self.assertEqual(self.replay_approach(target, samples), (True, []))

    def test_unresolved_stall_still_times_out_and_recovery_is_opt_in(self):
        self.navigator.max_duration = 5.0
        target = (0, 0)
        samples = [(t, pose((8, 0), 270.0)) for t in range(6)]
        for enabled, expected in ((False, []), (True, [2])):
            with self.subTest(enabled=enabled):
                result = self.replay_approach(target, samples, jump_when_stuck=enabled)
                self.assertEqual(result, (False, expected))
                self.assertEqual(self.navigator.last_stop_reason, "timeout")
                self.assertFalse(self.navigator.w_down)


if __name__ == "__main__":
    unittest.main()
