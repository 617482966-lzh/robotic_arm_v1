"""Force-protection regression checks using simulated sensor/robot data only."""

import importlib.util
import os
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
APP_DIR = Path(__file__).resolve().parents[1] / "apps" / "sensor_6_control_v4"
sys.path.insert(0, str(APP_DIR))
spec = importlib.util.spec_from_file_location("v4_main_force_test", APP_DIR / "main.py")
main = importlib.util.module_from_spec(spec)
spec.loader.exec_module(main)


class ForceProtectionTests(unittest.TestCase):
    def guard(self):
        stop = Mock()
        guard = main.ForceProtection(stop)
        guard.set_connected(True)
        return guard, stop

    def test_each_axis_both_signs_and_strict_threshold(self):
        for axis in range(3):
            for sign in (-1, 1):
                with self.subTest(axis=axis, sign=sign):
                    guard, stop = self.guard()
                    sample = [0.0] * 6
                    sample[axis] = sign * 200
                    guard.observe(sample)
                    stop.assert_not_called()
                    sample[axis] = sign * 200.01
                    guard.observe(sample)
                    stop.assert_called_once_with(("Fx", "Fy", "Fz")[axis], sample[axis])
                    self.assertTrue(guard.motion_blocked())

    def test_disabled_disconnected_and_torque_are_not_force_triggers(self):
        guard, stop = self.guard()
        guard.observe((0, 0, 0, 999, -999, 999))
        guard.set_enabled(False)
        guard.observe((300, 0, 0, 0, 0, 0))
        guard.set_connected(False)
        guard.set_enabled(True)
        guard.observe((300, 0, 0, 0, 0, 0))
        stop.assert_not_called()

    def test_toggle_on_checks_latest_force_and_latches_until_safe_enable(self):
        guard, stop = self.guard()
        guard.set_enabled(False)
        guard.observe((0, -250, 0, 0, 0, 0))
        guard.set_enabled(True)
        stop.assert_called_once_with("Fy", -250.0)
        guard.observe((0, -260, 0, 0, 0, 0))
        self.assertEqual(stop.call_count, 1)
        self.assertFalse(guard.rearm_for_enable())
        guard.observe((0, 0, 0, 0, 0, 0))
        self.assertTrue(guard.motion_blocked())
        self.assertTrue(guard.rearm_for_enable())
        self.assertFalse(guard.motion_blocked())
        guard.observe((201, 0, 0, 0, 0, 0))
        self.assertEqual(stop.call_count, 2)

    def test_acquisition_submits_stop_before_gui_signal_without_event_loop(self):
        events = []
        fake_sensor = SimpleNamespace(monitor_6_sensor=lambda: (0, 0, -201, 0, 0, 0))
        guard = main.ForceProtection(lambda axis, value: events.append(("stop", threading.get_ident())))
        guard.set_connected(True)
        worker = main.SensorWorker(fake_sensor, force_protection=guard)

        def on_data(*_values):
            events.append(("data", threading.get_ident()))
            worker.request_stop()

        worker.data_ready.connect(on_data, main.Qt.ConnectionType.DirectConnection)
        acquisition = threading.Thread(target=worker.run)
        acquisition.start()
        acquisition.join(2)
        self.assertFalse(acquisition.is_alive())
        self.assertEqual([item[0] for item in events], ["stop", "data"])
        self.assertNotEqual(events[0][1], threading.get_ident())

    def test_pending_emergency_cannot_be_replaced_by_trial_pause(self):
        guard, _ = self.guard()
        robot = main.RobotWorker("unused", 0, force_protection=guard)
        robot.submit("joint_increment", (0, 0, 0, 0, 0, 10), 12)
        robot.submit("stop", urgent=True)
        self.assertFalse(robot.submit("safe_stop", urgent=True))
        self.assertEqual(robot._commands.qsize(), 1)
        self.assertEqual(robot._commands.get_nowait()[2], "stop")

    def test_shutdown_preserves_pending_emergency_and_sends_before_disconnect(self):
        events = []
        worker = main.RobotWorker("unused", 0)
        fake = Mock()
        fake.emergency_stop.side_effect = lambda: (events.append("stop") or True)
        fake.disconnect.side_effect = lambda: events.append("disconnect")
        worker.submit("joint_increment", (0, 0, 0, 0, 0, 10), 12)
        worker.submit("stop", urgent=True)
        worker.stop()
        self.assertTrue(worker._emergency_pending.is_set())
        with worker._commands.mutex:
            self.assertIn("stop", [item[2] for item in worker._commands.queue])
        with patch.object(main, "RobotComm", return_value=fake):
            worker.run()
        self.assertEqual(events, ["stop", "disconnect"])
        fake.move_joints_increment.assert_not_called()
        self.assertFalse(worker._emergency_pending.is_set())

    def test_shutdown_does_not_repeat_a_completed_emergency(self):
        worker = main.RobotWorker("unused", 0)
        fake = Mock()
        worker.robot = fake
        worker.submit("stop", urgent=True)
        command = worker._commands.get_nowait()[2]
        worker._execute(command, ())
        worker.stop()
        with patch.object(main, "RobotComm", return_value=fake):
            worker.run()
        fake.emergency_stop.assert_called_once_with()

    def test_new_emergency_during_inflight_stop_keeps_priority(self):
        events = []
        worker = main.RobotWorker("unused", 0)
        fake = Mock()

        def emergency():
            events.append("stop")
            if len(events) == 1:
                worker.submit("stop", urgent=True)
            return True

        def poll():
            events.append("poll")
            worker._stop_event.set()
            return (0,) * 6, (0,) * 6

        fake.emergency_stop.side_effect = emergency
        fake.read_realtime_state.side_effect = poll
        worker.submit("stop", urgent=True)
        with patch.object(main, "RobotComm", return_value=fake):
            worker.run()
        self.assertEqual(events, ["stop", "stop", "poll"])

    def test_safe_reenable_cannot_overtake_pending_emergency(self):
        events = []
        guard = main.ForceProtection(lambda *_args: worker.submit("stop", urgent=True))
        guard.set_connected(True)
        worker = main.RobotWorker("unused", 0, force_protection=guard)
        fake = Mock()
        fake.emergency_stop.side_effect = lambda: (events.append("stop") or True)
        fake.enable.side_effect = lambda: (events.append("enable") or True)

        def move(*_args):
            events.append("motion")
            worker._stop_event.set()
            return True

        fake.move_joints_increment.side_effect = move
        fake.read_realtime_state.return_value = (0,) * 6, (0,) * 6
        guard.observe((201, 0, 0, 0, 0, 0))
        guard.observe((0, 0, 0, 0, 0, 0))
        self.assertTrue(guard.rearm_for_enable())
        worker.submit("enable")
        worker.submit("joint_increment", (0, 0, 0, 0, 0, 5), 5)
        with patch.object(main, "RobotComm", return_value=fake):
            worker.run()
        self.assertEqual(events, ["stop", "enable", "motion"])

    def test_latched_guard_rejects_motion_both_at_submit_and_execution(self):
        guard, _ = self.guard()
        robot = main.RobotWorker("unused", 0, force_protection=guard)
        guard.observe((201, 0, 0, 0, 0, 0))
        for command in main.MOTION_COMMANDS:
            self.assertFalse(robot.submit(command))
            with self.assertRaises(RuntimeError):
                robot._execute(command, ())

    def test_emergency_uses_existing_emergency_stop_not_safe_stop(self):
        guard, _ = self.guard()
        robot = main.RobotWorker("unused", 0, force_protection=guard)
        fake = Mock()
        robot.robot = fake
        robot._execute("stop", ())
        fake.emergency_stop.assert_called_once_with()
        fake.safe_stop_and_clear.assert_not_called()

    def test_emergency_is_sent_before_next_realtime_poll(self):
        events = []
        worker = main.RobotWorker("unused", 0)
        fake = Mock()

        def move(*_args):
            events.append("motion")
            worker.submit("stop", urgent=True)
            return True

        def stop():
            events.append("stop")
            return True

        def poll():
            events.append("poll")
            worker._stop_event.set()
            return (0,) * 6, (0,) * 6

        fake.move_joints_increment.side_effect = move
        fake.emergency_stop.side_effect = stop
        fake.read_realtime_state.side_effect = poll
        worker.submit("joint_increment", (0, 0, 0, 0, 0, 5), 5)
        with patch.object(main, "RobotComm", return_value=fake):
            worker.run()
        self.assertEqual(events, ["motion", "stop", "poll"])

    def test_manual_xyz_range_guard_preserves_other_motion_paths(self):
        controller = SimpleNamespace(
            window=SimpleNamespace(statusBar=lambda: SimpleNamespace(showMessage=Mock())),
            _submit_robot=Mock(),
        )
        for xyz in ((101, 0, 0), (0, -101, 0), (0, 0, 100.001), (float("nan"), 0, 0)):
            request = SimpleNamespace(pose=(*xyz, 0, 0, 0), speed_mm_s=5)
            main.AppController._on_world_increment(controller, request)
        controller._submit_robot.assert_not_called()
        request = SimpleNamespace(pose=(100, -100, 100, 0, 0, 0), speed_mm_s=5)
        main.AppController._on_world_increment(controller, request)
        controller._submit_robot.assert_called_once_with("world_increment", request.pose, 5)

    def test_initial_handshake_is_checked_before_sensor_worker_starts(self):
        events = []
        guard = main.ForceProtection(lambda *_args: events.append("stop"))
        fake_sensor = Mock()
        fake_sensor.monitor_6_sensor.return_value = (201, 0, 0, 0, 0, 0)
        fake_worker = Mock()
        fake_worker.start.side_effect = lambda: events.append("start_sampling")
        controller = SimpleNamespace(
            worker=None, sensor_comm=None, sensor_ctrl=None,
            _force_protection=guard,
            _on_data_ready=Mock(), _on_worker_error=Mock(),
            window=Mock(),
        )
        with patch.object(main, "SensorCommunication", return_value=Mock()), \
                patch.object(main, "SensorController", return_value=fake_sensor), \
                patch.object(main, "SensorWorker", return_value=fake_worker):
            main.AppController._on_sensor_connect(controller, "COM-test")
        self.assertEqual(events, ["stop", "start_sampling"])
        self.assertTrue(guard.motion_blocked())

    def test_robot_reconnect_while_latched_sends_stop_and_blocks_controls(self):
        guard, _ = self.guard()
        guard.observe((0, 0, -250, 0, 0, 0))
        controller = SimpleNamespace(
            _force_protection=guard, window=Mock(), robot_worker=Mock(),
        )
        main.AppController._on_robot_connected(controller, "unused", 0)
        controller.robot_worker.submit.assert_called_once_with("stop", urgent=True)
        controller.window.set_motion_controls_enabled.assert_called_once_with(False)


if __name__ == "__main__":
    unittest.main()
