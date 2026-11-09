"""V4 GUI checks without serial, robot sockets or user configuration writes."""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
APP_DIR = Path(__file__).resolve().parents[1] / "apps" / "sensor_6_control_v4"
sys.path.insert(0, str(APP_DIR))

from PySide6.QtWidgets import QApplication
import main_window_4 as ui


class WindowControlsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.settings = patch.object(ui, "SETTINGS_PATH", str(Path(self.temp.name) / "test.ini"))
        self.migration = patch.object(ui.MainWindow, "_migrate_legacy_registry_settings", lambda _self: None)
        self.settings.start()
        self.migration.start()
        self.window = ui.MainWindow()
        self.window.resize(1400, 800)
        self.window.show()
        self.app.processEvents()

    def tearDown(self):
        self.window.close()
        self.window.deleteLater()
        self.app.processEvents()
        self.migration.stop()
        self.settings.stop()
        self.temp.cleanup()

    def test_increment_limits_boundaries_and_defence(self):
        for axis in "XYZ":
            spin = self.window._find_child(f"robotTarget{axis}_zengliang")
            self.assertEqual((spin.minimum(), spin.maximum()), (-100.0, 100.0))
            spin.setValue(1000)
            self.assertEqual(spin.value(), 100.0)
            spin.setValue(-1000)
            self.assertEqual(spin.value(), -100.0)
            self.assertGreater(self.window._find_child(f"robotTarget{axis}_global").maximum(), 100)
        self.window._find_child("robotTargetX_zengliang").setValue(100)
        self.assertEqual(self.window.get_world_increment_request().pose[:3], (100.0, -100.0, -100.0))
        requests = []
        self.window.world_increment_requested.connect(requests.append)
        self.window._emit_world_increment()
        self.assertEqual(len(requests), 1)
        with patch.object(self.window, "_read_spin_values", return_value=(100.001, 0, 0, 0, 0, 0)):
            self.window._emit_world_increment()
        self.assertEqual(len(requests), 1, "out-of-range motion must not be emitted")

    def test_protection_switch_defaults_on_and_emits_state(self):
        button = self.window._find_child("sensorForceProtectionBtn")
        self.assertTrue(button.isCheckable())
        self.assertTrue(self.window.is_force_protection_enabled())
        self.assertTrue(button.isVisibleTo(self.window))
        states = []
        self.window.force_protection_toggled.connect(states.append)
        button.click()
        self.assertFalse(self.window.is_force_protection_enabled())
        self.assertIn("关闭", button.text())
        button.click()
        self.assertTrue(self.window.is_force_protection_enabled())
        self.assertEqual(states, [False, True])

    def test_hidden_history_page_switch_close_and_reset(self):
        # Hidden plotting must not sort/scan/render the history on every sample.
        with patch.object(self.window._wrench_renderer, "update", wraps=self.window._wrench_renderer.update) as render:
            for index in range(5005):
                self.window.add_wrench_sample(1, 2, 3, 4, 5, 6, timestamp=index / 20)
            self.window._refresh_wrench_plot()
            self.assertEqual(render.call_count, 0)
        self.assertEqual(len(self.window._wrench_data), 5000)
        self.assertEqual(self.window._wrench_data[0][0], 0.25)
        self.window._find_child("plotWrenchPageBtn").click()
        self.app.processEvents()
        self.assertTrue(self.window._wrench_plot_timer.isActive())
        for index, name in enumerate(("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"), 1):
            line = self.window.wrench_lines[name]
            self.assertEqual(len(line.get_xdata()), 5000)
            self.assertEqual(line.get_ydata()[-1], index)
        self.window.set_wrench_plot_visible(False)
        self.assertFalse(self.window._wrench_plot_timer.isActive())
        self.window.add_wrench_sample(21, 22, 23, 24, 25, 26, timestamp=251)
        self.window.set_wrench_plot_visible(True)
        self.assertEqual(self.window.wrench_lines["Fx"].get_ydata()[-1], 21)
        self.window.toggle_theme()
        self.window.resize(1920, 1080)
        self.app.processEvents()
        self.window.reset_wrench_plot()
        self.assertEqual(len(self.window._wrench_data), 0)
        self.assertEqual(len(self.window.wrench_lines["Fx"].get_xdata()), 0)
        self.window._find_child("plotExperimentPageBtn").click()
        self.app.processEvents()
        self.assertFalse(self.window._wrench_plot_timer.isActive())


if __name__ == "__main__":
    unittest.main()
