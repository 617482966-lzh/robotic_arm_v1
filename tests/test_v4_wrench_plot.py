"""Hardware-free checks for the six-axis plot's optimized render path."""

import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "apps" / "sensor_6_control_v4"))

import numpy as np
from PySide6.QtWidgets import QApplication
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg

from wrench_plot import WrenchPlotRenderer


class WrenchPlotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.figure = Figure(figsize=(8, 6), dpi=100)
        self.force = self.figure.add_subplot(211)
        self.torque = self.figure.add_subplot(212, sharex=self.force)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.lines = {
            name: axes.plot([], [], label=name)[0]
            for name, axes in zip(
                WrenchPlotRenderer.CHANNELS,
                (self.force,) * 3 + (self.torque,) * 3,
            )
        }
        for axes in (self.force, self.torque):
            axes.legend(loc="upper left", ncol=3, framealpha=1.0)
        self.renderer = WrenchPlotRenderer(
            self.canvas, self.force, self.torque, self.lines
        )

    def tearDown(self):
        self.canvas.close()

    def show_canvas(self):
        self.canvas.show()
        self.app.processEvents()

    def samples(self):
        t = np.arange(5000, dtype=float) / 20.0
        return np.column_stack((t,) + tuple(np.sin(t / (i + 1)) for i in range(6)))

    def test_hidden_page_never_consumes_samples(self):
        class NoRead:
            def __iter__(self):
                raise AssertionError("Hidden page must not process history")
        self.assertFalse(self.renderer.update(NoRead(), force=True))

    def test_all_channels_keep_full_history_and_peaks(self):
        self.show_canvas()
        samples = self.samples()
        samples[2357, 2] = 199.9
        samples[3100, 5] = -60.0
        self.renderer.update(samples, force=True)
        for i, name in enumerate(WrenchPlotRenderer.CHANNELS, 1):
            np.testing.assert_array_equal(self.lines[name].get_ydata(), samples[:, i])
            self.assertEqual(len(self.lines[name].get_xdata()), 5000)
        self.assertGreater(self.force.get_ylim()[1], 199.9)
        self.assertLess(self.torque.get_ylim()[0], -60.0)

    def test_stable_frames_reuse_background_and_resize_theme_can_invalidate(self):
        self.show_canvas()
        samples = self.samples()
        frames = []
        self.canvas.mpl_connect("draw_event", lambda event: frames.append(event))
        self.renderer.update(samples, force=True)
        full_draws = len(frames)
        for _ in range(10):
            self.renderer.update(samples)
        self.assertEqual(len(frames), full_draws)
        self.renderer.invalidate()
        self.renderer.update(samples)
        self.assertEqual(len(frames), full_draws + 1)

    def test_reset_clears_lines_and_limits(self):
        self.show_canvas()
        self.renderer.update(self.samples(), force=True)
        self.renderer.update((), force=True)
        self.assertTrue(all(len(line.get_xdata()) == 0 for line in self.lines.values()))
        self.assertEqual(self.force.get_xlim(), (0.0, 10.0))
        self.assertLess(self.force.get_ylim()[0], 0.0)
        self.assertGreater(self.force.get_ylim()[1], 0.0)

    def test_new_peak_and_time_boundary_invalidate_background(self):
        self.show_canvas()
        self.renderer.update(((0.0,) * 7,), force=True)
        self.renderer.update(((0.0,) * 7, (10.01, 201, 0, 0, 0, 0, 0)))
        self.assertGreaterEqual(self.force.get_xlim()[1], 10.01)
        self.assertGreater(self.force.get_ylim()[1], 201)

    def test_cached_legends_do_not_corrupt_tick_label_pixels(self):
        self.show_canvas()
        samples = self.samples()
        self.renderer.update(samples, force=True)

        # Capture an ordinary, nonanimated Matplotlib draw as the reference.
        self.canvas.mpl_disconnect(self.renderer._draw_connection)
        for line in self.lines.values():
            line.set_animated(False)
        self.canvas.draw()
        reference = np.asarray(self.canvas.buffer_rgba()).copy()

        for line in self.lines.values():
            line.set_animated(True)
        self.renderer._draw_connection = self.canvas.mpl_connect(
            "draw_event", self.renderer._on_draw
        )
        self.renderer.invalidate()
        self.renderer.update(samples, force=True)
        for _ in range(3):
            self.renderer.update(samples)
        cached = np.asarray(self.canvas.buffer_rgba()).copy()

        # Display-space bboxes have a bottom origin, while RGBA pixels have a
        # top origin. Compare ticks, labels, and the trace interiors. Ignore
        # only axes borders (animated traces have a different spine order)
        # and the two-pixel cache padding around actual legend rectangles.
        mask = np.ones(reference.shape[:2], dtype=bool)
        height = mask.shape[0]
        for axes in (self.force, self.torque):
            x0, y0, x1, y1 = axes.bbox.extents
            for x in (int(x0), int(x1)):
                mask[
                    max(0, int(height - y1) - 2):int(height - y0) + 3,
                    max(0, x - 2):x + 3,
                ] = False
            for y in (int(height - y0), int(height - y1)):
                mask[
                    max(0, y - 2):y + 3,
                    max(0, int(x0) - 2):int(x1) + 3,
                ] = False
            x0, y0, x1, y1 = axes.get_legend().get_window_extent().padded(3).extents
            mask[
                max(0, int(height - y1) - 1):int(height - y0) + 2,
                max(0, int(x0) - 1):int(x1) + 2,
            ] = False
        np.testing.assert_array_equal(cached[mask], reference[mask])

        # Legend pixels also stay at their actual display position rather
        # than being restored over the lower axes or its tick labels.
        for axes in (self.force, self.torque):
            x0, y0, x1, y1 = axes.get_legend().get_window_extent().extents
            region = (
                slice(int(height - y1) + 1, int(height - y0) - 1),
                slice(int(x0) + 1, int(x1) - 1),
            )
            np.testing.assert_array_equal(cached[region], reference[region])


if __name__ == "__main__":
    unittest.main()
