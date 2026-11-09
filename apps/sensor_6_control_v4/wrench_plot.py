"""Bounded-cost rendering for the V4 six-axis force/torque plot.

The application retains and exports acquisition samples independently. This
renderer only updates the six visible lines and reuses the axes background
between changes to limits, window size, or theme.
"""

from __future__ import annotations

import math
import time

import numpy as np


class WrenchPlotRenderer:
    CHANNELS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
    X_STEP_SECONDS = 10.0
    Y_SHRINK_INTERVAL_SECONDS = 2.0

    def __init__(self, canvas, force_axes, torque_axes, lines):
        self.canvas = canvas
        self.axes = (force_axes, torque_axes)
        self.lines = tuple(lines[name] for name in self.CHANNELS)
        self._background = None
        self._legend_backgrounds = ()
        self._last_rescale_at = 0.0
        self._uses_blit = bool(canvas.supports_blit)
        for line in self.lines:
            line.set_animated(self._uses_blit)
        for axes in self.axes:
            axes.set_autoscale_on(False)
            axes.set_ylim(-1.0, 1.0)
        self.axes[0].set_xlim(0.0, self.X_STEP_SECONDS)
        self._draw_connection = canvas.mpl_connect("draw_event", self._on_draw)
        self._resize_connection = canvas.mpl_connect(
            "resize_event", lambda _event: self.invalidate()
        )
        for axes in self.axes:
            axes.callbacks.connect("xlim_changed", lambda _axes: self.invalidate())
            axes.callbacks.connect("ylim_changed", lambda _axes: self.invalidate())

    def invalidate(self):
        """Discard cached pixels after theme, resize, or axes changes."""
        self._background = None

    def _draw_lines(self):
        for axes, lines in zip(self.axes, (self.lines[:3], self.lines[3:])):
            for line in lines:
                axes.draw_artist(line)
        # The static legend pixels are part of the background. Restore their
        # small rectangles after the traces instead of rendering six text
        # labels every frame; this also keeps traces behind the legends.
        if self._background is not None:
            for legend_background in self._legend_backgrounds:
                self.canvas.restore_region(legend_background)

    def _on_draw(self, event):
        if event.canvas is not self.canvas or not self._uses_blit:
            return
        self._background = self.canvas.copy_from_bbox(self.canvas.figure.bbox)
        self._legend_backgrounds = tuple(
            self.canvas.copy_from_bbox(
                legend.get_window_extent(event.renderer).padded(2)
            )
            for axes in self.axes
            if (legend := axes.get_legend()) is not None
        )
        self._draw_lines()

    @staticmethod
    def _padded_y_limits(low, high):
        low, high = min(0.0, low), max(0.0, high)
        span = max(high - low, 0.1)
        step = 10.0 ** math.floor(math.log10(span)) / 2.0
        pad = max(span * 0.1, step)
        return (
            math.floor((low - pad) / step) * step,
            math.ceil((high + pad) / step) * step,
        )

    def _update_limits(self, data, force):
        changed = False
        if len(data):
            earliest = float(data[:, 0].min())
            latest = float(data[:, 0].max())
            step = self.X_STEP_SECONDS
            left = math.floor(earliest / step) * step
            desired_x = (
                left,
                max(math.ceil(latest / step) * step, left + step),
            )
        else:
            desired_x = (0.0, self.X_STEP_SECONDS)
        current_x = self.axes[0].get_xlim()
        if (
            force
            or not len(data)
            or earliest < current_x[0]
            or latest > current_x[1]
            or desired_x[0] >= current_x[0] + self.X_STEP_SECONDS
        ) and current_x != desired_x:
            self.axes[0].set_xlim(*desired_x)
            changed = True

        now = time.monotonic()
        can_shrink = now - self._last_rescale_at >= self.Y_SHRINK_INTERVAL_SECONDS
        for axes, section in zip(self.axes, (slice(1, 4), slice(4, 7))):
            values = data[:, section] if len(data) else np.array((0.0,))
            finite_values = values[np.isfinite(values)]
            low = float(finite_values.min()) if finite_values.size else 0.0
            high = float(finite_values.max()) if finite_values.size else 0.0
            desired_y = self._padded_y_limits(low, high)
            current_y = axes.get_ylim()
            shrink = can_shrink and (
                desired_y[1] - desired_y[0]
                < (current_y[1] - current_y[0]) * 0.5
            )
            if (
                force or not len(data) or low < current_y[0]
                or high > current_y[1] or shrink
            ) and current_y != desired_y:
                axes.set_ylim(*desired_y)
                changed = True
        if changed:
            self._last_rescale_at = now
        return changed

    def update(self, samples, force=False):
        """Render one visible frame; return False when the page is hidden.

        ``samples`` contains ``(time_s, Fx, Fy, Fz, Tx, Ty, Tz)`` rows. No
        history is decimated; all retained points, including peaks, are drawn.
        """
        if not self.canvas.isVisible():
            return False
        data = np.asarray(tuple(samples), dtype=float)
        if not data.size:
            data = np.empty((0, 7), dtype=float)
        if data.ndim != 2 or data.shape[1] != 7:
            raise ValueError("Six-axis plot samples must have seven columns")
        data = data[np.isfinite(data[:, 0])]
        for column, line in enumerate(self.lines, 1):
            line.set_data(data[:, 0], data[:, column])
        limits_changed = self._update_limits(data, force)
        if not self._uses_blit or limits_changed or self._background is None:
            self.canvas.draw()
        else:
            self.canvas.restore_region(self._background)
            self._draw_lines()
            self.canvas.blit(self.canvas.figure.bbox)
        return True
