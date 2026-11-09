"""Compare old full-figure draws with V4 cached rendering; no hardware access."""

import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "apps" / "sensor_6_control_v4"))

import numpy as np
from PySide6.QtWidgets import QApplication
from matplotlib.figure import Figure
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from wrench_plot import WrenchPlotRenderer


def make_plot():
    figure = Figure(figsize=(10, 8), dpi=100)
    force = figure.add_subplot(211)
    torque = figure.add_subplot(212, sharex=force)
    canvas = FigureCanvasQTAgg(figure)
    lines = {
        name: axes.plot([], [], label=name, linewidth=1.3)[0]
        for name, axes in zip(WrenchPlotRenderer.CHANNELS, (force,) * 3 + (torque,) * 3)
    }
    force.set_ylabel("Force (N)")
    torque.set_ylabel("Torque (N m)")
    torque.set_xlabel("Time (s)")
    for axes in (force, torque):
        axes.legend(loc="upper left", ncol=3)
        axes.grid(True, alpha=0.25)
    canvas.show()
    QApplication.processEvents()
    return canvas, force, torque, lines


def summarize(durations):
    return {
        "median_ms": round(float(np.median(durations)) * 1000, 3),
        "p95_ms": round(float(np.percentile(durations, 95)) * 1000, 3),
    }


def benchmark(frames):
    app = QApplication.instance() or QApplication([])
    times = np.arange(5000) / 20.0
    samples = tuple(map(tuple, np.column_stack((times,) + tuple(np.sin(times / (i + 1)) for i in range(6)))))
    canvas, force, torque, lines = make_plot()
    baseline = []
    for _ in range(frames):
        started = time.perf_counter()
        columns = tuple(zip(*samples))
        for index, line in enumerate(lines.values(), 1):
            line.set_data(columns[0], columns[index])
        for axes in (force, torque):
            axes.relim()
            axes.autoscale_view()
        canvas.draw()
        baseline.append(time.perf_counter() - started)
    canvas.close()

    canvas, force, torque, lines = make_plot()
    renderer = WrenchPlotRenderer(canvas, force, torque, lines)
    renderer.update(samples, force=True)
    optimized = []
    for _ in range(frames):
        started = time.perf_counter()
        renderer.update(samples)
        optimized.append(time.perf_counter() - started)
    canvas.close()
    app.processEvents()
    return {
        "frames": frames,
        "samples_per_channel": len(samples),
        "channels": 6,
        "canvas_px": [1000, 800],
        "old_full_draw": summarize(baseline),
        "new_cached_draw": summarize(optimized),
        "median_speedup": round(float(np.median(baseline) / np.median(optimized)), 2),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=30)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    print(json.dumps(benchmark(args.frames), ensure_ascii=False, indent=2))
