"""Small drawing components shared by demos; scene semantics stay local."""
import numpy as np


class Ball:
    def __init__(self, ax, size=10, shape='o', color=None, label=None):
        self.scatter, = ax.plot([], [], [], shape, markersize=size, color=color, label=label)

    def update(self, pos):
        self.scatter.set_data_3d([pos[0]], [pos[1]], [pos[2]])


class Line:
    def __init__(self, ax, size=1, color='g'):
        self.line, = ax.plot([], [], [], linewidth=size, color=color)

    def update(self, pos):
        self.line.set_data_3d(pos)


class PhaseShading:
    """Reveal marked time intervals on multiple axes as playback advances."""

    def __init__(self, axes, times, active):
        from matplotlib.patches import Rectangle

        edges = np.diff(np.r_[False, active, False].astype(int))
        self.spans = []
        for start, stop in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)):
            end_time = times[stop] if stop < len(times) else times[-1]
            for axis in axes:
                patch = Rectangle((times[start], 0), 0, 1,
                                  transform=axis.get_xaxis_transform(),
                                  facecolor='darkorange', alpha=0.12,
                                  edgecolor='none', zorder=0)
                axis.add_patch(patch)
                self.spans.append((patch, times[start], end_time))

    def reset(self):
        for patch, _, _ in self.spans:
            patch.set_width(0)

    def update(self, time):
        for patch, start, stop in self.spans:
            patch.set_width(max(0., min(time, stop)-start))
