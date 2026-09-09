"""Wall-clock playback for precomputed demo trajectories."""
from time import perf_counter

import numpy as np


def realtime_frames(times, clock=perf_counter):
    """Select recorded frames by elapsed wall time, skipping late display frames."""
    started = clock()
    yield 0
    last = len(times) - 1
    while last > 0:
        elapsed = max(0., clock() - started)
        index = min(last, int(np.searchsorted(times, times[0] + elapsed, side='right')) - 1)
        yield index
        if index == last:
            return


class RealtimePlayback:
    """Replay recorded data with a restart button and an independent GUI timer."""

    def __init__(self, figure, times, update, init, interval=20, clock=perf_counter):
        from matplotlib.widgets import Button

        self.figure = figure
        self.times = times
        self.update = update
        self.init = init
        self.clock = clock
        self.running = False
        self._limits = [(axis, axis.get_xlim(), axis.get_ylim(),
                         axis.get_zlim() if hasattr(axis, 'get_zlim') else None)
                        for axis in figure.axes]
        self.timer = figure.canvas.new_timer(interval=interval)
        self.timer.add_callback(self._tick)
        self.button = Button(figure.add_axes([0.42, 0.015, 0.16, 0.035]), 'Restart')
        self.button.on_clicked(self.restart)
        self._close_id = figure.canvas.mpl_connect('close_event', self.close)
        self.restart()

    def restart(self, event=None):
        """Reset the displayed state and clock, including after playback ends."""
        self.timer.stop()
        for axis, xlim, ylim, zlim in self._limits:
            axis.set_xlim(xlim)
            axis.set_ylim(ylim)
            if zlim is not None:
                axis.set_zlim(zlim)
        self.init()
        self.update(0)
        self.figure.canvas.draw()
        self._frames = realtime_frames(self.times, clock=self.clock)
        next(self._frames)  # Start the clock after rendering the initial state.
        self.running = True
        self.timer.start()

    def _tick(self):
        if not self.running:
            return
        try:
            index = next(self._frames)
        except StopIteration:
            self.running = False
            self.timer.stop()
            return
        self.update(index)
        self.figure.canvas.draw_idle()
        if index == len(self.times) - 1:
            self.running = False
            self.timer.stop()

    def close(self, event=None):
        self.running = False
        self.timer.stop()
