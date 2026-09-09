import unittest
from unittest.mock import Mock

from matplotlib.backend_bases import MouseEvent
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from LIPM.demo_utils.playback import RealtimePlayback


class PlaybackTests(unittest.TestCase):
    def setUp(self):
        self.figure = Figure()
        FigureCanvasAgg(self.figure)
        self.axis = self.figure.add_subplot()
        self.axis.set_xlim(0, 1)
        self.now = 10.
        self.frames = []
        self.init = Mock()
        self.player = RealtimePlayback(
            self.figure, [0., .2, .4], self.frames.append, self.init,
            clock=lambda: self.now)

    def click_restart(self):
        x, y = self.player.button.ax.transAxes.transform((.5, .5))
        for kind in ['button_press_event', 'button_release_event']:
            event = MouseEvent(kind, self.figure.canvas, x, y, button=1)
            self.figure.canvas.callbacks.process(kind, event)

    def test_button_restarts_clock_and_restores_view_during_playback(self):
        self.now = 10.25
        self.player._tick()
        self.assertEqual(self.frames[-1], 1)
        self.axis.set_xlim(5, 6)
        self.click_restart()
        self.assertEqual(self.frames[-1], 0)
        self.assertEqual(self.axis.get_xlim(), (0., 1.))
        self.assertEqual(self.init.call_count, 2)
        self.now = 10.30
        self.player._tick()
        self.assertEqual(self.frames[-1], 0)
        self.assertTrue(self.player.running)

    def test_button_restarts_after_end_repeatedly(self):
        for _ in range(3):
            self.now += 1.
            self.player._tick()
            self.assertEqual(self.frames[-1], 2)
            self.assertFalse(self.player.running)
            self.click_restart()
            self.assertEqual(self.frames[-1], 0)
            self.assertTrue(self.player.running)
        self.assertEqual(len(self.player.timer.callbacks), 1)

    def test_close_stops_updates(self):
        self.player.close()
        self.now += 1.
        self.player._tick()
        self.assertFalse(self.player.running)
        self.assertEqual(self.frames, [0])
