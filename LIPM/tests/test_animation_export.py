import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from PIL import Image

from LIPM.demo_utils.animation_export import save_mp4_or_gif


class AnimationExportTests(unittest.TestCase):
    def test_missing_ffmpeg_writes_readable_gif(self):
        figure = Figure(figsize=(2, 2))
        FigureCanvasAgg(figure)
        axis = figure.add_subplot()
        axis.set(xlim=(0, 2), ylim=(0, 2))
        line, = axis.plot([], [], 'o')

        def update(i):
            line.set_data([i], [i])
            return [line]

        animation = FuncAnimation(figure, update, frames=3)
        with tempfile.TemporaryDirectory() as directory:
            with patch('LIPM.demo_utils.animation_export.writers.is_available', return_value=False):
                output = save_mp4_or_gif(animation, Path(directory)/'test.mp4', fps=10)
            self.assertEqual(output.suffix, '.gif')
            with Image.open(output) as image:
                self.assertEqual(image.format, 'GIF')
                self.assertEqual(image.n_frames, 3)
            self.assertFalse(output.with_suffix('.mp4').exists())

    def test_ffmpeg_uses_explicit_h264_writer(self):
        animation = Mock()
        with patch('LIPM.demo_utils.animation_export.writers.is_available', return_value=True):
            result = save_mp4_or_gif(animation, 'test.mp4', fps=50)
        self.assertEqual(result, Path('test.mp4'))
        args, kwargs = animation.save.call_args
        self.assertEqual(args, ('test.mp4',))
        self.assertEqual(set(kwargs), {'writer'})
        self.assertIsInstance(kwargs['writer'], FFMpegWriter)
        self.assertEqual(kwargs['writer'].codec, 'libx264')
        self.assertEqual(kwargs['writer'].fps, 50)
