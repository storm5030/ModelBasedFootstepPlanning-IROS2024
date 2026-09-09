"""Animation export with an explicit writer and matching file extension."""
from pathlib import Path

from matplotlib.animation import FFMpegWriter, PillowWriter, writers


def save_mp4_or_gif(animation, filepath, fps):
    """Save MP4 when FFmpeg is available, otherwise save GIF beside it."""
    filepath = Path(filepath)
    if writers.is_available('ffmpeg'):
        filepath = filepath.with_suffix('.mp4')
        writer = FFMpegWriter(fps=fps, codec='libx264')
    else:
        filepath = filepath.with_suffix('.gif')
        writer = PillowWriter(fps=fps)
        print('FFmpeg unavailable; saving GIF instead:', filepath)
    animation.save(str(filepath), writer=writer)
    print('Saved animation:', filepath.resolve())
    return filepath
