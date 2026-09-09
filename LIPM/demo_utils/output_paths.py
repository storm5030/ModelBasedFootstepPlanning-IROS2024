"""Default output locations shared by demo and analysis entry points."""
from pathlib import Path


def default_output_dir(script):
    """Use the repository outputs folder, independently of the working directory."""
    name = Path(script).stem.replace('demo_LIPM_3D_', 'lipm_', 1)
    repository = Path(__file__).resolve().parents[2]
    return repository / 'outputs' / name
