"""Dependency-light utilities shared by the MuJoCo simulation core.

This module intentionally avoids Gymnasium and Stable-Baselines3 imports so the
physics backend can run in deployments that do not include reinforcement
learning frameworks.
"""

from pathlib import Path

import numpy as np
import yaml


def normalize_rgba(rgba: list) -> list:
    new_rgba = [channel / 255.0 for channel in rgba]
    new_rgba[-1] = rgba[-1]
    return new_rgba


def filter_mask(segment_image: np.ndarray) -> np.ndarray:
    """Convert a MuJoCo segmentation image to an 8-bit mask."""
    geom_ids = segment_image[:, :, 0]
    geom_ids = geom_ids.astype(np.float64) + 1
    geom_ids = geom_ids / geom_ids.max()
    return 255 * geom_ids


def distance(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Calculate the Euclidean distance between equally shaped arrays."""
    assert a.shape == b.shape
    return np.linalg.norm(a - b, axis=-1)


def get_env_config(config: str | None = None) -> dict:
    config_folder = Path(__file__).parent / "config"
    config_path = config_folder / ("env.yaml" if config is None else f"{config}.yaml")
    if not config_path.exists():
        raise FileNotFoundError(f"Could not find config file {config_path}")
    return get_config(config_path)


def get_config(config_path: Path) -> dict:
    with config_path.open("r", encoding="utf-8") as file_obj:
        return yaml.safe_load(file_obj)
