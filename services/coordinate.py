from __future__ import annotations

from collections.abc import Sequence


Vector3 = tuple[float, float, float]


def _vector3(value: Sequence[float]) -> Vector3:
    if len(value) != 3:
        raise ValueError(f"Expected a 3D vector, got {value!r}")
    return (float(value[0]), float(value[1]), float(value[2]))


def lps_to_mujoco_mm(value: Sequence[float]) -> Vector3:
    x, y, z = _vector3(value)
    return (-x, -y, z)


def lps_to_godot_mm(value: Sequence[float]) -> Vector3:
    x, y, z = _vector3(value)
    return (-x, z, y)


def mm_to_m(value: Sequence[float]) -> Vector3:
    x, y, z = _vector3(value)
    return (x / 1000.0, y / 1000.0, z / 1000.0)
