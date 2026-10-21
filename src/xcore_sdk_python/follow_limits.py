"""Shared software bounds for NRT preparation and RT following."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

DEFAULT_SOFT_LO_DEG = (-180.0, -130.0, -165.0, -175.0, -175.0, -175.0)
DEFAULT_SOFT_HI_DEG = (+180.0, +130.0, +135.0, +175.0, +175.0, +175.0)


def _soft_pair(value: Any, defaults: tuple, dofs: int) -> np.ndarray:
    if value is None:
        if dofs != len(defaults):
            raise ValueError("Default following limits require six joints")
        return np.radians(defaults)
    values = np.asarray(value, dtype=float).ravel()
    if values.size == 1:
        return np.full(dofs, float(values[0]))
    if values.size != dofs:
        raise ValueError(f"Soft-limit bounds require one or {dofs} values")
    return values.copy()


def local_soft_limits(
    dofs: int = 6, soft_lo_rad: Any = None, soft_hi_rad: Any = None
) -> tuple[np.ndarray, np.ndarray]:
    lo = _soft_pair(soft_lo_rad, DEFAULT_SOFT_LO_DEG, dofs)
    hi = _soft_pair(soft_hi_rad, DEFAULT_SOFT_HI_DEG, dofs)
    if not np.all(np.isfinite(lo)) or not np.all(np.isfinite(hi)):
        raise ValueError("Local soft limits must be finite")
    if np.any(lo > hi):
        raise ValueError("Local soft-limit bounds are reversed")
    return lo, hi


def intersect_soft_limits(
    controller_bounds: Any,
    *,
    enabled: bool,
    dofs: int = 6,
    soft_lo_rad: Any = None,
    soft_hi_rad: Any = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply exactly the same controller/software intersection at both stages."""
    lo, hi = local_soft_limits(dofs, soft_lo_rad, soft_hi_rad)
    if not enabled:
        raise RuntimeError("Controller soft limits are disabled")
    ctrl = np.asarray(controller_bounds, dtype=float)
    if ctrl.shape != (dofs, 2) or not np.all(np.isfinite(ctrl)):
        raise RuntimeError("Invalid controller soft-limit feedback")
    if np.any(ctrl[:, 0] > ctrl[:, 1]):
        raise RuntimeError("Controller soft-limit bounds are reversed")
    lo, hi = np.maximum(lo, ctrl[:, 0]), np.minimum(hi, ctrl[:, 1])
    if np.any(lo > hi):
        raise ValueError("Local and controller soft limits do not overlap")
    return lo, hi


def limit_violation(q: Any, lo: np.ndarray, hi: np.ndarray) -> str | None:
    values = np.asarray(q, dtype=float)
    if values.shape != lo.shape or not np.all(np.isfinite(values)):
        raise ValueError("Following requires finite joint feedback")
    outside = np.flatnonzero((values < lo - 1e-9) | (values > hi + 1e-9))
    if not outside.size:
        return None
    return ", ".join(
        f"J{i + 1}({math.degrees(values[i]):+.1f}° 限 "
        f"{math.degrees(lo[i]):+g}~{math.degrees(hi[i]):+g}°)"
        for i in outside
    )
