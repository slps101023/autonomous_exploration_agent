"""Ideal pinhole depth processing for the simulated D435i (all lengths in metres)."""

import numpy as np


def depth_to_uint16(depth, min_depth=0.1, max_depth=10.0):
    """Convert optical Z in metres to RealSense-style millimetres, with invalid=0."""
    valid = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
    result = np.zeros(depth.shape, dtype=np.uint16)
    result[valid] = np.clip(np.rint(depth[valid] * 1000.0), 1, 65535).astype(np.uint16)
    return result


def align_depth_to_color(depth, depth_k, color_k, color_shape, translation,
                         min_depth=0.1, max_depth=10.0):
    """Reproject pixel footprints between parallel cameras with a Z buffer.

    The translation maps depth optical coordinates into color optical coordinates.
    A source pixel covers a square bounded by its half-pixel corners at its
    measured Z. Rasterize that projected rectangle instead of just its center:
    the narrower color FOV otherwise leaves a regular grid of unassigned pixels.
    Keep the nearest Z where footprints overlap. Unobserved regions stay zero;
    no interpolation joins neighboring pixels across depth discontinuities.
    """
    kd = np.asarray(depth_k).reshape(3, 3)
    kc = np.asarray(color_k).reshape(3, 3)
    if kd[0, 0] <= 0 or kd[1, 1] <= 0 or kc[0, 0] <= 0 or kc[1, 1] <= 0:
        raise ValueError('Camera focal lengths must be positive')
    valid = np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
    v, u = np.nonzero(valid)
    z = depth[v, u].astype(np.float64)
    tx, ty, tz = translation
    x = (u - kd[0, 2]) * z / kd[0, 0] + tx
    y = (v - kd[1, 2]) * z / kd[1, 1] + ty
    z = z + tz
    forward = (z >= min_depth) & (z <= max_depth)
    source_z = depth[v, u][forward].astype(np.float64)
    x, y, z = x[forward], y[forward], z[forward]
    cu = kc[0, 0] * x / z + kc[0, 2]
    cv = kc[1, 1] * y / z + kc[1, 2]
    half_width = 0.5 * kc[0, 0] / kd[0, 0] * source_z / z
    half_height = 0.5 * kc[1, 1] / kd[1, 1] * source_z / z
    height, width = color_shape
    # Include target pixel cells intersected by the projected source footprint.
    # Clip before computing loop bounds, including fully off-screen rectangles.
    left = np.clip(np.floor(cu - half_width + 0.5), 0, width).astype(np.int64)
    right = np.clip(np.ceil(cu + half_width - 0.5), -1, width - 1).astype(np.int64)
    top = np.clip(np.floor(cv - half_height + 0.5), 0, height).astype(np.int64)
    bottom = np.clip(np.ceil(cv + half_height - 0.5), -1, height - 1).astype(np.int64)
    inside = (left <= right) & (top <= bottom)
    left, right = left[inside], right[inside]
    top, bottom, z = top[inside], bottom[inside], z[inside]
    registered = np.full(height * width, np.inf, dtype=np.float64)
    if z.size:
        # Vectorize over all source pixels; typical D435i footprints are 2–3
        # target pixels per axis, avoiding a Python loop per depth sample.
        for dy in range(int(np.max(bottom - top)) + 1):
            rows = top + dy
            row_valid = rows <= bottom
            for dx in range(int(np.max(right - left)) + 1):
                cols = left + dx
                covered = row_valid & (cols <= right)
                np.minimum.at(registered, rows[covered] * width + cols[covered], z[covered])
    return depth_to_uint16(registered.reshape(height, width), min_depth, max_depth)
