"""Distance to a detected person from an aligned depth image. Pure numpy, no ROS imports."""
import numpy as np

DEPTH_ENCODINGS = {'16UC1': (np.uint16, 0.001), '32FC1': (np.float32, 1.0)}   # (dtype, to metres)


def depth_from_image(encoding, height, width, step, data, is_bigendian=False):
    """(array HxW, scale to metres) from the fields of a sensor_msgs/Image."""
    if encoding not in DEPTH_ENCODINGS:
        raise ValueError(f'unsupported depth encoding {encoding!r}; need 16UC1 or 32FC1')
    dtype, scale = DEPTH_ENCODINGS[encoding]
    dt = np.dtype(dtype).newbyteorder('>' if is_bigendian else '<')
    arr = np.frombuffer(data, dtype=dt).reshape(height, step // dt.itemsize)[:, :width]
    return arr, scale


def person_distance(depth, bbox, image_size, scale=0.001, region_frac=0.4, min_m=0.2, max_m=8.0,
                    min_valid_frac=0.1):
    """Median valid depth (metres) in the central part of the person's box; 0.0 if unknown.

    depth: 2D array aligned to the colour image; bbox: (x_min, y_min, x_max, y_max) in colour
    pixels; image_size: (width, height) of the colour image (depth may have another resolution).
    The central region (region_frac of the box) avoids the background around the silhouette.
    Zeros, NaNs and values outside [min_m, max_m] are ignored; fewer than min_valid_frac valid
    pixels means 0.0 ("unknown"), never a guess.
    """
    dh, dw = depth.shape
    iw, ih = image_size
    if iw <= 0 or ih <= 0:
        return 0.0
    sx, sy = dw / iw, dh / ih
    x0, y0, x1, y1 = bbox
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    hw, hh = (x1 - x0) * region_frac / 2.0, (y1 - y0) * region_frac / 2.0
    c0, c1 = int(max(0, np.floor((cx - hw) * sx))), int(min(dw, np.ceil((cx + hw) * sx)))
    r0, r1 = int(max(0, np.floor((cy - hh) * sy))), int(min(dh, np.ceil((cy + hh) * sy)))
    if c1 <= c0 or r1 <= r0:
        return 0.0
    patch = depth[r0:r1, c0:c1].astype(np.float64) * scale
    valid = patch[np.isfinite(patch) & (patch >= min_m) & (patch <= max_m)]
    if valid.size == 0 or valid.size < min_valid_frac * patch.size:
        return 0.0
    return float(np.median(valid))
