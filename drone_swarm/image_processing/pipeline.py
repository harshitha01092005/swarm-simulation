"""Decode PNG/JPEG, extract foreground, and sample a vertical drone formation."""

from dataclasses import dataclass
import math
from numbers import Real
import struct

import cv2
import numpy as np


MAX_FILE_BYTES = 8 * 1024 * 1024
MAX_PIXELS = 16_000_000
MAX_DIMENSION = 8192
PROCESSING_SIZE = 512
MAX_CANDIDATES = 10_000


@dataclass(frozen=True)
class ImageFormation:
    """An exact point set, sampled source RGB colors, and a PNG dot preview.

    ``points`` is float64[N,3] in metres, in the vertical XZ plane with Y=0.
    The source image's longer side maps to ``size``; its center maps to
    (0,0,altitude), and image-down maps to negative Z. ``colors`` is RGB[N,3]
    in [0,1]. The caller validates ground clearance and safe drone separation.
    """

    points: np.ndarray
    colors: np.ndarray
    preview_png: bytes
    metadata: dict


def _image_header(data):
    """Read PNG IHDR / JPEG SOF dimensions before allocating decoded pixels."""
    if data.startswith(b"\x89PNG\r\n\x1a\n"):
        if len(data) < 33 or data[12:16] != b"IHDR" or data[8:12] != b"\0\0\0\r":
            raise ValueError("Invalid PNG header")
        width, height = struct.unpack(">II", data[16:24])
        image_format = "png"
    elif data.startswith(b"\xff\xd8"):
        index = 2
        dimensions = None
        # SOF markers excluding DHT, JPG extension, and DAC markers.
        frame_markers = {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6,
                         0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
        while index < len(data):
            if data[index] != 0xFF:
                raise ValueError("Invalid JPEG marker")
            while index < len(data) and data[index] == 0xFF:
                index += 1
            if index >= len(data):
                break
            marker = data[index]
            index += 1
            if marker in (0xD9, 0xDA):
                break
            if marker == 0x01 or 0xD0 <= marker <= 0xD7:
                continue
            if index + 2 > len(data):
                break
            length = int.from_bytes(data[index:index + 2], "big")
            if length < 2 or index + length > len(data):
                raise ValueError("Truncated JPEG segment")
            if marker in frame_markers:
                if length < 8:
                    raise ValueError("Invalid JPEG frame header")
                height, width = struct.unpack(">HH", data[index + 3:index + 7])
                if dimensions is not None and dimensions != (width, height):
                    raise ValueError("JPEG contains inconsistent frame dimensions")
                dimensions = (width, height)
            index += length
        if dimensions is None:
            raise ValueError("JPEG dimensions could not be read")
        width, height = dimensions
        image_format = "jpeg"
    else:
        raise ValueError("Upload a PNG or JPEG image")
    if not width or not height:
        raise ValueError("Image dimensions must be positive")
    if width * height > MAX_PIXELS or max(width, height) > MAX_DIMENSION:
        raise ValueError("Image exceeds 16 megapixels or an 8192-pixel side")
    return image_format, width, height


def _decode(data):
    if not isinstance(data, (bytes, bytearray, memoryview)) or not data:
        raise ValueError("Image data must contain PNG or JPEG bytes")
    if len(data) > MAX_FILE_BYTES:
        raise ValueError("Image upload exceeds the 8 MiB limit")
    data = bytes(data)
    image_format, width, height = _image_header(data)
    try:
        decoded = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    except cv2.error as error:
        raise ValueError("The image cannot be decoded; upload a valid PNG or JPEG") from error
    if decoded is None or decoded.shape[:2] != (height, width):
        raise ValueError("The image is corrupt or its dimensions do not match its header")
    if decoded.dtype == np.uint16:
        decoded = (decoded >> 8).astype(np.uint8)
    if decoded.ndim == 2:
        rgb = cv2.cvtColor(decoded, cv2.COLOR_GRAY2RGB)
    elif decoded.shape[2] == 4:
        rgba = cv2.cvtColor(decoded, cv2.COLOR_BGRA2RGBA)
        alpha = rgba[:, :, 3:4].astype(np.float32) / 255
        rgb = np.rint(rgba[:, :, :3] * alpha + 255 * (1 - alpha)).astype(np.uint8)
    elif decoded.shape[2] == 3:
        rgb = cv2.cvtColor(decoded, cv2.COLOR_BGR2RGB)
    else:
        raise ValueError("Unsupported image color format")
    return rgb, image_format


def _resize(rgb, longest):
    height, width = rgb.shape[:2]
    factor = longest / max(width, height)
    dimensions = (max(1, round(width * factor)), max(1, round(height * factor)))
    interpolation = cv2.INTER_AREA if factor < 1 else cv2.INTER_NEAREST
    return cv2.resize(rgb, dimensions, interpolation=interpolation)


def _background(rgb):
    border = np.concatenate((rgb[0], rgb[-1], rgb[:, 0], rgb[:, -1]), axis=0)
    return np.median(border, axis=0)


def _extract_mask(rgb, mode, threshold, invert):
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    background = _background(rgb)
    # Automatic polarity handles dark art on white and light art on dark.
    light_background = float(np.dot(background, [0.299, 0.587, 0.114])) > 127
    if mode == "filled":
        threshold_type = cv2.THRESH_BINARY_INV if light_background else cv2.THRESH_BINARY
        if invert:
            threshold_type ^= cv2.THRESH_BINARY_INV
        _, mask = cv2.threshold(gray, threshold, 255, threshold_type)
    else:
        if invert:
            gray = 255 - gray
        edges = cv2.Canny(gray, max(1, threshold // 2), max(2, threshold), L2gradient=True)
        # Equal-luminance colors can carry important image details. Include
        # per-channel contours so grayscale conversion does not erase them.
        for channel in cv2.split(255 - rgb if invert else rgb):
            edges |= cv2.Canny(channel, max(1, threshold // 2), max(2, threshold),
                               L2gradient=True)
        # Keep all external and internal contours, including small colored
        # details, instead of silently retaining only the largest silhouette.
        contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        mask = np.zeros_like(edges)
        cv2.drawContours(mask, contours, -1, 255, thickness=1)
    return mask, background


def _farthest_points(candidates, count):
    """Deterministic farthest-point selection, O(count * <=10000 candidates)."""
    candidates = candidates.astype(np.float64, copy=False)
    center = candidates.mean(axis=0)
    index = int(np.argmin(np.sum((candidates - center) ** 2, axis=1)))
    nearest = np.full(len(candidates), np.inf)
    selected = np.empty(count, dtype=np.int64)
    for iteration in range(count):
        selected[iteration] = index
        difference = candidates - candidates[index]
        np.minimum(nearest, np.einsum("ij,ij->i", difference, difference), out=nearest)
        nearest[selected[:iteration + 1]] = -1
        index = int(np.argmax(nearest))
    return candidates[selected].astype(np.int32)


def _sample_colors(rgb, pixels, background):
    """Prefer nearby bright foreground colors over background-side edge halos.

    Every returned color is an actual RGB pixel in the processed image. This
    permits a colored shape's interior to illuminate its black contour.
    """
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV).astype(np.float32) / 255
    contrast = np.max(np.abs(rgb.astype(np.float32) - background), axis=2) / 255
    score = hsv[:, :, 1] * hsv[:, :, 2] + contrast * 0.1
    score[contrast < 0.04] = -1
    height, width = rgb.shape[:2]
    colors = np.empty((len(pixels), 3), dtype=np.float64)
    for index, (row, column) in enumerate(pixels):
        if hsv[row, column, 1] > 0.15 and hsv[row, column, 2] > 0.15 and contrast[row, column] > 0.04:
            colors[index] = rgb[row, column] / 255
            continue
        top, bottom = max(0, row - 3), min(height, row + 4)
        left, right = max(0, column - 3), min(width, column + 4)
        local = score[top:bottom, left:right]
        if local.max() < 0:
            colors[index] = rgb[row, column] / 255
        else:
            dy, dx = np.unravel_index(np.argmax(local), local.shape)
            colors[index] = rgb[top + dy, left + dx] / 255
    return colors


def _preview(pixels, colors, width, height):
    factor = PROCESSING_SIZE / max(width, height)
    preview = np.full((max(1, round(height * factor)), max(1, round(width * factor)), 3),
                      (24, 15, 8), dtype=np.uint8)
    radius = 2 if len(pixels) < 150 else 1
    for (row, column), color in zip(pixels, colors):
        point = (min(preview.shape[1] - 1, round(column * factor)),
                 min(preview.shape[0] - 1, round(row * factor)))
        bgr = tuple(int(round(value * 255)) for value in color[::-1])
        # A subtle outline keeps dark source colors visible in the preview;
        # the returned LED colors remain the sampled image RGB values.
        if max(bgr) < 50:
            cv2.circle(preview, point, radius + 1, (110, 105, 95), -1, cv2.LINE_AA)
        cv2.circle(preview, point, radius, bgr, -1, cv2.LINE_AA)
    success, encoded = cv2.imencode(".png", preview)
    if not success:
        raise ValueError("Could not encode the formation preview")
    return encoded.tobytes()


def process_image(data: bytes, count: int, size: float, altitude: float,
                  mode="edges", threshold=127, invert=False) -> ImageFormation:
    """Convert PNG/JPEG bytes into exactly 1..400 distinct drone positions.

    Uploads are limited to 8 MiB, 16 megapixels, and 8192 pixels per side;
    dimensions are checked before decoding. Processing uses at most 512 pixels
    on the longer side. Small images are enlarged only if more samples are
    needed. If fewer than ``count`` usable pixels remain, a clear error is
    raised rather than duplicating drone positions.

    Filled mode uses a grayscale threshold with automatic border polarity;
    invert reverses that threshold. Edges mode uses Canny contours, where
    threshold controls sensitivity and invert reverses intensity polarity.
    Color-channel contours retain boundaries between equal-luminance colors.
    Geometrically, intensity inversion usually preserves the same edges.
    """
    if isinstance(count, bool) or not isinstance(count, int) or not 1 <= count <= 400:
        raise ValueError("Drone count must be an integer from 1 to 400")
    for name, value in (("Size", size), ("Altitude", altitude)):
        if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    if size <= 0:
        raise ValueError("Size must be positive")
    if mode not in ("edges", "filled"):
        raise ValueError("Image mode must be edges or filled")
    if isinstance(threshold, bool) or not isinstance(threshold, int) or not 1 <= threshold <= 254:
        raise ValueError("Threshold must be an integer from 1 to 254")
    if not isinstance(invert, bool):
        raise ValueError("Invert must be a boolean")
    original, image_format = _decode(data)
    source_height, source_width = original.shape[:2]
    if np.max(np.ptp(original.reshape(-1, 3), axis=0)) < 3:
        raise ValueError("The image is blank; choose an image with a distinct foreground")
    longest = min(PROCESSING_SIZE, max(source_width, source_height))
    while True:
        rgb = _resize(original, longest)
        mask, background = _extract_mask(rgb, mode, threshold, invert)
        candidates = np.column_stack(np.nonzero(mask))
        if len(candidates) >= count or longest >= PROCESSING_SIZE:
            break
        longest = min(PROCESSING_SIZE, longest * 2)
    if len(candidates) < count:
        raise ValueError(f"Only {len(candidates)} usable image points for {count} drones; "
                         "adjust the threshold, change mode, or choose a clearer image")
    candidate_count = len(candidates)
    if len(candidates) > MAX_CANDIDATES:
        candidates = candidates[np.linspace(0, len(candidates) - 1, MAX_CANDIDATES, dtype=int)]
    selected = _farthest_points(candidates, count)
    height, width = rgb.shape[:2]
    denominator = max(width - 1, height - 1, 1)
    with np.errstate(over="ignore", invalid="ignore"):
        points = np.column_stack(((selected[:, 1] - (width - 1) / 2) * (size / denominator),
                                  np.zeros(count), altitude -
                                  (selected[:, 0] - (height - 1) / 2) * (size / denominator)))
    if not np.isfinite(points).all() or len(np.unique(points, axis=0)) != count:
        raise ValueError("Size and altitude exceed numerical precision for this image")
    colors = _sample_colors(rgb, selected, background)
    return ImageFormation(
        points=points,
        colors=colors,
        preview_png=_preview(selected, colors, width, height),
        metadata={
            "format": image_format,
            "source_dimensions": [source_width, source_height],
            "resized_dimensions": [width, height],
            "candidates": candidate_count,
            "sampled_candidates": len(candidates),
            "count": count,
            "mode": mode,
            "threshold": threshold,
            "inverted": invert,
            "upscaled": longest > max(source_width, source_height),
        },
    )
