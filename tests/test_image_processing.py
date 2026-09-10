"""Exercise actual PNG/JPEG decoding, pixel selection, and image-to-3D mapping."""

from pathlib import Path
import struct
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from drone_swarm.image_processing import process_image
from drone_swarm.image_processing.pipeline import MAX_FILE_BYTES, MAX_PIXELS


ROOT = Path(__file__).resolve().parents[1]


def encode(image, extension=".png"):
    success, data = cv2.imencode(extension, image)
    if not success:
        raise RuntimeError("Test image encoding failed")
    return data.tobytes()


def colored_rectangle(width=200, height=100):
    image = np.full((height, width, 3), 255, dtype=np.uint8)
    # Red foreground occupies a 2:1 rectangle; BGR source tests RGB output.
    cv2.rectangle(image, (width // 4, height // 4), (3 * width // 4, 3 * height // 4),
                  (10, 20, 230), -1)
    return image


class ImageProcessingTests(unittest.TestCase):
    def test_sample_produces_400_unique_points_and_a_real_preview(self):
        data = (ROOT / "assets/sample.png").read_bytes()
        for mode in ("edges", "filled"):
            with self.subTest(mode=mode):
                result = process_image(data, 400, 30, 25, mode=mode)
                self.assertEqual(result.points.shape, (400, 3))
                self.assertEqual(result.colors.shape, (400, 3))
                self.assertEqual(len(np.unique(result.points, axis=0)), 400)
                self.assertTrue(np.isfinite(result.points).all())
                self.assertTrue(((result.colors >= 0) & (result.colors <= 1)).all())
                self.assertGreaterEqual(result.metadata["candidates"], 400)
                self.assertLessEqual(result.metadata["sampled_candidates"], 10_000)
                preview = cv2.imdecode(np.frombuffer(result.preview_png, np.uint8), cv2.IMREAD_COLOR)
                self.assertEqual(preview.shape[:2], (512, 512))

    def test_aspect_orientation_and_rgb_are_preserved(self):
        result = process_image(encode(colored_rectangle()), 100, 40, 30, mode="filled")
        self.assertEqual(result.metadata["source_dimensions"], [200, 100])
        np.testing.assert_array_equal(result.points[:, 1], np.zeros(100))
        dimensions = np.ptp(result.points, axis=0)
        self.assertAlmostEqual(dimensions[0] / dimensions[2], 2.0, delta=0.1)
        np.testing.assert_allclose(result.colors, np.tile([230, 20, 10], (100, 1)) / 255)
        self.assertLessEqual(np.abs(result.points[:, 0]).max(), 20)
        self.assertLessEqual(np.abs(result.points[:, 2] - 30).max(), 10)

    def test_jpeg_grayscale_and_transparency_decode(self):
        jpeg = process_image(encode(colored_rectangle(), ".jpg"), 20, 20, 30)
        self.assertEqual(jpeg.metadata["format"], "jpeg")
        gray = np.full((80, 80), 255, dtype=np.uint8)
        cv2.circle(gray, (40, 40), 25, 0, 2)
        self.assertEqual(process_image(encode(gray), 20, 20, 30).points.shape, (20, 3))
        rgba = np.zeros((80, 80, 4), dtype=np.uint8)
        cv2.circle(rgba, (40, 40), 25, (20, 20, 230, 255), -1)
        result = process_image(encode(rgba), 20, 20, 30)
        self.assertGreater(result.colors[:, 0].mean(), 0.8)

    def test_downscale_and_small_image_upscale_remain_within_processing_limit(self):
        large = process_image(encode(colored_rectangle(1600, 800)), 20, 20, 30)
        self.assertEqual(large.metadata["resized_dimensions"], [512, 256])
        small = process_image(encode(colored_rectangle(8, 8)), 400, 20, 30, mode="filled")
        self.assertTrue(small.metadata["upscaled"])
        self.assertLessEqual(max(small.metadata["resized_dimensions"]), 512)
        self.assertEqual(len(np.unique(small.points, axis=0)), 400)

    def test_selection_is_deterministic_and_sparse_counts_work(self):
        data = encode(colored_rectangle())
        for count in (1, 2, 20):
            with self.subTest(count=count):
                first = process_image(data, count, 20, 30)
                second = process_image(data, count, 20, 30)
                np.testing.assert_array_equal(first.points, second.points)
                np.testing.assert_array_equal(first.colors, second.colors)
                self.assertEqual(first.preview_png, second.preview_png)

    def test_blank_invalid_and_insufficient_foreground_fail_clearly(self):
        for color in (0, 127, 255):
            with self.subTest(color=color), self.assertRaisesRegex(ValueError, "blank"):
                process_image(encode(np.full((100, 100, 3), color, np.uint8)), 20, 20, 30)
        for data in (b"", b"not an image", b"\xff\xd8\xff\xc0\x00", None, "image.png"):
            with self.subTest(data=data), self.assertRaises(ValueError):
                process_image(data, 20, 20, 30)
        subtle = np.full((512, 512, 3), 230, np.uint8)
        subtle[250:260, 250:260] = 235
        with self.assertRaisesRegex(ValueError, "usable image points"):
            process_image(encode(subtle), 400, 20, 30, mode="filled")

    def test_header_limits_are_checked_before_opencv_decodes(self):
        data = bytearray(encode(colored_rectangle()))
        data[16:24] = struct.pack(">II", 5000, MAX_PIXELS // 5000 + 1)
        with patch("drone_swarm.image_processing.pipeline.cv2.imdecode") as decoder:
            with self.assertRaisesRegex(ValueError, "megapixels"):
                process_image(data, 20, 20, 30)
            decoder.assert_not_called()
        with self.assertRaisesRegex(ValueError, "8 MiB"):
            process_image(b"x" * (MAX_FILE_BYTES + 1), 20, 20, 30)

    def test_inversion_changes_filled_foreground(self):
        data = encode(colored_rectangle())
        normal = process_image(data, 20, 20, 30, mode="filled")
        inverse = process_image(data, 20, 20, 30, mode="filled", invert=True)
        self.assertGreater(np.ptp(inverse.points[:, 0]), np.ptp(normal.points[:, 0]))

    def test_edges_preserve_equal_luminance_color_boundaries(self):
        rgb = np.zeros((120, 120, 3), dtype=np.uint8)
        rgb[10:110, 10:60] = [255, 0, 0]
        rgb[10:110, 60:110] = [0, 130, 0]
        # Both foreground colors have grayscale intensity about 76.
        data = encode(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        result = process_image(data, 300, 20, 20)
        middle = (np.abs(result.points[:, 0]) < 0.2) & (np.abs(result.points[:, 2] - 20) < 7)
        self.assertGreater(np.count_nonzero(middle), 20)

    def test_invalid_options_are_rejected(self):
        data = encode(colored_rectangle())
        for kwargs in ({"count": 0}, {"count": 401}, {"count": True},
                       {"size": 0}, {"size": float("nan")}, {"altitude": float("inf")},
                       {"mode": "unknown"}, {"threshold": 0}, {"threshold": 255},
                       {"invert": "yes"}):
            options = {"count": 20, "size": 20, "altitude": 30, **kwargs}
            with self.subTest(options=options), self.assertRaises(ValueError):
                process_image(data, **options)


if __name__ == "__main__":
    unittest.main()
