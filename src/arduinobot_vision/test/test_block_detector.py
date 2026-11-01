import pytest
import numpy as np
import cv2

from arduinobot_vision.block_detector import BlockDetector


def test_pixel_to_world_at_reference_point():
    detector = BlockDetector()
    x, y = detector.pixel_to_world(BlockDetector.REF_U, BlockDetector.REF_V)
    assert x == BlockDetector.REF_X
    assert y == BlockDetector.REF_Y


def test_pixel_to_world_known_data_point():
    detector = BlockDetector()
    x, y = detector.pixel_to_world(225, 226)
    assert x == pytest.approx(0.1, abs=0.01)
    assert y == pytest.approx(2.0, abs=0.01)


def test_find_centroid_detects_synthetic_square():
    detector = BlockDetector()

    image = np.full((100, 100, 3), (60, 50, 50), dtype=np.uint8)
    cv2.rectangle(image, (20, 30), (40, 50), (5, 200, 200), -1)

    lower, upper = BlockDetector.RED_RANGES[0]
    centroid = detector.find_centroid(image, lower, upper)

    assert centroid == pytest.approx((30, 40), abs=1)
    

def test_find_centroid_returns_none_when_no_match():
    detector = BlockDetector()

    image = np.full((100, 100, 3), (60, 50, 50), dtype=np.uint8)

    lower, upper = BlockDetector.RED_RANGES[0]
    centroid = detector.find_centroid(image, lower, upper)

    assert centroid is None


def test_detect_colors_finds_both():
    detector = BlockDetector()

    image = np.full((100, 100, 3), (60, 50, 50), dtype=np.uint8)
    cv2.rectangle(image, (10, 10), (30, 30), (5, 200, 200), -1)
    cv2.rectangle(image, (60, 60), (80, 80), (115, 200, 200), -1)

    result = detector.detect_colors(image)

    assert result['red'] == pytest.approx((20, 20), abs=1)
    assert result['blue'] == pytest.approx((70, 70), abs=1)


def test_detect_colors_missing_color_is_none():
    detector = BlockDetector()

    image = np.full((100, 100, 3), (60, 50, 50), dtype=np.uint8)
    cv2.rectangle(image, (10, 10), (30, 30), (5, 200, 200), -1)

    result = detector.detect_colors(image)

    assert result['red'] == pytest.approx((20, 20), abs=1)
    assert result['blue'] is None