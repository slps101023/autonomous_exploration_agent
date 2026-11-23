"""Check units, registration occlusions and the RealSense message contract."""

from types import SimpleNamespace

import numpy as np
from sensor_msgs.msg import Image, Imu

from simulation_bringup.d435i_bridge import D435iBridge, image_array
from simulation_bringup.d435i_geometry import align_depth_to_color, depth_to_uint16


def test_depth_units_and_invalid_values():
    depth = np.array([[1.234, 0.0, np.nan, np.inf, -1.0, 11.0]])
    np.testing.assert_array_equal(depth_to_uint16(depth), [[1234, 0, 0, 0, 0, 0]])


def test_registration_uses_translation_not_resize():
    k = np.eye(3).ravel()
    aligned = align_depth_to_color(np.ones((1, 3)), k, k, (1, 4), [1.0, 0.0, 0.0])
    np.testing.assert_array_equal(aligned, [[0, 1000, 1000, 1000]])


def test_registration_keeps_nearest_depth_when_pixels_collide():
    k = np.eye(3).ravel()
    aligned = align_depth_to_color(np.array([[1.0, 4.0]]), k, k, (1, 3), [1.0, 0.0, 0.0])
    # The farther pixel's footprint also covers pixel 2; overlap stays at 1m.
    np.testing.assert_array_equal(aligned, [[0, 1000, 4000]])


def test_registration_d435i_fovs_cover_a_continuous_plane():
    height, width = 36, 64
    depth_focal = height / (2 * np.tan(np.deg2rad(58 / 2)))
    color_focal = height / (2 * np.tan(np.deg2rad(42 / 2)))
    kd = [depth_focal, 0, (width - 1) / 2,
          0, depth_focal, (height - 1) / 2, 0, 0, 1]
    kc = [color_focal, 0, (width - 1) / 2,
          0, color_focal, (height - 1) / 2, 0, 0, 1]
    aligned = align_depth_to_color(
        np.full((height, width), 2.0), kd, kc, (height, width), [0.015, 0, 0])
    np.testing.assert_array_equal(aligned, np.full((height, width), 2000))


def test_registration_does_not_fill_invalid_source_regions():
    k = np.eye(3).ravel()
    aligned = align_depth_to_color(
        np.array([[1.0, 0.0, 2.0]]), k, k, (1, 3), [0, 0, 0])
    np.testing.assert_array_equal(aligned, [[1000, 0, 2000]])


def test_registration_magnified_footprints_preserve_an_unknown_gap():
    kd = np.eye(3).ravel()
    kc = [2, 0, 0.5, 0, 2, 0.5, 0, 0, 1]
    aligned = align_depth_to_color(
        np.array([[1.0, 0.0, 2.0]]), kd, kc, (2, 6), [0, 0, 0])
    np.testing.assert_array_equal(
        aligned, [[1000, 1000, 0, 0, 2000, 2000],
                  [1000, 1000, 0, 0, 2000, 2000]])


def test_registration_empty_and_out_of_view_stay_invalid():
    k = np.eye(3).ravel()
    for depth, translation in ((np.zeros((2, 3)), [0, 0, 0]),
                               (np.ones((2, 3)), [100, 0, 0]),
                               (np.ones((2, 3)), [0, 0, -2])):
        aligned = align_depth_to_color(depth, k, k, (2, 3), translation)
        np.testing.assert_array_equal(aligned, np.zeros((2, 3)))


def test_depth_decode_honours_endianness_and_padding():
    message = Image(height=2, width=2, encoding='32FC1', is_bigendian=True, step=12)
    message.data = np.array([[1.0, 2.0, 99.0], [3.0, 4.0, 99.0]], dtype='>f4').tobytes()
    np.testing.assert_array_equal(image_array(message), [[1.0, 2.0], [3.0, 4.0]])


def test_public_imu_contains_measurements_without_ground_truth_orientation():
    published = {}
    harness = SimpleNamespace(publish=lambda topic, msg: published.update({topic: msg}))
    message = Imu()
    message.header.stamp.sec = 12
    message.orientation.w = 1.0
    message.angular_velocity.z = 0.2
    message.linear_acceleration.y = -9.80665
    D435iBridge.on_imu(harness, message)
    combined = published['/camera/imu']
    assert combined.header.stamp.sec == 12
    assert combined.orientation.w == 0.0
    assert combined.orientation_covariance[0] == -1.0
    assert combined.angular_velocity.z == 0.2
    assert combined.linear_acceleration.y == -9.80665
    assert published['/camera/gyro/sample'].linear_acceleration_covariance[0] == -1.0
    assert published['/camera/accel/sample'].angular_velocity_covariance[0] == -1.0
