"""Tests for the 2D scan registration (no GPU or plotting needed)."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python-receiver-code"))

from registration import (  # noqa: E402
    ScanMap,
    accumulate,
    apply_pose_2d,
    frame_support,
    icp_point_to_line_2d,
    make_pose_2d,
    register_sequence,
    retime_azimuths,
    voxel_downsample,
)


def room_outline(n: int = 200, seed: int = 0) -> np.ndarray:
    """Points on the walls of an asymmetric L-shaped room, so the alignment is unambiguous."""
    corners = np.array([[0, 0], [3, 0], [3, 1.2], [1.4, 1.2], [1.4, 2.5], [0, 2.5], [0, 0]], float)
    rng = np.random.default_rng(seed)
    seg = rng.integers(0, len(corners) - 1, n)
    t = rng.random(n)[:, None]
    return corners[seg] + t * (corners[seg + 1] - corners[seg]) - [1.0, 1.0]


def yaw_of(pose: np.ndarray) -> float:
    return float(np.arctan2(pose[1, 0], pose[0, 0]))


def test_point_to_line_icp_recovers_known_transform():
    target = ScanMap(room_outline(n=1500))
    truth = make_pose_2d(0.12, -0.08, np.radians(10))
    src = apply_pose_2d(np.linalg.inv(truth), room_outline(n=150, seed=3))
    src += np.random.default_rng(1).normal(scale=0.005, size=src.shape)

    pose, score = icp_point_to_line_2d(src, target)

    assert np.allclose(pose[:2, 2], truth[:2, 2], atol=0.02)
    assert abs(yaw_of(pose) - np.radians(10)) < np.radians(1)
    assert score < 0.02


def test_register_sequence_tracks_a_moving_scanner():
    world = room_outline(n=3000)
    truths = [make_pose_2d(0.02 * i, 0.01 * i, np.radians(1.5 * i)) for i in range(20)]
    rng = np.random.default_rng(2)
    scans = [apply_pose_2d(np.linalg.inv(t), world[rng.choice(len(world), 150, replace=False)]) for t in truths]

    poses, scores = register_sequence(scans, refine_passes=1)

    # Poses are relative to the first scan, which is the identity in the truth as well.
    assert np.allclose(poses[-1][:2, 3], truths[-1][:2, 2], atol=0.03)
    assert abs(yaw_of(poses[-1]) - np.radians(1.5 * 19)) < np.radians(1.5)
    assert np.nanmedian(scores[1:]) < 0.02

    frames = [np.column_stack([s, np.zeros(len(s))]) for s in scans]
    points, frame_ids = accumulate(frames, poses)
    assert points.shape == (20 * 150, 3)
    assert frame_ids.max() == 19


def test_retime_removes_a_spurious_encoder_tick():
    # Three packets of eight readings; the encoder jumped an extra 24 deg before the last packet.
    encoder = [10, 130, 274]
    raw = np.array([[500, angle, channel] for angle in encoder for channel in range(8)], dtype=np.int32)

    retimed = retime_azimuths(raw)

    channel7 = retimed[retimed[:, 2] == 7, 1]
    assert np.all(np.diff(channel7) == 120)
    assert np.array_equal(retimed[:, [0, 2]], raw[:, [0, 2]])


def test_frame_support_counts_other_frames_only():
    points = np.array([[0, 0, 0], [0.01, 0, 0], [0.02, 0, 0], [1, 1, 0]], float)
    frame_ids = np.array([0, 0, 1, 2])
    assert frame_support(points, frame_ids, radius=0.05).tolist() == [1, 1, 1, 0]


def test_voxel_downsample_merges_points_in_one_cell():
    points = np.array([[0.01, 0.01], [0.02, 0.02], [0.5, 0.5]])
    down = voxel_downsample(points, voxel=0.1)
    assert len(down) == 2
