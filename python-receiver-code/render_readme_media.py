"""Register a recording into a map and render images and GIFs from it.

Needs the optional media dependencies (`uv sync --group media`). Renders off-screen, so no
display or window is needed. Without --path it renders the bundled sample into `Images/` (the
README media); with --path it writes into a `media/` folder next to the recording.
Three variants:

- map:    all revolutions registered into one map of the surroundings, built up while orbiting
- replay: real-time playback inside that map, with the eight sensor beams sweeping
- range:  the 8-channel x 360 deg range image as an unrolled panorama strip
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from offline_visualization import (
    CHANNEL_ANGLE_LUT,
    MAX_RANGE_MM,
    collect_frame_paths,
    convert_to_cartesian,
    repo_root,
    resolve_input_path,
)
from registration import accumulate, frame_support, register_sequence, retime_azimuths

BACKGROUND = "#0b0f14"
TEXT_COLOR = "#c9d1d9"
SCANNER_COLOR = "#ffd23f"
BEAM_COLOR = "#ff5e5b"
GRID_COLOR = "#26313d"
PATH_COLOR = "#f2f5f8"
# The bottom channel looks down at this angle; its hits on the floor give the floor height.
FLOOR_CHANNEL = 7
# Beams of readings with no return are drawn as short stubs of this length (m).
NO_RETURN_BEAM = 0.35
# Readings this close come from the scanner's own base and mount, not from the room.
MIN_RANGE_MM = 120
# Registration uses only wall hits: the lowest channels see the floor, which moves with the
# scanner, and readings near the 1.3 m range limit are too noisy.
REGISTRATION_MAX_CHANNEL = 5
REGISTRATION_MAX_RANGE_MM = 1200
# A map point is kept when at least this many other revolutions saw a point within 4 cm.
MIN_SUPPORT = 3
HEIGHT_CLIM = (-0.30, 0.33)
RANGE_CLIM = (0.1, 1.3)
VARIANTS = ("map", "replay", "range")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--path", type=Path, help="A .npy file or directory of frames. Defaults to the bundled sample.")
    parser.add_argument("--out-dir", type=Path, help="Output directory. Defaults to Images/ in the repository.")
    parser.add_argument(
        "--only",
        choices=VARIANTS,
        action="append",
        help="Render only this variant. Repeat to pick several. Defaults to all.",
    )
    parser.add_argument("--size", default="800x450", help="Output size WIDTHxHEIGHT for the 3D renders.")
    parser.add_argument(
        "--min-support",
        type=int,
        default=MIN_SUPPORT,
        help=f"Keep map points seen by at least this many other revolutions. Defaults to {MIN_SUPPORT}.",
    )
    parser.add_argument(
        "--replay-revolutions",
        type=int,
        default=40,
        help="Length of the replay in revolutions (2 per second). Defaults to 40.",
    )
    return parser.parse_args()


# --- data ------------------------------------------------------------------------------------


def valid_mask(raw: np.ndarray) -> np.ndarray:
    distances = raw[:, 0]
    return (distances >= MIN_RANGE_MM) & (distances < MAX_RANGE_MM)


def headings_deg(xyz: np.ndarray) -> np.ndarray:
    """Horizontal direction of each point in the scanner frame, 0-360 deg."""
    return np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) % 360


def load_raw_frames(frame_paths: list[Path]) -> list[np.ndarray]:
    return [np.load(path) for path in frame_paths]


def transform(points: np.ndarray, pose: np.ndarray) -> np.ndarray:
    return points @ pose[:3, :3].T + pose[:3, 3]


def yaw_of(pose: np.ndarray) -> float:
    return float(np.arctan2(pose[1, 0], pose[0, 0]))


def interpolate_pose(a: np.ndarray, b: np.ndarray, t: float) -> np.ndarray:
    """Planar pose between `a` and `b`, for smooth scanner motion between revolutions."""
    yaw_a, yaw_b = yaw_of(a), yaw_of(b)
    yaw = yaw_a + t * ((yaw_b - yaw_a + np.pi) % (2 * np.pi) - np.pi)
    pose = np.eye(4)
    pose[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    pose[:3, 3] = (1 - t) * a[:3, 3] + t * b[:3, 3]
    return pose


@dataclass
class SequenceMap:
    raw_frames: list[np.ndarray]  # re-timed raw readings, one array per revolution
    clouds: list[np.ndarray]  # valid points of each revolution in the scanner frame
    poses: list[np.ndarray]  # 4x4 scanner -> map
    points: np.ndarray  # map points that passed the support filter
    frame_ids: np.ndarray  # revolution index of each map point
    floor_z: float  # floor height relative to the sensor head (m)

    @property
    def trajectory(self) -> np.ndarray:
        return np.array([pose[:3, 3] for pose in self.poses])

    def bounds_center_and_size(self) -> tuple[np.ndarray, float]:
        lo, hi = np.percentile(self.points[:, :2], [1, 99], axis=0)
        center = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, -0.05])
        return center, float(np.max(hi - lo))


def estimate_floor_z(raw_frames: list[np.ndarray]) -> float:
    """Floor height from the bottom channel's hits, or just below the lowest points without them."""
    floor_hits = []
    for raw in raw_frames:
        hits = raw[valid_mask(raw) & (raw[:, 2] == FLOOR_CHANNEL)]
        floor_hits.append(convert_to_cartesian(hits)[:, 2])
    floor_hits = np.concatenate(floor_hits)
    if len(floor_hits) >= 50:
        return float(np.median(floor_hits))
    every_z = np.concatenate([convert_to_cartesian(raw[valid_mask(raw)])[:, 2] for raw in raw_frames])
    return float(np.percentile(every_z, 2)) if len(every_z) else -0.2


def build_map(encoder_frames: list[np.ndarray], min_support: int = MIN_SUPPORT) -> SequenceMap:
    raw_frames = [retime_azimuths(raw) for raw in encoder_frames]
    clouds = [convert_to_cartesian(raw[valid_mask(raw)]) for raw in raw_frames]
    scans = []
    for raw in raw_frames:
        wall = valid_mask(raw) & (raw[:, 2] <= REGISTRATION_MAX_CHANNEL) & (raw[:, 0] < REGISTRATION_MAX_RANGE_MM)
        scans.append(convert_to_cartesian(raw[wall])[:, :2])
    poses, scores = register_sequence(scans)
    points, frame_ids = accumulate(clouds, poses)
    # A short recording cannot confirm points by many other revolutions.
    min_support = min(min_support, len(raw_frames) - 1)
    support = frame_support(points, frame_ids)
    keep = support >= min_support
    if len(poses) > 1:
        print(f"  registered {len(poses)} revolutions, median fit {np.nanmedian(scores[1:]) * 100:.1f} cm")
    print(f"  kept {keep.sum()} of {len(points)} points seen by >= {min_support} other revolutions")
    if not keep.any():
        raise SystemExit("No map points left to draw. Try a lower --min-support or a longer recording.")
    floor_z = estimate_floor_z(raw_frames)
    return SequenceMap(raw_frames, clouds, poses, points[keep], frame_ids[keep], floor_z)


# --- output helpers ---------------------------------------------------------------------------


def save_gif(frames: list[np.ndarray], path: Path, fps: float, colors: int = 255) -> None:
    """Write RGB frames as a looping GIF with one shared palette (less flicker, smaller file)."""
    from PIL import Image

    images = [Image.fromarray(frame[..., :3]) for frame in frames]
    sample = Image.new("RGB", (images[0].width, images[0].height * min(len(images), 6)))
    for k, image in enumerate(images[:: max(1, len(images) // 6)][:6]):
        sample.paste(image, (0, k * image.height))
    palette = sample.quantize(colors=colors, method=Image.Quantize.MEDIANCUT)
    paletted = [image.quantize(palette=palette, dither=Image.Dither.NONE) for image in images]
    paletted[0].save(
        path,
        save_all=True,
        append_images=paletted[1:],
        duration=round(1000 / fps),
        loop=0,
        optimize=True,
        disposal=1,
    )
    print(f"  wrote {path} ({path.stat().st_size / 1e6:.1f} MB, {len(frames)} frames)")


def make_plotter(size: tuple[int, int]):
    import pyvista as pv

    plotter = pv.Plotter(off_screen=True, window_size=size)
    plotter.set_background(BACKGROUND)
    plotter.enable_anti_aliasing("ssaa")
    return plotter


def add_floor_grid(
    plotter, center: np.ndarray, size: float, floor_z: float, scale: float = 1.0, spacing: float = 0.5
) -> None:
    """Faint 0.5 m grid on the floor for scale."""
    import pyvista as pv

    half = np.ceil(size * 0.65 / spacing) * spacing
    ticks = np.arange(-half, half + 1e-9, spacing)
    cx, cy = np.round(center[:2] / spacing) * spacing
    lines = [pv.Line((cx + t, cy - half, floor_z), (cx + t, cy + half, floor_z)) for t in ticks]
    lines += [pv.Line((cx - half, cy + t, floor_z), (cx + half, cy + t, floor_z)) for t in ticks]
    plotter.add_mesh(pv.merge(lines), color=GRID_COLOR, line_width=1.2 * scale)


def show_scanner(plotter, pose: np.ndarray) -> None:
    """Sensor head as a small cylinder at the scanner pose."""
    import pyvista as pv

    head = pv.Cylinder(center=pose[:3, 3], direction=(0, 0, 1), radius=0.045, height=0.07, resolution=40)
    plotter.add_mesh(head, name="scanner", color=SCANNER_COLOR, smooth_shading=True, specular=0.4)


def show_path(plotter, trajectory: np.ndarray, floor_z: float, scale: float = 1.0) -> None:
    """The scanner's path so far, drawn on the floor."""
    import pyvista as pv

    if len(trajectory) < 2:
        plotter.remove_actor("path")
        return
    on_floor = trajectory.copy()
    on_floor[:, 2] = floor_z + 0.002
    plotter.add_mesh(pv.lines_from_points(on_floor), name="path", color=PATH_COLOR, line_width=2 * scale, opacity=0.8)


def show_points(plotter, name: str, xyz: np.ndarray, size: float, opacity: float = 1.0) -> None:
    import pyvista as pv

    if len(xyz) == 0:
        plotter.remove_actor(name)
        return
    mesh = pv.PolyData(np.asarray(xyz, dtype=np.float32))
    mesh["height"] = xyz[:, 2]
    plotter.add_mesh(
        mesh,
        name=name,
        scalars="height",
        cmap="turbo",
        clim=HEIGHT_CLIM,
        point_size=size,
        render_points_as_spheres=True,
        opacity=opacity,
        show_scalar_bar=False,
    )


def orbit_camera(plotter, azimuth_deg: float, elevation_deg: float, distance: float, target) -> None:
    az, el = np.radians(azimuth_deg), np.radians(elevation_deg)
    offset = distance * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    plotter.camera.focal_point = tuple(target)
    plotter.camera.position = tuple(np.asarray(target) + offset)
    plotter.camera.up = (0, 0, 1)
    plotter.camera.view_angle = 35


# --- variant A: registered map ----------------------------------------------------------------


def render_map(seq: SequenceMap, out_dir: Path, size: tuple[int, int], hold_frames: int = 12) -> None:
    print("map: rendering")
    center, extent = seq.bounds_center_and_size()
    distance = 1.75 * extent

    # Hero still at twice the size (point and line sizes are in pixels, so they are scaled along).
    hero = make_plotter((size[0] * 2, size[1] * 2))
    add_floor_grid(hero, center, extent, seq.floor_z, scale=2)
    show_points(hero, "map", seq.points, size=12)
    show_path(hero, seq.trajectory, seq.floor_z, scale=2)
    show_scanner(hero, seq.poses[-1])
    orbit_camera(hero, azimuth_deg=-70, elevation_deg=40, distance=distance, target=center)
    hero_path = out_dir / "map_hero.png"
    hero.screenshot(hero_path)
    hero.close()
    print(f"  wrote {hero_path}")

    # Build-up: the map grows revolution by revolution (every third one keeps the GIF small),
    # the current revolution drawn larger, then the finished map keeps turning for a moment.
    plotter = make_plotter(size)
    add_floor_grid(plotter, center, extent, seq.floor_z)
    steps = list(range(0, len(seq.poses), 3)) + [len(seq.poses) - 1] * hold_frames
    frames = []
    for k, i in enumerate(steps):
        show_points(plotter, "map", seq.points[seq.frame_ids <= i], size=6)
        current = transform(seq.clouds[i], seq.poses[i]) if k < len(steps) - hold_frames else np.empty((0, 3))
        show_points(plotter, "current", current, size=10)
        show_path(plotter, seq.trajectory[: i + 1], seq.floor_z)
        show_scanner(plotter, seq.poses[i])
        orbit_camera(plotter, azimuth_deg=-70 + 180 * k / len(steps), elevation_deg=40, distance=distance, target=center)
        frames.append(plotter.screenshot(return_img=True))
    plotter.close()
    save_gif(frames, out_dir / "map_orbit.gif", fps=6, colors=128)


# --- variant B: real-time replay inside the map -----------------------------------------------


def busiest_segment(trajectory: np.ndarray, length: int) -> int:
    """Start index of the `length`-revolution stretch where the scanner travels farthest."""
    steps = np.linalg.norm(np.diff(trajectory[:, :2], axis=0), axis=1)
    if len(steps) < length:
        return 0
    travelled = np.convolve(steps, np.ones(length - 1), mode="valid")
    return int(np.argmax(travelled))


def render_replay(
    seq: SequenceMap,
    out_dir: Path,
    size: tuple[int, int],
    revolutions: int = 40,
    steps_per_rev: int = 8,
    trail: int = 3,
) -> None:
    """Revolutions revealed in recording order while the eight beams sweep, at the hardware's 2 Hz.

    The finished map stays in the background as a faint ghost, so the scan can be seen moving
    through the surroundings it is measuring.
    """
    import pyvista as pv

    print("replay: rendering")
    center, extent = seq.bounds_center_and_size()
    plotter = make_plotter(size)
    add_floor_grid(plotter, center, extent, seq.floor_z)
    show_points(plotter, "ghost", seq.points, size=5, opacity=0.22)
    # A fixed camera keeps the GIF small: only the moving scan changes between frames.
    orbit_camera(plotter, azimuth_deg=-70, elevation_deg=55, distance=2.05 * extent, target=center)

    first = busiest_segment(seq.trajectory, revolutions)
    last = min(len(seq.poses), first + revolutions)
    frames = []
    for rev in range(first, last):
        raw = seq.raw_frames[rev]
        valid = valid_mask(raw)
        progress = (np.arange(len(raw)) + 1) / len(raw)  # readings arrive in row order
        pose = seq.poses[rev]
        xyz = transform(convert_to_cartesian(raw), pose)
        unit_raw = raw.copy()
        unit_raw[:, 0] = 1000
        directions = convert_to_cartesian(unit_raw)
        next_pose = seq.poses[min(rev + 1, len(seq.poses) - 1)]

        for age in range(1, trail + 1):
            if rev - age < first:
                plotter.remove_actor(f"trail{age}")
                continue
            old = transform(seq.clouds[rev - age], seq.poses[rev - age])
            show_points(plotter, f"trail{age}", old, size=8 - age, opacity=0.6 / age)

        for step in range(1, steps_per_rev + 1):
            fraction = step / steps_per_rev
            head = interpolate_pose(pose, next_pose, fraction)
            show_points(plotter, "current", xyz[valid & (progress <= fraction)], size=12)
            show_path(plotter, np.vstack([seq.trajectory[: rev + 1], head[:3, 3]]), seq.floor_z)
            show_scanner(plotter, head)

            # The eight sensors point in eight headings and rotate together; draw each beam
            # from the scanner to where its latest reading in this revolution landed.
            origin = head[:3, 3]
            beams = []
            for channel in range(len(CHANNEL_ANGLE_LUT)):
                in_channel = np.flatnonzero((raw[:, 2] == channel) & (progress <= fraction))
                if len(in_channel) == 0:
                    continue
                latest = in_channel[-1]
                tip = xyz[latest] if valid[latest] else origin + head[:3, :3] @ directions[latest] * NO_RETURN_BEAM
                beams.append(pv.Line(origin, tip))
            if beams:
                plotter.add_mesh(pv.merge(beams), name="beams", color=BEAM_COLOR, line_width=2, opacity=0.65)

            frames.append(plotter.screenshot(return_img=True))
    plotter.close()
    save_gif(frames, out_dir / "replay.gif", fps=2 * steps_per_rev)


# --- variant C: range-image strip -------------------------------------------------------------


def range_image(raw: np.ndarray, bins: int = 48) -> np.ndarray:
    """8 x `bins` image of distance (m) by channel and heading; NaN where the beam got no return.

    Each cell takes the nearest sample (circularly) of its channel, so the ~25 samples per channel
    and revolution fill the whole panorama without gaps.
    """
    xyz = convert_to_cartesian(raw)
    heading = headings_deg(xyz)
    distance = raw[:, 0] / 1000.0
    valid = valid_mask(raw)
    centers = (np.arange(bins) + 0.5) * 360 / bins
    image = np.full((len(CHANNEL_ANGLE_LUT), bins), np.nan)
    for channel in range(len(CHANNEL_ANGLE_LUT)):
        sel = np.flatnonzero(raw[:, 2] == channel)
        if len(sel) == 0:
            continue
        diff = np.abs((centers[:, None] - heading[sel][None, :] + 180) % 360 - 180)
        nearest = sel[diff.argmin(axis=1)]
        image[channel] = np.where(valid[nearest], distance[nearest], np.nan)
    return image


def render_range(raw_frames: list[np.ndarray], out_dir: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    print("range: rendering")
    cmap = plt.get_cmap("plasma_r").copy()
    cmap.set_bad("#1c232b")

    fig, ax = plt.subplots(figsize=(8, 2.2), dpi=100)
    fig.patch.set_facecolor(BACKGROUND)
    ax.set_facecolor(BACKGROUND)
    artist = ax.imshow(
        range_image(raw_frames[0]),
        cmap=cmap,
        vmin=RANGE_CLIM[0],
        vmax=RANGE_CLIM[1],
        aspect="auto",
        interpolation="nearest",
        extent=(0, 360, len(CHANNEL_ANGLE_LUT) - 0.5, -0.5),
    )
    ax.set_xticks(range(0, 361, 45))
    ax.set_xticklabels([f"{d}°" for d in range(0, 361, 45)])
    ax.set_yticks(range(len(CHANNEL_ANGLE_LUT)))
    ax.set_yticklabels([f"{a:+.0f}°" for a in CHANNEL_ANGLE_LUT])
    ax.set_xlabel("heading", color=TEXT_COLOR)
    ax.set_ylabel("elevation", color=TEXT_COLOR)
    ax.tick_params(colors=TEXT_COLOR, labelsize=8)
    for spine in ax.spines.values():
        spine.set_visible(False)
    bar = fig.colorbar(artist, ax=ax, pad=0.015, fraction=0.04)
    bar.set_label("distance (m)", color=TEXT_COLOR)
    bar.ax.tick_params(colors=TEXT_COLOR, labelsize=8)
    bar.outline.set_visible(False)
    fig.tight_layout()

    frames = []
    for raw in raw_frames:
        artist.set_data(range_image(raw))
        fig.canvas.draw()
        frames.append(np.asarray(fig.canvas.buffer_rgba())[..., :3].copy())

    still_index = len(raw_frames) // 3
    artist.set_data(range_image(raw_frames[still_index]))
    still_path = out_dir / "range_strip.png"
    fig.savefig(still_path, dpi=200, facecolor=BACKGROUND)
    print(f"  wrote {still_path}")
    plt.close(fig)
    save_gif(frames, out_dir / "range_strip.gif", fps=4)


def main() -> None:
    args = parse_args()
    input_path, using_sample = resolve_input_path(args.path)
    if args.out_dir:
        out_dir = args.out_dir.resolve()
    elif using_sample:
        out_dir = repo_root() / "Images"
    else:
        recording = input_path.parent if input_path.is_file() else input_path
        # data/sequences/<name>/lidar_points -> data/sequences/<name>/media
        out_dir = (recording.parent if recording.name == "lidar_points" else recording) / "media"
    out_dir.mkdir(parents=True, exist_ok=True)
    width, height = (int(v) for v in args.size.lower().split("x"))
    raw_frames = load_raw_frames(collect_frame_paths(input_path))
    print(f"Loaded {len(raw_frames)} frame(s) from {input_path}; writing to {out_dir}")

    variants = args.only or VARIANTS
    if "map" in variants or "replay" in variants:
        print("Registering revolutions into one map")
        seq = build_map(raw_frames, args.min_support)
        if "map" in variants:
            render_map(seq, out_dir, (width, height))
        if "replay" in variants:
            render_replay(seq, out_dir, (width, height), revolutions=args.replay_revolutions)
    if "range" in variants:
        render_range(raw_frames, out_dir)


if __name__ == "__main__":
    main()
