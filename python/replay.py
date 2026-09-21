"""Replay a pre-computed discrete trajectory in CARLA (no physics simulation).

Supports one or more agents sharing a timeline (see trajectory_io.py schema).
Each agent gets its own vehicle actor; between two of an agent's waypoints its
transform just holds the last value unless interpolate.py has densified them.

Usage:
    python replay.py --config ../config.yaml --trajectory ../data/example_trajectory.csv
"""
from __future__ import annotations

import argparse
import math
import time

import carla
import yaml

from interpolate import estimate_source_hz, interpolate_trajectory
from trajectory_io import Waypoint, group_by_agent, load_trajectory


def recenter_waypoints(waypoints: list[Waypoint]) -> None:
    """Shift all waypoints in place so their x/y bounding-box center is at (0, 0)."""
    xs = [wp.x for wp in waypoints]
    ys = [wp.y for wp in waypoints]
    center_x = (min(xs) + max(xs)) / 2
    center_y = (min(ys) + max(ys)) / 2
    for wp in waypoints:
        wp.x -= center_x
        wp.y -= center_y


def to_carla_transform(wp: Waypoint, cfg: dict, z_offset: float, scale: float) -> carla.Transform:
    y = -wp.y if cfg["flip_y"] else wp.y
    yaw_rad = -wp.yaw if cfg["negate_yaw"] else wp.yaw
    offset = cfg.get("origin_offset", {"x": 0.0, "y": 0.0})
    location = carla.Location(x=wp.x * scale + offset["x"], y=y * scale + offset["y"], z=wp.z * scale + z_offset)
    rotation = carla.Rotation(pitch=0.0, yaw=math.degrees(yaw_rad), roll=0.0)
    return carla.Transform(location, rotation)


def resolve_vehicle_spec(agent_id: str, vehicle_cfg: dict) -> tuple[str, str | None]:
    """Per-agent blueprint/color from vehicle_cfg["by_agent"], falling back to defaults."""
    spec = vehicle_cfg.get("by_agent", {}).get(str(agent_id), {})
    blueprint = spec.get("blueprint", vehicle_cfg["blueprint"])
    color = spec.get("color", vehicle_cfg.get("color"))
    return blueprint, color


def spawn_vehicle(world: carla.World, blueprint_name: str, color: str | None, transform: carla.Transform):
    bp_library = world.get_blueprint_library()
    blueprint = bp_library.find(blueprint_name)
    if color and blueprint.has_attribute("color"):
        blueprint.set_attribute("color", color)
    vehicle = world.spawn_actor(blueprint, transform)
    vehicle.set_simulate_physics(False)
    return vehicle


def update_spectator_topdown(world: carla.World, transforms: list[carla.Transform], height: float = 25.0):
    locations = [t.location for t in transforms]
    cx = sum(loc.x for loc in locations) / len(locations)
    cy = sum(loc.y for loc in locations) / len(locations)
    cz = sum(loc.z for loc in locations) / len(locations)
    spectator = world.get_spectator()
    spectator.set_transform(
        carla.Transform(
            carla.Location(x=cx, y=cy, z=cz + height),
            carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.0),
        )
    )


def advance_cursor(wps: list[Waypoint], cursor: int, t: float) -> int:
    while cursor + 1 < len(wps) and wps[cursor + 1].t <= t:
        cursor += 1
    return cursor


def run(
    config_path: str,
    trajectory_path: str,
    speed_factor_override: float | None = None,
    scale_override: float | None = None,
    interpolate_hz_override: float | None = None,
):
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    waypoints = load_trajectory(trajectory_path)
    if not waypoints:
        raise ValueError(f"No waypoints found in {trajectory_path}")

    interpolate_hz = (
        interpolate_hz_override if interpolate_hz_override is not None else cfg["playback"].get("interpolate_hz", 0)
    )
    if interpolate_hz:
        source_hz = estimate_source_hz(waypoints)
        waypoints = interpolate_trajectory(waypoints, interpolate_hz)
        print(f"Interpolated ~{source_hz:.1f}Hz -> {interpolate_hz:.1f}Hz ({len(waypoints)} waypoints)")

    if cfg["coordinate_transform"].get("center_on_offset", False):
        recenter_waypoints(waypoints)

    agents = group_by_agent(waypoints)
    timeline = sorted({wp.t for wp in waypoints})

    client = carla.Client(cfg["server"]["host"], cfg["server"]["port"])
    client.set_timeout(cfg["server"]["timeout"])

    if cfg.get("map"):
        world = client.load_world(cfg["map"])
    else:
        world = client.get_world()

    default_camera = cfg["playback"].get("default_camera")
    if default_camera:
        world.get_spectator().set_transform(
            carla.Transform(
                carla.Location(x=default_camera["x"], y=default_camera["y"], z=default_camera["z"]),
                carla.Rotation(pitch=default_camera["pitch"], yaw=default_camera["yaw"], roll=default_camera["roll"]),
            )
        )

    z_offset = cfg["vehicle"]["z_offset"]
    coord_cfg = cfg["coordinate_transform"]
    speed_factor = speed_factor_override if speed_factor_override is not None else cfg["playback"]["speed_factor"]
    scale = scale_override if scale_override is not None else coord_cfg.get("scale", 1.0)

    vehicles = {}
    cursors = {agent_id: 0 for agent_id in agents}
    try:
        for agent_id, wps in agents.items():
            transform = to_carla_transform(wps[0], coord_cfg, z_offset, scale)
            # Agents can start closer together than a vehicle's bounding box, which trips
            # CARLA's spawn-time collision check. Spawn high up, then drop into place.
            spawn_transform = carla.Transform(
                carla.Location(transform.location.x, transform.location.y, transform.location.z + 50.0),
                transform.rotation,
            )
            blueprint_name, color = resolve_vehicle_spec(agent_id, cfg["vehicle"])
            vehicle = spawn_vehicle(world, blueprint_name, color, spawn_transform)
            vehicle.set_transform(transform)
            vehicles[agent_id] = vehicle

        prev_t = timeline[0]
        for t in timeline:
            transforms = []
            for agent_id, wps in agents.items():
                cursors[agent_id] = advance_cursor(wps, cursors[agent_id], t)
                wp = wps[cursors[agent_id]]
                transform = to_carla_transform(wp, coord_cfg, z_offset, scale)
                vehicles[agent_id].set_transform(transform)
                transforms.append(transform)

                if cfg["playback"]["show_speed_label"]:
                    world.debug.draw_string(
                        transform.location + carla.Location(z=2.5),
                        f"{agent_id}: {wp.speed:.1f} m/s",
                        life_time=max((timeline[1] - timeline[0]) / speed_factor, 0.05)
                        if len(timeline) > 1
                        else 0.5,
                        color=carla.Color(255, 255, 0),
                    )

            if cfg["playback"]["follow_camera"]:
                update_spectator_topdown(world, transforms)

            dt = (t - prev_t) / speed_factor
            if dt > 0:
                time.sleep(dt)
            prev_t = t
    finally:
        for vehicle in vehicles.values():
            vehicle.destroy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="../config.yaml")
    parser.add_argument("--trajectory", default="../data/example_trajectory.csv")
    parser.add_argument(
        "--speed-factor",
        type=float,
        default=None,
        help="Override playback.speed_factor from config.yaml (e.g. 0.05 to play a short trajectory in slow motion)",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=None,
        help="Override coordinate_transform.scale from config.yaml (e.g. 2.0 to double the trajectory's spatial extent)",
    )
    parser.add_argument(
        "--interpolate-hz",
        type=float,
        default=None,
        help="Override playback.interpolate_hz from config.yaml (0 disables interpolation)",
    )
    args = parser.parse_args()
    run(args.config, args.trajectory, args.speed_factor, args.scale, args.interpolate_hz)
