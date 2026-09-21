"""Replay a pre-computed discrete trajectory in CARLA (no physics simulation).

Supports one or more agents sharing a timeline (see trajectory_io.py schema).
Each agent gets its own vehicle actor; between two of an agent's waypoints its
transform just holds the last value (no interpolation yet).

Usage:
    python replay.py --config ../config.yaml --trajectory ../data/example_trajectory.csv
"""
from __future__ import annotations

import argparse
import math
import time

import carla
import yaml

from trajectory_io import Waypoint, group_by_agent, load_trajectory


def to_carla_transform(wp: Waypoint, cfg: dict, z_offset: float) -> carla.Transform:
    y = -wp.y if cfg["flip_y"] else wp.y
    yaw_rad = -wp.yaw if cfg["negate_yaw"] else wp.yaw
    location = carla.Location(x=wp.x, y=y, z=wp.z + z_offset)
    rotation = carla.Rotation(pitch=0.0, yaw=math.degrees(yaw_rad), roll=0.0)
    return carla.Transform(location, rotation)


def spawn_vehicle(world: carla.World, blueprint_name: str, transform: carla.Transform):
    bp_library = world.get_blueprint_library()
    blueprint = bp_library.find(blueprint_name)
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


def run(config_path: str, trajectory_path: str, speed_factor_override: float | None = None):
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    waypoints = load_trajectory(trajectory_path)
    if not waypoints:
        raise ValueError(f"No waypoints found in {trajectory_path}")

    agents = group_by_agent(waypoints)
    timeline = sorted({wp.t for wp in waypoints})

    client = carla.Client(cfg["server"]["host"], cfg["server"]["port"])
    client.set_timeout(cfg["server"]["timeout"])

    if cfg.get("map"):
        world = client.load_world(cfg["map"])
    else:
        world = client.get_world()

    z_offset = cfg["vehicle"]["z_offset"]
    coord_cfg = cfg["coordinate_transform"]
    speed_factor = speed_factor_override if speed_factor_override is not None else cfg["playback"]["speed_factor"]

    vehicles = {}
    cursors = {agent_id: 0 for agent_id in agents}
    for agent_id, wps in agents.items():
        transform = to_carla_transform(wps[0], coord_cfg, z_offset)
        vehicles[agent_id] = spawn_vehicle(world, cfg["vehicle"]["blueprint"], transform)

    try:
        prev_t = timeline[0]
        for t in timeline:
            transforms = []
            for agent_id, wps in agents.items():
                cursors[agent_id] = advance_cursor(wps, cursors[agent_id], t)
                wp = wps[cursors[agent_id]]
                transform = to_carla_transform(wp, coord_cfg, z_offset)
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
    args = parser.parse_args()
    run(args.config, args.trajectory, args.speed_factor)
