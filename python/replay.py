"""Replay a pre-computed discrete trajectory in CARLA (no physics simulation).

Usage:
    python replay.py --config ../config.yaml --trajectory ../data/example_trajectory.csv
"""
from __future__ import annotations

import argparse
import math
import time

import carla
import yaml

from trajectory_io import Waypoint, load_trajectory


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


def update_spectator(world: carla.World, transform: carla.Transform):
    spectator = world.get_spectator()
    behind = transform.get_forward_vector() * -8.0
    cam_location = transform.location + behind + carla.Location(z=4.0)
    cam_rotation = carla.Rotation(pitch=-15.0, yaw=transform.rotation.yaw, roll=0.0)
    spectator.set_transform(carla.Transform(cam_location, cam_rotation))


def run(config_path: str, trajectory_path: str):
    with open(config_path) as f:
        cfg = yaml.safe_load(f)

    waypoints = load_trajectory(trajectory_path)
    if not waypoints:
        raise ValueError(f"No waypoints found in {trajectory_path}")

    client = carla.Client(cfg["server"]["host"], cfg["server"]["port"])
    client.set_timeout(cfg["server"]["timeout"])

    if cfg.get("map"):
        world = client.load_world(cfg["map"])
    else:
        world = client.get_world()

    z_offset = cfg["vehicle"]["z_offset"]
    coord_cfg = cfg["coordinate_transform"]
    speed_factor = cfg["playback"]["speed_factor"]

    first_transform = to_carla_transform(waypoints[0], coord_cfg, z_offset)
    vehicle = spawn_vehicle(world, cfg["vehicle"]["blueprint"], first_transform)

    try:
        prev_t = waypoints[0].t
        for wp in waypoints:
            transform = to_carla_transform(wp, coord_cfg, z_offset)
            vehicle.set_transform(transform)

            if cfg["playback"]["show_speed_label"]:
                world.debug.draw_string(
                    transform.location + carla.Location(z=2.5),
                    f"{wp.speed:.1f} m/s",
                    life_time=max((wp.t - prev_t) / speed_factor, 0.05),
                    color=carla.Color(255, 255, 0),
                )

            if cfg["playback"]["follow_camera"]:
                update_spectator(world, transform)

            dt = (wp.t - prev_t) / speed_factor
            if dt > 0:
                time.sleep(dt)
            prev_t = wp.t
    finally:
        vehicle.destroy()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="../config.yaml")
    parser.add_argument("--trajectory", default="../data/example_trajectory.csv")
    args = parser.parse_args()
    run(args.config, args.trajectory)
