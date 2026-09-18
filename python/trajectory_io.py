"""Load discrete trajectory waypoints exported from Julia."""
from __future__ import annotations

import csv
from dataclasses import dataclass


@dataclass
class Waypoint:
    t: float
    x: float
    y: float
    z: float
    yaw: float  # radians
    speed: float  # m/s


def load_trajectory(path: str) -> list[Waypoint]:
    waypoints = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            waypoints.append(
                Waypoint(
                    t=float(row["t"]),
                    x=float(row["x"]),
                    y=float(row["y"]),
                    z=float(row["z"]),
                    yaw=float(row["yaw"]),
                    speed=float(row["speed"]),
                )
            )
    waypoints.sort(key=lambda w: w.t)
    return waypoints
