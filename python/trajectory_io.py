"""Load discrete trajectory waypoints exported from Julia.

Schema: t,agent_id,x,y,z,yaw,speed  (one or more agents sharing a timeline)
"""
from __future__ import annotations

import csv
from dataclasses import dataclass


@dataclass
class Waypoint:
    t: float
    agent_id: str
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
                    agent_id=row["agent_id"],
                    x=float(row["x"]),
                    y=float(row["y"]),
                    z=float(row["z"]),
                    yaw=float(row["yaw"]),
                    speed=float(row["speed"]),
                )
            )
    waypoints.sort(key=lambda w: w.t)
    return waypoints


def group_by_agent(waypoints: list[Waypoint]) -> dict[str, list[Waypoint]]:
    agents: dict[str, list[Waypoint]] = {}
    for wp in waypoints:
        agents.setdefault(wp.agent_id, []).append(wp)
    for wps in agents.values():
        wps.sort(key=lambda w: w.t)
    return agents
