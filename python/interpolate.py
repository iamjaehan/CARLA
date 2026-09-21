"""Fill in intermediate frames between discrete waypoints via linear interpolation.

Kept separate from replay.py so the interpolation method (currently linear,
maybe spline/etc. later) can change without touching playback logic.
"""
from __future__ import annotations

import math

from trajectory_io import Waypoint, group_by_agent


def _lerp(a: float, b: float, frac: float) -> float:
    return a + (b - a) * frac


def _lerp_angle(a: float, b: float, frac: float) -> float:
    """Interpolate a radian angle along its shorter direction."""
    delta = (b - a + math.pi) % (2 * math.pi) - math.pi
    return a + delta * frac


def estimate_source_hz(waypoints: list[Waypoint]) -> float:
    """Average sample rate of one agent's timestamps (any agent, they share a timeline)."""
    agent_id = waypoints[0].agent_id
    ts = sorted(wp.t for wp in waypoints if wp.agent_id == agent_id)
    if len(ts) < 2:
        return 0.0
    return (len(ts) - 1) / (ts[-1] - ts[0])


def _resample_agent(wps: list[Waypoint], times: list[float]) -> list[Waypoint]:
    """Resample one agent's sorted waypoints onto `times` (also sorted)."""
    agent_id = wps[0].agent_id
    out = []
    i = 0  # (wps[i], wps[i + 1]) is the pair currently bracketing t
    for t in times:
        if t <= wps[0].t:
            src = wps[0]
        elif t >= wps[-1].t:
            src = wps[-1]
        else:
            while i + 1 < len(wps) - 1 and wps[i + 1].t <= t:
                i += 1
            a, b = wps[i], wps[i + 1]
            frac = (t - a.t) / (b.t - a.t)
            out.append(
                Waypoint(
                    t=t,
                    agent_id=agent_id,
                    x=_lerp(a.x, b.x, frac),
                    y=_lerp(a.y, b.y, frac),
                    z=_lerp(a.z, b.z, frac),
                    yaw=_lerp_angle(a.yaw, b.yaw, frac),
                    speed=_lerp(a.speed, b.speed, frac),
                )
            )
            continue
        out.append(Waypoint(t, agent_id, src.x, src.y, src.z, src.yaw, src.speed))
    return out


def interpolate_trajectory(waypoints: list[Waypoint], target_hz: float) -> list[Waypoint]:
    """Resample every agent onto a shared uniform grid at ~target_hz, spanning
    the full trajectory's [min t, max t]. Densifies below target_hz, decimates
    above it — always produces an evenly spaced grid either way.
    """
    if not waypoints or target_hz <= 0:
        return list(waypoints)

    t_min = min(wp.t for wp in waypoints)
    t_max = max(wp.t for wp in waypoints)
    if t_max <= t_min:
        return list(waypoints)

    n = max(round((t_max - t_min) * target_hz), 1) + 1
    times = [t_min + i * (t_max - t_min) / (n - 1) for i in range(n)]

    out: list[Waypoint] = []
    for wps in group_by_agent(waypoints).values():
        out.extend(_resample_agent(wps, times))
    out.sort(key=lambda w: w.t)
    return out
