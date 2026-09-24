# CARLA trajectory visualization

Replay a precomputed trajectory (position/heading/speed) as vehicles in CARLA.

## Requirements

- CARLA **0.9.16** — server and Python client must match exactly (`pip install -r requirements.txt` pins `carla==0.9.16`)
- Python 3.x
- Julia, only if you're generating trajectories via `julia/export_trajectory.jl`

## Usage

1. Prepare a trajectory CSV with columns `t,agent_id,x,y,z,yaw,speed`
   (see `data/hospital_trajectory.csv` for an example).
2. Make sure a CARLA server is running and `config.yaml` points at it.
3. From the `python/` folder, run:

```
python3 replay.py --config ../config.yaml --trajectory ../data/hospital_trajectory.csv
```

See `CLAUDE.md` for design details and `--speed-factor`/`--scale`/`--interpolate-hz` options.

## CSV trajectory format

| column     | type   | unit | description                                                                    |
|------------|--------|------|----------------------------------------------------------------------------------|
| `t`        | float  | s    | time since trajectory start                                                    |
| `agent_id` | string | -    | groups waypoints into one vehicle; multiple agents can share a file/timeline   |
| `x`        | float  | m    | world-frame x position, before `config.yaml`'s `scale`/`origin_offset`         |
| `y`        | float  | m    | world-frame y position                                                         |
| `z`        | float  | m    | world-frame height; `0` for flat/2D scenarios                                  |
| `yaw`      | float  | rad  | heading, standard math convention (CCW from +x axis, e.g. `atan2(vy, vx)`)     |
| `speed`    | float  | m/s  | forward speed; only used for the on-screen debug label (no wheel/steer animation yet) |

Waypoints can be sparse or irregularly spaced per agent — `replay.py` holds each
agent's last waypoint until its next one (or, if `playback.interpolate_hz` is
set, `python/interpolate.py` resamples everything onto a shared uniform-rate
grid first: linear for x/y/z/speed, shortest-angle for yaw).
