# CARLA trajectory visualization

Replay a precomputed trajectory (position/heading/speed) as vehicles in CARLA.

## Usage

1. Prepare a trajectory CSV with columns `t,agent_id,x,y,z,yaw,speed`
   (see `data/hospital_trajectory.csv` for an example).
2. Make sure a CARLA server is running and `config.yaml` points at it.
3. From the `python/` folder, run:

```
python3 replay.py --config ../config.yaml --trajectory ../data/hospital_trajectory.csv
```

See `CLAUDE.md` for design details and `--speed-factor`/`--scale`/`--interpolate-hz` options.
