# CARLA trajectory visualization

## Goal
Visualize a Julia-precomputed trajectory (position/heading/speed) in CARLA.
No real-time coupling — compute the full trajectory first, replay it after.
Discrete waypoints played back as-is; interpolation is future work.
v0: CARLA vehicle physics off, position forced via transform. Later: wheel
rotation, steering, body lean. Supports one or more agents on a shared timeline.

## Layout
```
config.yaml                  server connection, coordinate transform, playback options
requirements.txt             pip deps (carla version must match the server exactly)
data/example_trajectory.csv  t,agent_id,x,y,z,yaw,speed sample (single agent, curved path)
data/hospital_trajectory.csv real Reduced-GOOP hospital MPC result (4 agents, 5s, 51 steps)
julia/export_trajectory.jl   CSV exporter for this schema (stdlib only, no deps)
python/trajectory_io.py      CSV -> Waypoint list, grouped by agent_id
python/replay.py             connects to CARLA, spawns per-agent vehicles, set_transform per waypoint
```

Only commit within this repo (CARLA) — nothing in `../Reduced-GOOP` or
`../ScholtesReducedGOOP.jl` gets committed from here, even though export
scripts were added to those repos as files. Their output gets copied into
`CARLA/data/` as test fixtures.

```
../Reduced-GOOP/experiments/export_to_carla.jl   (not committed, file only)
    Exports an existing hospital MPC result (mpc_solution.jld2) to the CARLA
    schema. State is position-only, so yaw/speed are derived from each step's
    control u=(ux,uy): yaw=atan(uy,ux), speed=‖u‖. Reads an existing run_dir,
    doesn't resolve. Run:
    `cd ../Reduced-GOOP && julia --project=. experiments/export_to_carla.jl <run_dir> [output_path]`
    data/hospital_trajectory.csv came from
    `data/Hospital_open_loop/debug/2026-08-11T08-41-41`.

../ScholtesReducedGOOP.jl/examples/hospital_to_carla.jl   (not committed, file only)
    Same purpose, ported single-shot version (hospital.jl, 3 agents, 0.3s),
    resolves fresh each time. Unused now — the real MPC result is a better
    fixture. Still usable if needed.
```

## Key decisions
- `vehicle.set_simulate_physics(False)` + `set_transform()` per waypoint (v0). See `replay.py`.
- Multi-agent: waypoints grouped by agent_id, one vehicle each. Each agent holds
  its last waypoint (step function, no interpolation) as the shared timeline advances.
- Camera: `playback.follow_camera: true` recenters the spectator to a top-down
  view of all agents every frame — confirmed this jitters, since the centroid
  shifts slightly each frame (2026-09-21, hospital_trajectory.csv visual test).
  **Default is now `false`**: `replay.py` leaves the spectator alone, so whatever
  view was set in the CARLA window before running stays put (verified spectator
  transform is identical before/after a run).
- `playback.default_camera`: spectator transform applied once at start,
  regardless of `follow_camera`. Current value was captured from the CARLA
  window on 2026-09-21 while framing the hospital scenario (verified it's
  restored exactly even if something else moves the spectator mid-spawn).
- `vehicle.by_agent` in `config.yaml`: per-agent blueprint/color override,
  falls back to the top-level default. Confirmed `vehicle.ford.ambulance` and
  `vehicle.carlamotors.carlacola` exist in CARLA 0.9.16 and support `color`.
  Current hospital_trajectory.csv mapping (2026-09-21, user-confirmed): agent
  1 = high priority (ambulance), 4 = low priority (cargo truck), 2/3 = medium
  (tesla model3, default). Verified by spawning each and checking type_id.
- Coordinate transform (`flip_y`, `negate_yaw`) assumes Julia uses a
  right-handed frame (x=forward, y=left, yaw=CCW, rad); CARLA is left-handed
  (x=forward, y=right, yaw=CW, deg). **Not yet visually verified in CARLA** —
  playback runs without errors, but turning direction hasn't been eyeballed.
- `origin_offset` / `scale` / `center_on_offset`: visualization-only transform,
  applied in memory by `replay.py` — never touches the source CSV (especially
  hospital_trajectory.csv, which is a real MPC result).
  - `center_on_offset: true` recenters the trajectory's x/y bounding-box
    center onto `origin_offset`, regardless of the file's original local origin.
  - `scale` multiplies x/y/z after recentering (not yaw). hospital_trajectory.csv
    is ~4m natively (indoor-robot scale), needs ~6-7x to read clearly in a
    Town10HD_Opt junction; example_trajectory.csv (~16.5x14.5m) needs less
    (`--scale 2.0`) — scale differs per file, so override at runtime. Note:
    the `speed` label shows the CSV's raw value, not adjusted for scale.
  - `origin_offset` exists because world origin (0,0) overlaps a building in
    Town10HD_Opt — spawn collision is no longer an issue (vehicles spawn
    airborne and drop in), but placement still matters visually. Set to
    junction 189's center (-47.8, 20.4), bbox ~38.2 x 44.3m. Re-tune for
    other maps/scenarios.
- Spawning multiple agents closer together than a vehicle's bounding box
  (e.g. hospital scenario, 1-4m apart) trips CARLA's spawn-time collision
  check. `replay.py` spawns 50m up, then `set_transform()`s down — bypasses
  spawn collision but vehicles can visually overlap on the first frame
  (known v0 limitation, same bucket as interpolation).
- Playback paced with `time.sleep`; `playback.speed_factor` or
  `replay.py --speed-factor` (CLI wins). Switching to synchronous mode
  (`fixed_delta_seconds`) later would make timing more deterministic.
- Speed shown via `world.debug.draw_string` as `"{agent_id}: {speed} m/s"`
  above each vehicle (v0). No arrows/wheel rotation yet.
- `data/hospital_trajectory.csv` (Δt=0.1, 51 steps, 5s real closed-loop MPC)
  reads fine at `speed_factor=1.0`; use `--speed-factor` to adjust if needed.

## Remote environment
- CARLA server runs on a separate remote machine (needs GPU, no official macOS
  support). `ssh ji5332@ase-a71908.ece.utexas.edu`.
- Install path: `~/Documents/carla`, launched via its `.sh` script (usually
  `CarlaUE4.sh` — confirm with `ls ~/Documents/carla`).
- CARLA version: **0.9.16**, pinned in `requirements.txt`.
- **Recommended first test: go to the remote machine and run server+client
  both on localhost** (removes network/firewall/VNC variables, isolates the
  pipeline logic). Needs `config.yaml`'s `server.host` set to `127.0.0.1`.
  **2026-09-21**: done this way, on a machine with hostname `asg-a69681`
  (different from `ase-a71908.ece.utexas.edu`, but same CARLA 0.9.16 install
  under `~/Documents/carla`, GPU (RTX 3070), DISPLAY (:1) — re-check `hostname`
  if unsure). `server.host` is committed as `127.0.0.1` right now —
  **switch back to the real remote address when returning to Mac↔server**.
  Both example_trajectory.csv and hospital_trajectory.csv run to completion
  without errors via `CarlaUE4.sh -windowed` + `python replay.py` (see "Key
  decisions" above for origin_offset/spawn workaround). **Coordinate-transform
  direction still needs a visual check.**
- For later, going back to Mac ↔ remote:
  - Monitor is physically attached to the remote machine — SSH alone shows no
    display. Either sit at the machine, or mirror it with `x11vnc` (not set up yet).
  - If campus firewall blocks port 2000 directly, tunnel with
    `ssh -L 2000:localhost:2000 -L 2001:localhost:2001 ji5332@ase-a71908.ece.utexas.edu`
    and set `config.yaml`'s host to `127.0.0.1`.

## Not done yet / next steps
- [x] Fill in `config.yaml` server.host with the real remote address (switch to 127.0.0.1 for local testing)
- [x] Match `requirements.txt` to the remote server's CARLA version (0.9.16)
- [x] Multi-agent support (`replay.py`, `trajectory_io.py`) + hospital MPC export
      (`export_to_carla.jl` in Reduced-GOOP, not committed) — Julia run and CSV
      parsing verified locally; CARLA connection itself untested until now
- [x] Ran CARLA server locally via `~/Documents/carla`'s `.sh` script (on a
      machine matching the remote spec)
- [x] First connection test on the same machine: host set to `127.0.0.1`,
      `python replay.py` with example_trajectory.csv (single agent) — pip deps were already installed
- [x] Multi-agent test with hospital_trajectory.csv (4 agents, 5s) — found and
      fixed two spawn-collision issues (origin_offset, airborne spawn +
      set_transform, orphaned-actor leak from the spawn loop being outside
      try/finally). Both trajectories now run to completion
- [x] Scaled hospital_trajectory.csv to junction 189's scale (`scale`,
      `center_on_offset: true`) — visually confirmed in the CARLA window (2026-09-21)
- [x] Found and fixed per-frame camera jitter — `follow_camera` was
      re-centering every frame; now defaults to `false` so the pre-set CARLA view holds
- [ ] Playback runs cleanly, but coordinate-transform direction (`flip_y`,
      `negate_yaw`) still needs a visual check — confirm vehicles turn the expected way
- **Currently testing with hospital_trajectory.csv only** (2026-09-21~) —
  example_trajectory.csv is parked, revisit when needed
- [ ] (optional) SSH tunnel or x11vnc setup for Mac ↔ remote later
- [ ] Wire up the real Julia trajectory-generation code to the `export_trajectory.jl` schema
- [ ] Wheel rotation, steering visualization, body lean (v1)

## Git
- remote: `git@github.com:iamjaehan/CARLA.git` (origin), branch `master`
