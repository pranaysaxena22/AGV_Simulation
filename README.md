# AGV Gazebo Simulation

A 3D physics-based digital twin of an AGV (Automated Guided Vehicle) fleet for a smart-factory floor, built on **Gazebo Sim (Harmonic) + ROS 2 (Jazzy)**, driven in real time by a discrete-event Python simulator over Redis.

The simulator remains the single source of truth — routing, traffic management, battery, and task dispatch all happen there. Gazebo is a physics-accurate render/motion layer that mirrors what the simulator decides, not an independent decision-maker.

---

## Architecture

Python Simulator (SimPy) → Redis (agv:telemetry, 10Hz per AGV) → ROS 2 Bridge Node → Gazebo Sim
(routing, traffic, (pub/sub message bus) (velocity control, (physics,
battery, tasks) state → visuals) rendering)


- **Simulator**: owns all "brain" logic. Publishes each AGV's position, heading, speed, state, and payload status 10x/second.
- **Bridge node**: translates telemetry into `cmd_vel` commands (proportional + feedforward control) that drive each AGV's physical body in Gazebo. Also syncs visual state indicators.
- **Gazebo**: simulates real rigid-body physics — actual contact, actual motion — not a teleporting sprite.

---

## Status

| Phase | Description | Status |
|---|---|---|
| 0 | Environment setup (ROS 2 Jazzy, Gazebo Harmonic, Redis) | ✅ Done |
| 1 | World generator — floor layout, stations, waypoints, coordinate system | ✅ Done |
| 2 | Pose-puppeting spike + live browser telemetry viewer | ✅ Done |
| 3 | Velocity-control bridge — real physics-driven AGV movement | ✅ Done |
| 4 | Visual state indicators — beacons, charge lamps, payload crates, blocked-corridor markers | ✅ Done |
| 4b | In-Gazebo text status labels | 🧪 Experimental |
| 6 | Real wheel physics (differential drive) | 🔧 In progress |
| 5 / 7 | Standalone browser 3D view + live physics feed | ⏳ Planned |
| 8 | Demo hardening (cold-start test, unattended stability run) | ⏳ Planned |

---

## Prerequisites

- [pixi](https://pixi.sh) package manager
- A machine with GPU support for Gazebo's GUI (headless mode also works for the physics-only server)
- No admin/sudo required — all dependencies resolve through pixi

Dependencies (ROS 2 Jazzy, Gazebo Harmonic / `gz-sim8`, Redis, and Python packages) are declared in `pixi.toml` and resolve automatically on first `pixi run`.

---

## Running the simulation

**One command brings up the full stack:**

```bash
cd local-iops/agents/agv-gazebo-bridge
./start_all.sh
```

This starts, in order:
1. Redis (message bus)
2. Gazebo (headless physics server)
3. `ros_gz_bridge` (ROS 2 ↔ Gazebo transport bridge)
4. `agv_gazebo_bridge.node` (velocity control + visual state sync)
5. The Python AGV simulator (seeds initial tasks, begins dispatch)
6. `agv-stream-bridge` (live telemetry feed for the browser viewer)
7. Gazebo GUI window

The script automatically kills any stale processes from a previous run before starting, so it's safe to re-run without manual cleanup.

**Stop everything:** `Ctrl+C` in the same terminal — this cleanly shuts down all six processes.

### Optional: browser-based live status viewer

For a plain-text, at-a-glance view of every AGV's live state (separate from the 3D Gazebo window):

```bash
cd local-iops/agents/agv-simulator/demo
pixi run python3 -m http.server 8899
```

Then open `http://localhost:8899/live_viewer.html` in a browser.

---

## Coordinate system

The floor layout (`shared/layouts/poc-floor.json`) uses the simulator's native 2D coordinates. Conversion to Gazebo's world frame:

gz_x = sim_x - 30
gz_y = sim_y - 20
gz_z = 0.2
yaw = radians(heading_degrees), wrapped to (-π, π]


---

---
