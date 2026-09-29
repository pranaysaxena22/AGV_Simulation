# AGV Gazebo Simulation

A 3D physics-based digital twin of an AGV (Automated Guided Vehicle) fleet, built on Gazebo Sim + ROS 2, driven by real telemetry from a discrete-event Python simulator.

## Status

| Phase | Description | Status |
|---|---|---|
| 0 | Environment setup (ROS 2 Jazzy, Gazebo Harmonic) | ✅ Done |
| 1 | World generator, floor layout, coordinate system | ✅ Done |
| 2 | Pose-puppeting spike + live browser viewer | ✅ Done |
| 3 | Velocity-control bridge (real physics-driven movement) | ✅ Done |
| 4 | State visualization (beacons, charge lamps, crates, blocked-corridor markers) | ✅ Done |
| 6 | Real wheel physics (differential drive) | 🔧 In progress |
| 5, 7 | Browser 3D view + live physics feed | ⏳ Planned |
| 8 | Demo hardening | ⏳ Planned |

## What this does

- Simulator (Python, discrete-event) owns routing, traffic management, and battery/task logic
- Publishes live telemetry (position, heading, state) over Redis at 10Hz per AGV
- A ROS 2 bridge node translates that into velocity commands, driving each AGV's physical body in Gazebo with real rigid-body physics
- Visual state indicators (color-coded markers, lamps, cargo crates) reflect the fleet's real-time status directly in the 3D scene

## Key technical notes

- Coordinate conversion between the 2D floor layout and Gazebo's world frame is handled in `scripts/gen_gazebo_world.py`
- All state-driven visual updates use a dirty-check pattern (fire only on actual state transitions, not every tick) to avoid overloading the simulation
- `start_all.sh` orchestrates the full stack (Redis, Gazebo, bridge, simulator) with automatic cleanup of stale processes on every run
