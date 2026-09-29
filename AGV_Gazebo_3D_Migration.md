# Porting the AGV Simulation into Gazebo (3D)

**Status:** design document, pre-implementation
**Date:** 2026-09-02
**Scope:** add a 3D Gazebo digital twin of the AGV floor, driven by the existing Python simulator
**Supersedes:** `AGV_FactoryIO_3D_Migration.docx` as the recommended 3D target. That document's system inventory (§1) is still accurate and is referenced rather than repeated.

---

## 0. Executive summary

### What is decided

- **The simulator stays authoritative.** Routing, the node-lock traffic manager, deadlock cycle resolution, the battery model, the dispatcher, the 18 `chatbot:*` command channels, Mongo persistence, and both chat agents are unchanged. Gazebo is a render + physics target, not a second brain.
- **The world is generated, not hand-built.** Gazebo worlds are SDF (XML). `shared/layouts/poc-floor.json` is machine-translated into `agv_floor.sdf`. This deletes the single largest cost in the Factory I/O plan.
- **Gazebo is additive.** The 2D SVG map at `/home/agv-map`, the KPI bar, the task scheduler and both chat surfaces keep working with Gazebo stopped. Nothing gains a hard dependency on it.
- **The bridge is a new service** (`local-iops/agents/agv-gazebo-bridge`) built on the same Redis-pub/sub-with-reconnect shape as [agv-stream-bridge/app.py](local-iops/agents/agv-stream-bridge/app.py). Both bridges subscribe independently.
- **Target: Gazebo Harmonic (LTS) + ROS 2 Jazzy on Ubuntu 24.04.** Verified: Harmonic binaries are published for Noble 24.04 as the `gz-harmonic` metapackage, supported Sep 2023 → May 2029. This dev container is already Ubuntu 24.04, so the pairing is native. Jetty is the newer stable release; Harmonic is chosen for the longer LTS window and the mature `ros_gz` bridge.

### Why this is a much better target than Factory I/O

The Factory I/O document had to spend most of its length working around four hard walls. Gazebo removes all four.

| Factory I/O wall | Gazebo |
| --- | --- |
| No AGV/AMR part exists; needed a "stand-in" (Track A conveyors vs Track B rail vehicles), and Track B was unproven | A wheeled robot is the native primitive. `diff_drive`, `velocity_control`, `mecanum_drive`, `ackermann_steering` all ship in-box (verified in the `gz-sim8` system list) |
| No layout import; scene hand-built from `poc-floor.json` as a "reference drawing" (~400 m of conveyor, 200+ parts, 3–5 days) | SDF is XML. One generator script emits the whole floor from the canonical layout, so 2D and 3D geometry cannot drift |
| Only interface is a flat Modbus tag table; "no API to set an arbitrary object's world position" | `/world/<world>/set_pose_vector` takes a `gz.msgs.Pose_V` — all five AGV poses in one call. Plus per-model `cmd_vel` |
| Windows + DirectX; cannot join the Docker network | Linux-native, runs in Compose next to Redis |

Two workstreams from that plan are now simply cancelled:

- **The five diagonal station spurs** (S28, S29, S30, H02, H03) needed realigning because Factory I/O conveyors are grid-locked. Gazebo takes arbitrary poses. **Leave `poc-floor.json` alone.**
- **The soft-PLC question** (OpenPLC / CODESYS / PLCSIM, §3.5, +2–4 days) existed only because Python-over-TCP could not hold conveyor interlocks at scan rate. There are no conveyor interlocks here. No PLC.

### Honest statement of what Gazebo does *not* give you

Read this before committing, because it is the one place where the ask and the tool are not aligned.

**Gazebo has no supported in-browser renderer.** `gzweb` is a Gazebo *Classic* project (Classic went EOL in January 2025) and there is no equivalent shipped for Harmonic. So "the 2D map in our React UI becomes 3D" is *not* something Gazebo delivers directly — it needs a delivery mechanism bolted on (§5), and every option there has a real cost.

If the actual goal is **"our floor map page renders in 3D"**, the cheapest and best-looking answer is react-three-fiber in the existing plugin, fed by the WebSocket the UI already consumes — no Gazebo at all (§5, Option 4). Roughly 3–4 days, no new infrastructure, 60 fps, no GPU on the server.

Gazebo's real value is different and worth having: **rigid-body physics, wheel contact, sensors (lidar/depth/camera), and ROS 2 lineage.** It is what makes the demo a *robotics* digital twin rather than an animated diagram, and it is the only path to "the AGV has a lidar and sees the pallet."

**Recommendation: do both, and cheaply.** One generator emits the SDF world *and* the three.js scene from `poc-floor.json`. The React page gets a real 3D tab (Option 4). Gazebo runs as the physics-grade twin, shown in its own window for the walkthrough (§5, Option 1) and optionally piped into the same three.js canvas later (§5, Option 5). §13 sequences it so the in-product 3D view lands first and Gazebo depth accrues behind it.

---

## 1. What is already established

`AGV_FactoryIO_3D_Migration.docx` §1 inventories the services, config, channels, geometry and the full 33-station table. All of it still holds. The facts that drive *this* plan, re-verified against the code:

| Fact | Source |
| --- | --- |
| Floor is 60 × 40 m, `units: meters`; 33 stations + 36 waypoints = 69 nodes, 72 edges | [poc-floor.json](shared/layouts/poc-floor.json) |
| `TICK_HZ=10`, `AGV_SPEED=1.5` m/s constant (no accel/decel), `FLEET_SIZE=5`, `LOAD_SECONDS=3` | [config.py](local-iops/agents/agv-simulator/src/agv_simulator/config.py) + compose overrides |
| Fleet start nodes: H01, H02, H03, W34, W38 | [fleet.py:21](local-iops/agents/agv-simulator/src/agv_simulator/fleet.py#L21) |
| States: `idle, moving, loading, unloading, blocked, charging, depleted` | [agv.py:33](local-iops/agents/agv-simulator/src/agv_simulator/agv.py#L33) |
| `snapshot()` publishes `x, y, heading, speed, state, trip_id, current_node, payload_count, battery_pct, origin_node, dest_node, blocked_by, blocked_at_node` at 10 Hz × 5 = 50 msg/s | [agv.py:765](local-iops/agents/agv-simulator/src/agv_simulator/agv.py#L765) |
| Node-level path is computed but **not published** | [agv.py:626](local-iops/agents/agv-simulator/src/agv_simulator/agv.py#L626) `planned_path_nodes()` |
| 14 Redis channels: `agv:telemetry`, `agv:state`, `task:*`, `trip:*`, `traffic:*` | [telemetry.py:13-24](local-iops/agents/agv-simulator/src/agv_simulator/telemetry.py#L13-L24) |
| Traffic is node-lock based: acquire `b` before traversing `a→b`, release `a` on arrival | [traffic.py](local-iops/agents/agv-simulator/src/agv_simulator/traffic.py) |
| Three copies of `poc-floor.json` exist and must stay identical | canonical `shared/layouts/`, UI `iops-agv-map-ui/src/data/`, `agv-floor-map-ui/html/` |

### 1.1 Two corrections to the Factory I/O document

**The layout is y-up, not y-down.** [FloorMap.tsx:155](local-iops/ui/packages/iops-agv-map-ui/src/components/FloorMap.tsx#L155) wraps the whole scene in `transform="translate(0 maxY) scale(1 -1)"`. So `poc-floor.json` uses mathematical convention (y increases northward) — S01 at `y=37` renders at the top of the screen. **This means no axis flip is needed for Gazebo**, whose ENU frame is also y-up. Getting this wrong produces a silently mirrored world, so it is called out in the coordinate contract (§6).

Confirming the same for heading: [AgvMarker.tsx:203](local-iops/ui/packages/iops-agv-map-ui/src/components/AgvMarker.tsx#L203) applies `rotate(agv.heading)` *inside* the flipped group, so `heading` is CCW-positive from +x — standard `atan2`, directly convertible to a Gazebo yaw.

**There are 7 station families, not 6.** [StationMarker.tsx:10-30](local-iops/ui/packages/iops-agv-map-ui/src/components/StationMarker.tsx#L10-L30) maps 19 roles onto `logistics, smt, test, rework, buffer, assembly, parking`. The Factory I/O table dropped `assembly` (conformal_coat, depanel, final_assembly, packaging). Reuse this map verbatim so 2D and 3D read identically (§7.3).

---

## 2. Target architecture

```
                        ┌──────────────────────────────┐
                        │  agv-simulator (unchanged)   │
                        │  routing · traffic · battery │
                        │  dispatcher · command bridge │
                        └──────────────┬───────────────┘
                                       │ Redis pub/sub
                 ┌─────────────────────┴─────────────────────┐
                 │                                           │
      ┌──────────▼───────────┐                  ┌────────────▼─────────────┐
      │ agv-stream-bridge    │                  │ agv-gazebo-bridge   NEW  │
      │ (unchanged, :8081)   │                  │ Redis → ROS 2 → gz       │
      └──────────┬───────────┘                  └────────────┬─────────────┘
                 │ WebSocket /ws                             │ ros_gz_bridge
      ┌──────────▼───────────┐                  ┌────────────▼─────────────┐
      │ React UI (:3001)     │                  │ Gazebo Harmonic          │
      │ ├ 2D SVG map         │                  │ agv_floor.sdf            │
      │ ├ 3D r3f canvas ★    │                  │ physics · sensors        │
      │ └ KPI · sched · chat │                  └──────────────────────────┘
      └──────────────────────┘
                        ▲                                     │
                        └──── ★ optional: Gazebo poses ───────┘
                              back into the r3f canvas (§5, Option 5)
```

Everything left of the split is today's system, untouched. Everything right of it is new and independently killable: with the Gazebo container stopped, the 2D map, the KPI bar and both chat agents are unaffected.

`network_mode` matters here — see R5 in §14. Gazebo transport uses UDP multicast discovery, so Gazebo, `ros_gz_bridge` and the bridge process should share one network namespace, with **Redis as the only cross-container hop**.

---

## 3. Who owns motion: four fidelity modes

This is the central design decision. It is the Gazebo equivalent of the Factory I/O document's Track A/B question, but unlike that one it is not a gamble — all four modes are known to work, and they differ in cost and fidelity.

| | **K — Kinematic** | **V — Velocity** | **D — DiffDrive** | **N — Nav2** |
| --- | --- | --- | --- | --- |
| AGV pose comes from | `set_pose_vector`, straight from telemetry | Twist on `cmd_vel`, closed-loop on telemetry | wheel joints + contact, pure-pursuit on the node path | Nav2 planner + controller |
| Gazebo mechanism | `user_commands` system | `velocity_control` system | `diff_drive` system | `diff_drive` + lidar + AMCL/Nav2 |
| Wheels turn | no | no | **yes** | yes |
| Parity with the 2D map | exact, frame for frame | within ~10 cm | within ~30 cm, needs a leash | diverges by design |
| Sensors possible | no | no | yes | required |
| Who decides the route | simulator | simulator | simulator | **Nav2 — second brain** |
| Effort on top of the world | ~1 day | ~2 days | ~3 days | ~2+ weeks |

**Mode N is rejected.** Nav2 replans around obstacles and runs its own costmap-based collision avoidance. Put it next to the simulator's node-lock traffic manager with deadlock cycle resolution ([traffic.py:84](local-iops/agents/agv-simulator/src/agv_simulator/traffic.py#L84)) and you have two independent congestion controllers fighting — exactly risk R4/R11 from the Factory I/O plan, and far harder to debug here because both are real planners. If autonomy is wanted as a story, scope it as a *separate single-robot side demo* on its own world, disconnected from the fleet.

**Recommended path: K as a throwaway spike → V as the production baseline → D as the fidelity upgrade.**

Each is a working deliverable, and each reuses the previous one's world file and bridge.

### 3.1 Mode K — kinematic (spike only)

`user_commands` advertises `/world/<world>/set_pose_vector` taking a `gz.msgs.Pose_V`, with commands "queued in order of reception and executed in order during PreUpdate". One request per tick carries all five AGVs.

Use it to prove the coordinate contract and the world geometry, driven by shelling out to `gz service` at ~5 Hz. It is deliberately disposable: process-per-call does not belong in a 10 Hz loop, and a teleporting robot looks worse than a rolling one. Do not build features on it.

### 3.2 Mode V — velocity (production baseline)

`velocity_control` is documented as "a linear and angular velocity controller which is directly set on a model", subscribing to `<topic>`, default `/model/<model_name>/cmd_vel`. So each AGV model carries the system, and the bridge publishes a Twist per AGV per tick.

Why this is the baseline rather than Mode K:

- **No custom plugin and no gz-transport Python bindings.** `geometry_msgs/Twist ↔ gz.msgs.Twist` is a core `ros_gz_bridge` conversion, so the bridge is a plain `rclpy` publisher.
- **Velocity is integrated by Gazebo**, so motion is smooth between the simulator's 10 Hz ticks instead of stepping.
- **Heading comes out for free** — command angular velocity and the model rotates, instead of snapping yaw each frame.

The control law is a trivial position servo, not a planner. Per AGV per tick, with target `p*` from telemetry and actual `p` from `pose_publisher`:

```
e        = p* − p                      # world-frame error, metres
v_cmd    = clamp(Kp · ‖e‖, 0, 2.0)     # m/s; AGV_SPEED is 1.5
yaw_des  = atan2(e.y, e.x)             # if ‖e‖ > 0.05, else hold
ω_cmd    = clamp(Kω · wrap(yaw_des − yaw), −1.5, 1.5)   # rad/s
```

Because it is closed-loop on the simulator's own position, it cannot drift: the simulator remains the single source of truth for *where* an AGV is, and Gazebo only decides *how it gets there* over the next 100 ms.

### 3.3 Mode D — differential drive (fidelity upgrade)

Swap `velocity_control` for `diff_drive`, add two wheel links + joints and a caster, and the AGV becomes a real rolling body: wheels spin, the chassis pitches under acceleration, and the floor friction is real.

Two things this needs that Mode V does not:

1. **A path to follow, not a point to chase.** Chasing a target 15 cm ahead makes a physical robot wobble. Pure-pursuit against the *node-level* path is the right controller — which is why §10.1 publishes `planned_path`.
2. **A leash.** Physics will diverge from the simulator (wheel slip, contact, Gazebo's real-time factor < 1). Track `‖e‖`; if it exceeds `LEASH_M` (start at 0.5 m) for more than ~1 s, correct with a single `set_pose` call and log it. Leash trips per minute is the honest fidelity metric for Mode D, and it belongs in the acceptance criteria (§15).

Only reach for Mode D once Mode V is demo-stable. It is the difference between "convincing" and "credible to a robotics audience", but it is also the first mode that can visibly misbehave.

---

## 4. World generation from `poc-floor.json`

`scripts/gen_gazebo_world.py` reads the canonical layout and emits `shared/gazebo/agv_floor.sdf`. Generated, never hand-edited — the same rule the Factory I/O plan applied to its tag map, for the same reason: a hand-maintained scene drifts from the layout and produces silent misrouting.

### 4.1 What it emits

| Element | Count | Geometry |
| --- | --- | --- |
| Ground plane | 1 | 60 × 40 m box, dark matte, centred at origin |
| Aisle lane markings | 72 | thin flat boxes (0.4 m wide, 5 mm thick) along each edge, from the endpoint coordinates — visual only, no collision |
| Waypoint dots | 36 | 0.15 m cylinders, flush with the floor |
| Station models | 30 | 2.8 × 1.7 m footprint (matching `W`/`H` in [StationMarker.tsx:7-8](local-iops/ui/packages/iops-agv-map-ui/src/components/StationMarker.tsx#L7-L8)), 1.2 m tall, coloured by family (§7.3), name in a `<visual>` text or a floating billboard |
| Charge bays | 3 | H01–H03, floor-marked pad + a lamp marker (§8.2) |
| Zone floor tint | 3 | Zone A/B/C bands from `ZONES` in [FloorMap.tsx:15-19](local-iops/ui/packages/iops-agv-map-ui/src/components/FloorMap.tsx#L15-L19) |
| AGV models | 5 | §4.3 |
| Marker sets | 5 × N | per-AGV state/payload markers (§8) |

~150 models, all box/cylinder primitives, no meshes, no textures beyond flat colours. That is a light scene — which matters a lot given R1 (no GPU).

### 4.2 World header

```xml
<sdf version="1.9">
  <world name="agv_floor">
    <physics name="10ms" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>       <!-- set_pose_vector -->
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <!-- no <sensors> system until a GPU host exists; see R1 -->
```

`real_time_factor` must be 1.0: the simulator runs on `simpy.rt.RealtimeEnvironment(factor=1.0)` ([main.py:66](local-iops/agents/agv-simulator/src/agv_simulator/main.py#L66)), so any sustained RTF below 1 makes Gazebo lag wall-clock telemetry and, in Mode D, trips the leash continuously.

### 4.3 AGV model (Mode V)

```xml
<model name="AGV01">
  <pose>-4 -10 0.2 0 0 0</pose>              <!-- H01 (26,10) → see §6 -->
  <link name="chassis">
    <inertial><mass>120</mass>
      <inertia><ixx>8</ixx><iyy>16</iyy><izz>20</izz>
               <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
    <visual name="body">
      <geometry><box><size>1.2 0.8 0.35</size></box></geometry>
      <material><ambient>0.16 0.5 0.9 1</ambient>
                <diffuse>0.16 0.5 0.9 1</diffuse></material>
    </visual>
    <collision name="body">
      <geometry><box><size>1.2 0.8 0.35</size></box></geometry>
    </collision>
  </link>

  <plugin filename="gz-sim-velocity-control-system"
          name="gz::sim::systems::VelocityControl">
    <topic>/model/AGV01/cmd_vel</topic>
  </plugin>
  <plugin filename="gz-sim-pose-publisher-system"
          name="gz::sim::systems::PosePublisher">
    <publish_link_pose>false</publish_link_pose>
    <publish_model_pose>true</publish_model_pose>
    <update_frequency>20</update_frequency>
  </plugin>
</model>
```

`pose_publisher` closes the loop for the §3.2 servo, and is the same feed Option 5 in §5 would push to the browser. Plugin *filenames* are the one thing here worth confirming against the installed build rather than trusting the convention — that is audit item A0.3.

Under Mode D this model gains `left_wheel`/`right_wheel` links, two revolute joints, a caster sphere, and `gz-sim-diff-drive-system` with `<wheel_separation>0.7</wheel_separation>` and `<wheel_radius>0.12</wheel_radius>` in place of `velocity_control`.

### 4.4 Guardrails

- **Read only the canonical copy** at `shared/layouts/poc-floor.json`.
- **Add a test that the three copies are byte-identical** (this is R7 from the Factory I/O plan, still live). Cheapest form: a pytest in `agv-simulator/tests/` hashing all three. Better: make the UI copy a build-time copy.
- **Name every model by its node or edge ID** (`S05`, `W14`, `seg_W11_W12`). The bridge keys on these names; an unnamed model is untraceable when it misbehaves.
- **Regenerate, never patch.** If the SDF is wrong, fix the generator.

---

## 5. Getting 3D into the browser

This is the hard part, and it is where the honest trade-off lives (§0). Five options, cheapest first.

### Option 1 — Gazebo GUI, side by side (Phase 2)

Run `gz sim` on the demo machine, put it next to the browser. Zero integration work, full interactivity, all Gazebo tooling available. Genuinely the right answer for a walkthrough demo, and it is what the early phases target.
*Cost: 0. Limitation: not in the product.*

### Option 2 — noVNC iframe (recommended for "in the UI, this week")

Run `gz sim` headless in a container with Xvfb + x11vnc + noVNC; add a **3D** tab to `AgvMapPage` holding `<iframe src="http://<host>:6080/vnc.html?autoconnect=1&view_only=1">`.

Guaranteed to work, no GPU strictly required (llvmpipe), and the whole Gazebo view — including its GUI chrome — appears in the product.
*Cost: ~1 day. Limitations: bandwidth-heavy; desktop chrome leaks into the page; one shared camera for all viewers; awkward on a laptop over VPN.*

### Option 3 — video stream from a camera sensor

Place a `camera` sensor at an isometric vantage and stream its output (MJPEG over HTTP, or WebRTC via GStreamer) into an `<img>`/`<video>`. Cleaner than VNC — no desktop chrome, no input surface.
*Cost: ~2 days. Hard blocker here: camera sensors need the `sensors` system and real GPU rendering. On llvmpipe this will not hold 10 fps. Parked until a GPU host exists (R1).*

### Option 4 — react-three-fiber in the plugin (recommended for the product)

A `<FloorMap3D>` component in `iops-agv-map-ui`, fed by the **existing** `ws://<host>:8081/ws` stream that [useAgvWebSocket.ts:8](local-iops/ui/packages/iops-agv-map-ui/src/hooks/useAgvWebSocket.ts#L8) already consumes. The scene is generated from the same `poc-floor.json` the SVG map imports, by the same generator that emits the SDF.

This is the best in-product 3D view by a wide margin: 60 fps on the client's GPU, no server rendering, no new container, camera controls, click-to-select an AGV wired straight into the existing `useFleetState` selection, and it degrades exactly like the 2D map does. It needs no Gazebo — the simulator already publishes `x`, `y`, `heading` at 10 Hz.
*Cost: ~3–4 days. Limitation: it is a visualiser, not a physics engine. No sensors, no contact, no wheel dynamics.*

### Option 5 — Gazebo physics rendered by Option 4 (the end state)

Once both exist, repoint the r3f canvas from `agv:telemetry` to Gazebo's own `pose_publisher` output (bridged to ROS 2, fanned out over WebSocket alongside the existing channels). The browser then shows **genuine Gazebo physics** at 60 fps with no VNC and no video encoding: Gazebo owns motion, the simulator owns decisions, three.js renders.
*Cost: ~1 day on top of Option 4 + Mode D. This is where the architecture wants to end up.*

### Recommendation

**Option 1 for the early phases, Option 4 for the product, Option 5 as the end state.** Option 2 is the fallback if an in-product view of *Gazebo specifically* is needed before Option 4 lands. Option 3 stays parked until there is a GPU.

---

## 6. Coordinate and unit contract

Every bug in this class of work is a frame bug, so this is specified once and asserted in tests.

| Quantity | Simulator | Gazebo | Conversion |
| --- | --- | --- | --- |
| x | 0 … 60 m, +x east | ENU metres | `gz_x = sim_x − 30` |
| y | 0 … 40 m, **+y north** (§1.1) | ENU metres | `gz_y = sim_y − 20` — **no flip** |
| z | absent (2D) | metres up | `gz_z = 0.2` (chassis half-height + clearance) |
| heading | degrees, CCW from +x, −180…180 (`atan2`) | yaw radians | `yaw = radians(heading)`, then wrap to (−π, π] |
| speed | m/s, 0 or `AGV_SPEED` (1.5) | m/s | identity |
| time | wall clock, `RealtimeEnvironment(factor=1.0)` | sim time, RTF must be 1.0 | identity while RTF = 1 |

Origin is re-centred so the floor straddles `(0,0)`; Gazebo's grid, default camera and lighting all assume a world near the origin. The offset is a single constant pair in the generator and the bridge — derive both from `layout["bounds"]`, never hardcode `30`/`20`.

**Assertions worth writing** (they catch the mirrored-world and radians/degrees bugs immediately):

- `S01 (3, 37) → (−27, +17)` — north-west corner, top-left on screen and in the world.
- `S28 (47, 3) → (+17, −17)` — south-east.
- `heading = 90` (northbound) → `yaw = π/2`, model nose toward +y.

---

## 7. Visual mapping

### 7.1 Fleet identity

Five AGVs, `AGV01`…`AGV05`, coloured distinctly and consistently with the 2D chips. Model names are the join key for `cmd_vel` topics, pose feedback and marker lookup, so they must equal `agv_id` exactly.

### 7.2 Station geometry

Footprint from `StationMarker` (2.8 × 1.7 m) so a station occupies the same floor area in both views. Height 1.2 m, except `parking` bays which are floor-flush pads so a parked AGV is visible on them.

### 7.3 Role → family → colour

Import the mapping from [StationMarker.tsx:10-30](local-iops/ui/packages/iops-agv-map-ui/src/components/StationMarker.tsx#L10-L30) — all 19 roles, 7 families (`logistics, smt, test, rework, buffer, assembly, parking`). The generator should read it from a shared JSON rather than duplicating the table in Python; a divergence here means the 3D view colour-codes the plant differently from the 2D one, which is worse than no colour at all.

---

## 8. Encoding AGV state in 3D

The simulator has seven states, plus payload count, battery, and blocked-by. Runtime *material* changes are awkward in Gazebo (there is no first-class "set this visual's colour" service to rely on). The robust technique is **marker swapping**: pre-spawn every marker in the generated world and move the relevant one into place, parking the rest at `z = −5` under the floor. It is one `set_pose` per change, uses only APIs already needed, and is entirely deterministic.

### 8.1 State beacon

One small cone per AGV per state colour, spawned at `z = −5`. On an `agv:state` event, lift the new state's cone to `(x, y, 0.6)` above that AGV and sink the previous one. Because `agv:state` fires only on transition ([agv.py:111-125](local-iops/agents/agv-simulator/src/agv_simulator/agv.py#L111-L125)), this is a handful of calls per minute, not a per-tick cost.

| State | Beacon |
| --- | --- |
| `idle` | none (all sunk) |
| `moving` | none — motion is self-evident |
| `loading` / `unloading` | amber, pulsed by the station cycle (§8.3) |
| `blocked` | red, plus a floor disc at `blocked_at_node` |
| `charging` | green on the bay lamp (§8.2) |
| `depleted` | red, and the AGV stops — matches the 2D `agv-depleted-alert` |

### 8.2 Charge bay lamps

Three lamp models at H01–H03, green marker raised while an AGV reports `state == "charging"` on that node. Driven off `agv:telemetry.current_node` + `state`, dirty-checked so it writes once per transition.

### 8.3 Payload and station cycles

- **Payload:** a crate model per AGV, parked below the floor when `payload_count == 0` and lifted onto the deck when it is non-zero. Scale it with `payload_count` against `AGV_CAPACITY` (50) if a fuller-looking load is wanted.
- **Station cycle:** on `trip:stop_done`, raise the station's amber marker for `LOAD_SECONDS` (3 s) then sink it. **This needs an explicit timer** — the Factory I/O skeleton's known gap 8.3 was exactly this (a pulse set true and never cleared). Here it is a `asyncio.call_later`, and the de-pulse must survive the bridge reconnecting mid-cycle.

### 8.4 Traffic and deadlock

`traffic:blocked` / `traffic:resumed` carry `agv_id`, `node` and `blocked_by`. Raise a red disc at the contended node while blocked. This is the one part of the simulator's intelligence that has a genuine 3D analogue, and it makes the x=22 / x=44 corridor bottleneck (Factory I/O §1.6a — the entire vertical capacity of the plant) visible as the thing the traffic manager is actually solving. Worth building properly.

---

## 9. The bridge service

New directory `local-iops/agents/agv-gazebo-bridge/`, structured like [agv-stream-bridge/app.py](local-iops/agents/agv-stream-bridge/app.py): `psubscribe` with a reconnect loop, no request/response protocol, fail-soft throughout.

### 9.1 Design decisions

1. **Coalesce, then flush.** `agv:telemetry` arrives at 50 msg/s. Keep a latest-value dict per AGV and flush on a fixed timer at `FLUSH_HZ=10`. Never one write per message.
2. **Dirty-check every marker.** Poses for state beacons, lamps, crates and discs only get written when the value changes. With the fleet parked, bridge output drops to zero.
3. **Fail soft, always.** If Gazebo is down, log with backoff and keep draining Redis. The consumer must never block and the process must never crash — the 2D map and both chat agents have to be unaffected by a dead Gazebo container. This is the same contract `agv-stream-bridge` already honours.
4. **One-way for decisions.** The bridge reads Gazebo poses only to close the §3.2 servo loop and to measure leash error. It never feeds Gazebo state back into the simulator. The simulator does not learn that Gazebo exists.
5. **ROS 2 is the boundary.** `rclpy` publisher → `ros_gz_bridge` → gz transport. This avoids both incomplete gz-transport Python bindings and process-per-call CLI shelling. `set_pose` for markers and leash corrections is rare enough that `gz service` CLI is acceptable there; if it turns out not to be, a ~50-line `AgvPoseSink` system plugin subscribing to one `gz.msgs.Pose_V` topic replaces it.
6. **Mode behind an env var.** `MODE=K|V|D` selects the renderer class, exactly as the Factory I/O connector planned `TRACK=A|B`.

### 9.2 Skeleton

```python
"""Redis → Gazebo bridge.

Subscribes to agv:* / trip:* / traffic:* and drives the Gazebo scene.
The simulator remains authoritative for every decision; this process
only renders fleet state into poses and velocity commands.
"""
from __future__ import annotations

import asyncio, json, logging, math, os

import rclpy
import redis.asyncio as redis
from geometry_msgs.msg import Twist
from rclpy.node import Node

log = logging.getLogger("agv-gazebo-bridge")

REDIS_HOST = os.getenv("REDIS_HOST", "redis")
WORLD      = os.getenv("GZ_WORLD", "agv_floor")
MODE       = os.getenv("MODE", "V")            # K | V | D
FLUSH_HZ   = float(os.getenv("FLUSH_HZ", "10"))
LEASH_M    = float(os.getenv("LEASH_M", "0.5"))
PATTERNS   = ["agv:*", "trip:*", "traffic:*"]

# Derived from layout["bounds"] at startup — never hardcoded.
OFF_X, OFF_Y, AGV_Z = 30.0, 20.0, 0.2


def to_gz(x: float, y: float, heading_deg: float) -> tuple[float, float, float]:
    """Sim floor coords → Gazebo ENU. No y flip: the layout is y-up (§6)."""
    yaw = math.radians(heading_deg)
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))      # wrap to (-pi, pi]
    return x - OFF_X, y - OFF_Y, yaw


class VelocityRenderer(Node):
    """Mode V: position servo per AGV, closed on Gazebo pose feedback."""

    KP, KW = 1.6, 2.5
    V_MAX, W_MAX = 2.0, 1.5

    def __init__(self) -> None:
        super().__init__("agv_gazebo_bridge")
        self.target: dict[str, tuple[float, float, float]] = {}   # from Redis
        self.actual: dict[str, tuple[float, float, float]] = {}   # from Gazebo
        self.cmd = {
            aid: self.create_publisher(Twist, f"/model/{aid}/cmd_vel", 10)
            for aid in (f"AGV{i:02d}" for i in range(1, 6))
        }

    def on_telemetry(self, d: dict) -> None:
        aid = d.get("agv_id")
        if aid in self.cmd:
            self.target[aid] = to_gz(d.get("x", 0.0), d.get("y", 0.0),
                                     d.get("heading", 0.0))

    def flush(self) -> None:
        for aid, pub in self.cmd.items():
            tgt, act = self.target.get(aid), self.actual.get(aid)
            if tgt is None or act is None:
                continue
            ex, ey = tgt[0] - act[0], tgt[1] - act[1]
            dist = math.hypot(ex, ey)
            t = Twist()
            if dist > 0.05:
                t.linear.x = min(self.KP * dist, self.V_MAX)
                dyaw = math.atan2(math.sin(math.atan2(ey, ex) - act[2]),
                                  math.cos(math.atan2(ey, ex) - act[2]))
                t.angular.z = max(-self.W_MAX, min(self.W_MAX, self.KW * dyaw))
            pub.publish(t)
            if dist > LEASH_M:
                log.warning("%s leash %.2fm — snapping", aid, dist)
                # TODO: gz service set_pose correction; see §9.3


async def subscribe_loop(renderer: VelocityRenderer) -> None:
    while True:
        try:
            client = redis.Redis(
                host=REDIS_HOST,
                port=int(os.getenv("REDIS_PORT", "6379")),
                password=os.getenv("REDIS_PASSWORD") or None,
                decode_responses=True,
            )
            pubsub = client.pubsub()
            await pubsub.psubscribe(*PATTERNS)
            log.info("subscribed to %s", PATTERNS)
            async for msg in pubsub.listen():
                if msg.get("type") not in ("pmessage", "message"):
                    continue
                raw = msg.get("data")
                try:
                    d = json.loads(raw) if isinstance(raw, str) else raw
                except Exception:
                    continue
                if not isinstance(d, dict):
                    continue
                if msg.get("channel") == "agv:telemetry":
                    renderer.on_telemetry(d)
                # agv:state / trip:stop_done / traffic:* → markers (§8)
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            log.warning("subscribe loop crashed (%s); retrying in 2s", exc)
            await asyncio.sleep(2)


async def flush_loop(renderer: VelocityRenderer) -> None:
    period = 1.0 / FLUSH_HZ
    while True:
        renderer.flush()
        await asyncio.sleep(period)


async def main() -> None:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    rclpy.init()
    renderer = VelocityRenderer()
    await asyncio.gather(subscribe_loop(renderer), flush_loop(renderer))


if __name__ == "__main__":
    asyncio.run(main())
```

### 9.3 Known gaps in the skeleton

Deliberately left for implementation, listed so they are not forgotten:

1. **Pose feedback is not wired.** `self.actual` is never populated. Needs a subscription to the bridged `pose_publisher` output, and until it arrives the servo emits nothing (which is a safe failure, but a silent one — log it).
2. **`rclpy` spin is missing.** `rclpy.init()` alone does not process callbacks. Either run an executor on a thread or use `rclpy.spin_once` inside `flush_loop`. Mixing `asyncio` and `rclpy` executors is the fiddliest part of this file; do it once, deliberately.
3. **Leash correction is a TODO.** Needs the `set_pose` call, plus rate-limiting so a persistently-lagging AGV does not snap every tick.
4. **Markers are entirely unimplemented.** All of §8 — beacons, lamps, crates, blocked discs, and the station-cycle de-pulse timer.
5. **Mode K and Mode D renderers do not exist.** `VelocityRenderer` is Mode V only; the others sit behind `MODE` as separate classes.
6. **Startup pose sync.** Fleet start nodes are H01, H02, H03, W34, W38 ([fleet.py:21](local-iops/agents/agv-simulator/src/agv_simulator/fleet.py#L21)) and the generator bakes those into the SDF. If the bridge attaches to an already-running simulator the AGVs are elsewhere — do one `set_pose_vector` on connect before starting the servo, or the fleet drives across the floor from its spawn points on every reconnect.
7. **`depleted` must actually stop the robot.** A depleted AGV freezes in place; the servo must publish a zero Twist rather than simply stopping publishing, or the model coasts on its last commanded velocity.

---

## 10. Simulator-side changes

Two changes. One is required only for Mode D.

### 10.1 Publish the node-level planned path (Mode D)

`planned_path_nodes()` exists at [agv.py:626](local-iops/agents/agv-simulator/src/agv_simulator/agv.py#L626); `snapshot()` at [agv.py:765](local-iops/agents/agv-simulator/src/agv_simulator/agv.py#L765) omits it. Add:

```python
# agv.py, in snapshot()
"planned_path": self.planned_path_nodes(),
```

It must be *published*, not recomputed in the bridge. `_plan_path()` ([agv.py:656](local-iops/agents/agv-simulator/src/agv_simulator/agv.py#L656)) applies both a hard-avoid set and peer reservations from `reservation_provider` ([main.py:243](local-iops/agents/agv-simulator/src/agv_simulator/main.py#L243)); a fresh Dijkstra in the bridge will not reproduce that, and pure-pursuit against a path the AGV is not taking makes the robot cut across a corridor the simulator is deliberately avoiding.

Cost: `routing.shortest_path` is an O(1) lookup against the precomputed all-pairs table ([routing.py](local-iops/agents/agv-simulator/src/agv_simulator/routing.py)), so this is cheap — but it lands in a 50 Hz path, so measure. Fallback if it shows up: publish on `trip:requested` and on reroute only.

This is item 7.1 of the Factory I/O plan, unchanged, and it is worth doing regardless: the 2D map's `TripOverlay` currently draws only a straight line to `dest_node`, so publishing the real path improves that too.

### 10.2 Nothing else

No changes to the traffic manager, dispatcher, battery model, command bridge or either chat agent. If the Gazebo work starts wanting simulator changes beyond §10.1, that is a signal that logic is leaking into the render layer — stop and re-read §3.

---

## 11. Compose changes

```yaml
  # Gazebo Harmonic — 3D digital twin of the AGV floor
  gazebo:
    build: ./agents/agv-gazebo          # ubuntu:24.04 + gz-harmonic + ros-jazzy
    container_name: iops-gazebo
    restart: unless-stopped
    environment:
      GZ_WORLD: agv_floor
      GZ_PARTITION: agv
      LIBGL_ALWAYS_SOFTWARE: "1"        # no GPU in this environment; see R1
      DISPLAY: ":99"
    volumes:
      - ../shared:/shared:ro            # agv_floor.sdf + poc-floor.json
    devices:
      - /dev/dri:/dev/dri               # present here (card0); harmless if not
    ports:
      - "6080:6080"                     # noVNC, only if §5 Option 2 is built
    command: >
      gz sim -s -r -v3 /shared/gazebo/agv_floor.sdf

  # Redis pub/sub → ROS 2 → Gazebo
  agv-gazebo-bridge:
    build: ./agents/agv-gazebo-bridge
    container_name: iops-agv-gazebo-bridge
    restart: unless-stopped
    network_mode: "service:gazebo"      # share the gz transport namespace (R5)
    environment:
      REDIS_HOST: redis
      REDIS_PORT: 6379
      REDIS_PASSWORD: ${REDIS_PASSWORD}
      GZ_WORLD: agv_floor
      GZ_PARTITION: agv
      MODE: V                           # K | V | D
      FLUSH_HZ: "10"
      LEASH_M: "0.5"
      LAYOUT_PATH: /shared/layouts/poc-floor.json
    volumes:
      - ../shared:/shared:ro
      - ./logs/agv-gazebo-bridge:/app/logs
    depends_on:
      redis:
        condition: service_healthy
```

`gz sim -s` runs server-only (no rendering) — correct for every phase before a GUI or web view is needed, and the only sane mode without a GPU. Note `network_mode: "service:gazebo"` means the bridge cannot also be on the default network by name, so `REDIS_HOST` resolution needs checking; if it breaks, put both processes in one container instead. That is A0.5.

Nothing else in compose changes. Neither service appears in any `depends_on`, so the existing stack starts and runs identically with both stopped. If they should show up in the ops tooling, extend the service list in `agv-smart-factory-assistant-tool-server`'s health check.

---

## 12. Phase 0 — environment audit

Half a day. Much shorter than the Factory I/O audit because nothing here is a capability gamble — these are environment measurements, and two of them can change the delivery plan.

**A0.1 — Rendering capability (the one that matters).** `/dev/dri/card0` exists in this container; `nvidia-smi` is absent. So: install `gz-harmonic`, run the generated world headless (`gz sim -s`), and record the sustained real-time factor with 5 AGVs moving. Then try the GUI under llvmpipe and record fps.
*Pass:* RTF ≥ 0.95 headless. *Consequence if the GUI is unusable:* §5 Options 2 and 3 are both out, Option 4 becomes the only in-product path, and Gazebo is a headless physics engine until a GPU host is found.

**A0.2 — Where does the demo run?** Is the demo machine this container, a developer laptop, or a GPU host? This decides §5 outright and should be answered before Phase 5. (The Factory I/O plan's R5 asked the same question and it was never closed.)

**A0.3 — Plugin filenames.** Confirm `gz-sim-velocity-control-system`, `gz-sim-diff-drive-system`, `gz-sim-pose-publisher-system`, `gz-sim-user-commands-system` load in the installed build. All four systems are confirmed present in the `gz-sim8` source tree; this checks the shared-library names only. Five minutes, saves an afternoon.

**A0.4 — `ros_gz_bridge` conversions.** Confirm `geometry_msgs/Twist ↔ gz.msgs.Twist` (needed, Mode V) and check whether `geometry_msgs/PoseArray ↔ gz.msgs.Pose_V` exists (would let Mode K and the marker writes avoid CLI shelling entirely).

**A0.5 — Container transport.** Verify a `rclpy` publisher in the bridge container reaches `gz sim` in the Gazebo container, and that Redis is still resolvable under `network_mode: "service:gazebo"`.
*Consequence if either fails:* collapse both into one container.

**A0.6 — `set_pose_vector` under load.** Write 5 poses at 10 Hz for a minute and confirm the queue-and-apply-in-`PreUpdate` behaviour keeps up without growing latency. Only gates Mode K, but it is also the leash mechanism for Mode D.

**Deliverable:** one page answering — RTF headless and GUI fps? Where does the demo run? Which §5 option? One container or two?

---

## 13. Phase plan

| Phase | Work | Depends on | Estimate |
| --- | --- | --- | --- |
| 0 | Environment audit (§12). Findings memo. | — | 0.5 d |
| 1 | `scripts/gen_gazebo_world.py` → `agv_floor.sdf`: floor, 72 lane markings, 30 stations, 3 bays, zone tints, 5 AGVs, marker sets. Coordinate assertions (§6) as tests. Byte-identical check on the three `poc-floor.json` copies. | 0 | 1.5–2 d |
| 2 | **Mode K spike.** `gz service set_pose_vector` at 5 Hz from telemetry. Fleet visibly tracks the 2D map. Throwaway. | 1 | 1 d |
| 3 | **Mode V production bridge.** `agv-gazebo-bridge` + `ros_gz_bridge`, pose feedback, position servo, both compose services, fail-soft verified with Gazebo stopped. | 2 | 2 d |
| 4 | **State fidelity (§8).** Beacons, charge lamps, payload crates, blocked discs, station-cycle pulse with de-pulse timer. Startup pose sync. Closes gaps 9.3.1–9.3.4, 9.3.6, 9.3.7. | 3 | 1.5–2 d |
| 5 | **`<FloorMap3D>` r3f tab** in `iops-agv-map-ui` (§5 Option 4), generated from the same layout, on the existing WebSocket. **This is the in-product 3D view.** | 1 | 3–4 d |
| 6 | **Mode D.** Wheels + `diff_drive`, publish `planned_path` (§10.1), pure-pursuit, leash + metric. | 4 | 2–3 d |
| 7 | **Option 5:** repoint the r3f canvas at Gazebo pose output — Gazebo physics in the browser. | 5, 6 | 1 d |
| 8 | Demo hardening: cold start via the Smart Factory Assistant, chat-driven dispatch visible in 3D, 10-minute sustained run, leash-trip and RTF numbers recorded. | 7 | 1 d |

**Total 13.5–17.5 days** for the full arc — comparable to the Factory I/O estimate (9–17 d) but delivering strictly more: real physics, an in-product 3D view, and no Windows dependency.

Useful earlier checkpoints:

- **End of Phase 2 (~3 days):** a recognisable 3D floor with the fleet moving on it. Enough to show and to decide whether to continue.
- **End of Phase 3 (~5 days):** production-shaped, smooth motion, safe to leave running.
- **End of Phase 5 (~9 days):** 3D inside the product. **If the deadline is tight, Phases 1 → 5 alone (≈6 days) deliver the in-product 3D view and skip Gazebo entirely** — see §0.

Phases 5 and 3 are independent (5 needs only the generator from Phase 1), so they can run in parallel with two people.

---

## 14. Risks

| # | Risk | Impact | Mitigation |
| --- | --- | --- | --- |
| R1 | **No GPU.** `nvidia-smi` absent; llvmpipe software GL only. | Gazebo GUI may be unusable; §5 Options 2 and 3 blocked; camera/lidar sensors impractical | Measured in A0.1. Run `gz sim -s` headless — physics does not need rendering. Keep the world to flat-colour primitives (§4.1). §5 Option 4 needs no server GPU at all. Sensors wait for a GPU host. |
| R2 | **Real-time factor drops below 1.** | Gazebo lags wall-clock telemetry; Mode D trips the leash continuously; motion looks laggy next to the 2D map | RTF is an acceptance metric (§15), not an afterthought. `max_step_size` 10 ms, no shadows, no meshes, no sensors. If RTF stays low, Mode V's servo absorbs it gracefully where Mode D's pure-pursuit does not — stay on V. |
| R3 | **Two brains** if Nav2 or Gazebo collision response starts making routing decisions. | Simulator stops being authoritative; deadlocks nothing can debug | Mode N rejected outright (§3). AGV collision geometry exists for contact realism, never for avoidance. Reviewers reject any bridge code that alters a route. |
| R4 | **Physics drift in Mode D.** | 3D and 2D disagree about where an AGV is | Leash + `set_pose` snap, with trips-per-minute logged and reported (§15). Mode V is always available as the fallback. |
| R5 | **gz transport across containers.** Multicast discovery is finicky in Docker. | Bridge cannot see Gazebo; silent no-op | A0.5. Shared network namespace, or one container. Redis stays the only cross-container hop. |
| R6 | **`rclpy` + `asyncio` integration** (gap 9.3.2). | Callbacks never fire; the servo silently emits nothing | Settle the executor pattern in Phase 3 before adding features. Log loudly when `self.actual` is empty rather than publishing nothing. |
| R7 | **The three `poc-floor.json` copies desync.** | 3D geometry disagrees with routing; silent misrouting | Byte-identical test in Phase 1 (carried over from the Factory I/O plan, still unfixed). Generator reads only the canonical copy. |
| R8 | **`planned_path` at 50 Hz** adds telemetry load (§10.1). | Telemetry lag | O(1) lookups against the precomputed table, but measure. Fallback: publish on `trip:requested` + reroute only. |
| R9 | **Harmonic vs Jetty.** Harmonic is "an older but still supported version"; Jetty is current stable. | Building on a release already one generation back | Harmonic is LTS to May 2029 and pairs with ROS 2 Jazzy on Ubuntu 24.04. Revisit only if a Jetty-only feature is needed. |
| R10 | **Scope creep into a robotics project.** Lidar, SLAM, fleet-manager standards, Nav2 — each is individually reasonable and collectively a different project. | The 3D view never ships | §3 fixes the boundary: Gazebo renders and simulates physics, the Python simulator decides. Anything past that is a separate proposal. |

---

## 15. Acceptance criteria

**Phase 1 — world generates**
- `agv_floor.sdf` regenerates deterministically from `shared/layouts/poc-floor.json`; no hand edits in the file.
- Coordinate assertions pass: `S01 → (−27, +17)`, `S28 → (+17, −17)`, `heading 90° → yaw π/2`. **The world is not mirrored.**
- All 33 stations and 36 waypoints present, within 0.05 m of the layout; all 72 edges have a lane marking.
- Every model named by its node or edge ID.
- The three `poc-floor.json` copies are byte-identical (enforced by test).

**Phase 3 — Mode V live**
- `docker compose up -d` brings up `gazebo` and `agv-gazebo-bridge` healthy.
- 5 AGVs track the 2D map within **0.15 m** during a sustained run.
- Headless RTF ≥ 0.95 over 10 minutes.
- **Kill the Gazebo container: the 2D map, KPI bar, task scheduler and both chat agents are unaffected.** Restart it: the fleet re-syncs without driving across the floor (gap 9.3.6).
- A `depleted` AGV comes to a stop and stays stopped (gap 9.3.7).

**Phase 4 — state is legible**
- Each of the seven states is distinguishable in 3D without reading a label.
- Charge lamps track `charging` at H01–H03 and clear on departure.
- Payload crates appear and disappear with `payload_count`.
- Station cycles pulse for `LOAD_SECONDS` on `trip:stop_done` **and clear** — including if the bridge reconnects mid-cycle.
- `blocked` shows a disc at the contended node; corridor contention at x=22/x=44 is visibly the thing the traffic manager is resolving.

**Phase 5 — 3D in the product**
- A **3D** tab on `/home/agv-map` renders the floor from the same layout the SVG map imports.
- 5 AGVs animate smoothly on the existing `ws://<host>:8081/ws` feed; no new backend service.
- Camera controls work; clicking an AGV drives the same selection state as the 2D map.
- With the WebSocket down it degrades exactly as the 2D map does.

**Phase 6 — Mode D**
- Wheels rotate consistently with travel direction; no visible slip at constant speed.
- Leash trips **< 1 per minute** on a 10-minute run, each one logged.
- Pure-pursuit follows the published `planned_path`, including after a reroute — the AGV does not cut through a corridor the simulator is avoiding.

**Phase 8 — demo ready**
- Cold start per the operator guide leaves 3D correct with no manual intervention.
- A chat-created task via the AGV Map drawer is visible end to end in 3D.
- 10-minute unattended run: no bridge reconnects, no leash storm, RTF ≥ 0.95, no growing memory.

---

## 16. Open questions

1. **Is an in-product 3D view (React UI) a hard requirement, or is a separate Gazebo window acceptable for the demo?** This decides §5 and changes the phase order. *(A0.2)*
2. **Where does the demo actually run** — this container, a laptop, or a GPU host? Gates Gazebo GUI, noVNC and any sensor work. *(A0.1, A0.2)*
3. **Is physics fidelity (Mode D: rolling wheels, contact) actually wanted, or is Mode V's smooth exact tracking enough?** Mode D is +2–3 days and introduces the only component that can visibly misbehave.
4. **Are sensors (lidar / depth / camera) in scope at all?** If yes, a GPU host stops being optional and this becomes a materially larger project.
5. **Is ROS 2 lineage part of the pitch,** or is Gazebo purely a visual upgrade? If the former, `ros_gz_bridge` and standard ROS topic naming are worth doing carefully; if the latter, §5 Option 4 alone may be the whole answer.
6. **Headless RTF with 5 AGVs and ~150 models on the target machine?** *(A0.1)* — the number the whole plan leans on.
7. **Does `ros_gz_bridge` convert `PoseArray ↔ gz.msgs.Pose_V`** in the installed version? *(A0.4)* — decides whether marker writes need CLI shelling.
8. **Should the UI's `poc-floor.json` become a build-time copy** of the canonical file, closing R7 permanently rather than testing for it?
