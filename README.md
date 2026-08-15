# Multi-AMR Warehouse Fleet

A ROS 2 navigation stack for a two-robot heterogeneous AMR fleet in a
multi-level warehouse: cooperative SLAM with selective map iteration and map
fusion, a ramp-aware global planner, space-time conflict detection with
priority-based yielding, a jerk-limited motion smoother, a sensor-driven safety
override, and a BSP-style sensor validation gateway.

**Target platform:** ROS 2 Humble · Ignition Gazebo Fortress · Ubuntu 22.04

---

## Table of contents

- [Quick start](#quick-start)
- [Running the assignment](#running-the-assignment)
- [What to look at first](#what-to-look-at-first)
- [Architecture](#architecture)
- [The command chain](#the-command-chain)
- [The sensor path](#the-sensor-path)
- [The warehouse](#the-warehouse)
- [Scaling the fleet](#scaling-the-fleet)
- [Packages](#packages)
- [Testing](#testing)
- [Configuration reference](#configuration-reference)
- [Goal completion](#goal-completion)
- [Troubleshooting](#troubleshooting)

---

## Quick start

### 1. Dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-sim \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-smac-planner \
  ros-humble-slam-toolbox ros-humble-robot-localization \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
  python3-colcon-common-extensions python3-rosdep
```

Verify nothing is missing:

```bash
cd ~/delhivery_assgn/ros2_ws
rosdep check --from-paths src --ignore-src   # All system dependencies have been satisfied
```

### 2. Build

```bash
cd ~/delhivery_assgn/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

Plain `colcon build`, **not** `--symlink-install`. The latter installs
`amr_core` as an egg-link that resolves only when `build/` happens to be on
`PYTHONPATH`; a shell without it fails at launch with `No module named
'amr_core'`.

### 3. Run — two commands

**Terminal 1** brings up everything and sends nothing:

```bash
ros2 run amr_bringup stop_stack.py      # clear anything left from a previous run
ros2 launch amr_bringup fleet.launch.py
```

World, both robots, dynamic obstacles, sensor validation, SLAM, map fusion,
Nav2, traffic control, safety override and RViz. Takes about two minutes.

Wait for this before sending anything:

```
[fleet] amr1 ready
[fleet] amr2 ready
[fleet] READY - all 2 robots accept goals.
```

That banner is a **check, not a timer** — and it checks the state that
actually matters. `fleet_ready.py` polls each robot's
`bt_navigator/get_state` and waits for `PRIMARY_STATE_ACTIVE`, not merely for
the `navigate_to_pose` server to exist. bt_navigator creates that server in
`on_configure`, so it is discoverable while the node is still CONFIGURING and
rejecting every goal. AMR-2 is launched `ROBOT_STAGGER_SEC` behind AMR-1 and
was therefore reliably the robot that had not activated when the banner
printed — measured 3.5 s early, which is exactly long enough for a goal sent
on the banner to come back rejected. If you get `NOT READY - bt_navigator not
ACTIVE for: …`, that is the genuine intermittent bringup stall; relaunch.

**Terminal 2** gives goals, as often as you like, without restarting anything:

```bash
ros2 run amr_bringup send_goal.py amr1=packing_bay_4 amr2=rack_aisle
ros2 run amr_bringup send_goal.py amr1=heavy_storage amr2=ramp_side
ros2 run amr_bringup send_goal.py amr1=-13,-1           # raw x,y (or x,y,yaw)
ros2 run amr_bringup send_goal.py --list                # what is available
```

The first line is the default pairing, and it matches the spawn lanes shipped
in `fleet.yaml` — see **Spawn lanes** below before inventing your own pairing.

Goals are dispatched to every named robot first and awaited afterwards, so the
robots drive concurrently. Each reports `REACHED`, `ABORTED` or `TIMEOUT`. A
rejected goal or a missing action server is retried rather than treated as
fatal. The default per-goal budget is **600 s**, not 300: AMR-1 runs a
0.12 m / 0.12 rad `StoppedGoalChecker`, so it creeps and settles at the end
rather than snapping to success the moment it clips tolerance, and on a long
route that settle outlasted the old budget while the robot was already parked
inside tolerance. Override with `--timeout=N`.

**Stop with Ctrl+C in Terminal 1.** `ros2 launch` owns the process group —
closing the window instead leaves orphans, and a second simulator makes every
object flicker.

---

## Running the assignment

`fleet.launch.py` + `send_goal.py` above is the normal way to drive the
system. The per-task launches below exist so each assignment task can be run
and judged on its own. Every terminal needs `source install/setup.bash`, and
every launch should be preceded by `ros2 run amr_bringup stop_stack.py`.

### Task 1 — Simulation environment

```bash
ros2 launch amr_bringup warehouse_fleet.launch.py
```

Flags: `headless:=true` (server only), `obstacles:=false` (park the pedestrians).

### Task 2 — Cooperative SLAM, map fusion, ramp-aware navigation

```bash
ros2 launch amr_bringup task2.launch.py rviz:=true goals:=true perfect_localization:=true
```

### Task 3 — Motion smoothing, conflict handling, safety override

```bash
ros2 launch amr_bringup task3.launch.py rviz:=true goals:=true perfect_localization:=true
```

### Task 4 — Sensor validation and fleet scalability

```bash
python3 demos/scenario_f_sensor_bsp_and_scaling.py --offline   # 10-robot scaling, no stack
python3 demos/scenario_f_sensor_bsp_and_scaling.py             # + data path, fault injection
ros2 topic echo /amr1/sensor_health                            # live diagnostics
```

Ten robots, each namespaced — starts no simulator, so run it on its own:

```bash
ros2 run amr_bringup stop_stack.py
ros2 launch amr_bringup fleet_10_demo.launch.py
ros2 node list        # /amr1/sensor_bsp_node … /amr10/sensor_bsp_node
```

### Task 5 — Build, dependencies, code quality

```bash
cd ~/delhivery_assgn/ros2_ws
colcon test && colcon test-result             # 92 tests: unit + every ament linter
python3 -m pytest src/amr_core/test/ -q       # 72 unit tests on their own
rosdep check --from-paths src --ignore-src
```

### About `perfect_localization`

A **simulation aid**, off by default, and worth understanding before you
demonstrate anything. It anchors TF to Gazebo ground truth so navigation plans
for a robot that is where it believes it is.

* **With it** — both robots complete their goals, no recovery spinning.
* **Without it** — AMR-2 works on its own merits (~0.3 m error); AMR-1 drifts
  ~3.5 m, its believed pose lands inside a mapped rack, and Nav2 falls back to
  spin recoveries.

SLAM still runs and still builds the fused map either way, so the Task 2
mapping deliverable is unaffected. See
[Goal completion](#goal-completion).

---

## What to look at first

If you want to judge the engineering rather than the feature list, these are
the files worth opening. The code itself is deliberately comment-free — the
reasoning behind each decision lives in [The command chain](#the-command-chain)
and [The sensor path](#the-sensor-path) below, and in
[REFACTORING.md](REFACTORING.md), so the two are read together:

| File | Why |
|---|---|
| [`amr_core/motion_smoothing.py`](ros2_ws/src/amr_core/amr_core/motion_smoothing.py) | The jerk-limited approach cap — accelerating at full authority up to the target guarantees overshoot. |
| [`amr_core/safety.py`](ros2_ws/src/amr_core/amr_core/safety.py) | Why the self-return floor must use the inscribed radius, and why `d_min` has to clear the robot's own body before any clearance remains. |
| [`amr_core/sensor_bsp.py`](ros2_ws/src/amr_core/amr_core/sensor_bsp.py) | Severity versus validation state — the action and the reason are different questions — and why frame checking is ownership, not equality. |
| [`amr_core/conflict.py`](ros2_ws/src/amr_core/amr_core/conflict.py) | Space-time conflict detection, and why a purely spatial test fires on routes traversed minutes apart. |
| [`amr_navigation/src/ramp_cost_layer.cpp`](ros2_ws/src/amr_navigation/src/ramp_cost_layer.cpp) | Why the ramp cost is stamped unconditionally rather than max-merged. |
| [`amr_bringup/scripts/stop_stack.py`](ros2_ws/src/amr_bringup/scripts/stop_stack.py) | Process identification through `/proc` rather than `pkill -f` substring matching, with ancestors protected. |
| [`amr_bringup/scripts/send_goal.py`](ros2_ws/src/amr_bringup/scripts/send_goal.py) | Concurrent dispatch, and retry-on-rejection instead of treating an early goal as fatal. |
| [`REFACTORING.md`](REFACTORING.md) | The refactoring deliverable: the area, the plan, and the part implemented. |

---

## Architecture

```
              ┌──────────────────────────────────────────┐
              │              amr_core                    │
              │  RobotConfig · FleetManager · validators │
              │  smoother · safety · conflict            │
              │  (pure Python — no rclpy)                │
              └────────────────────┬─────────────────────┘
                                   │ every node reads its
                                   │ limits from here
     ┌───────────────┬─────────────┼─────────────┬────────────────┐
     │               │             │             │                │
┌────┴──────┐ ┌──────┴─────┐ ┌─────┴──────┐ ┌────┴───────┐ ┌──────┴─────┐
│amr_gazebo │ │amr_mapping │ │amr_safety  │ │amr_        │ │amr_        │
│           │ │            │ │            │ │navigation  │ │description │
│ world     │ │ selective  │ │ smoother   │ │ RampCost   │ │ URDF for   │
│ BSP gate  │ │ map filter │ │ safety     │ │ Layer      │ │ both models│
│ obstacles │ │ map fusion │ │ traffic    │ │ peer filter│ │            │
└───────────┘ └────────────┘ └────────────┘ └────────────┘ └────────────┘
```

Two structural decisions run through the codebase:

**Algorithms are ROS-free.** `MotionSmoother`, `SafetyMonitor`,
`TrafficPolicy`, `ImuValidator`, `LidarValidator`, `RateMonitor` and
`FleetManager` are plain Python classes with no `rclpy` import. The nodes are
thin adapters. That is why the control laws can be tested directly — including
boundary cases and failure modes that are impractical to stage in a simulator —
rather than inferred by watching a robot.

**Configuration has one home.** Physical limits live in
`amr_core/config/robot_models.yaml`, the roster in `fleet.yaml`. No node
branches on a robot's name; everything that differs between robots arrives as
configuration. That is what makes a ten-robot fleet a config file rather than a
code change.

---

## The command chain

This is what makes the safety override an override rather than a suggestion:

```
   Nav2 controller_server
          │  /<robot>/cmd_vel_nav
          ▼
   ┌──────────────────────────────────────────────────┐
   │ motion_smoother_node                             │
   │  • applies the traffic controller's speed_scale  │
   │  • enforces this model's acceleration and jerk   │
   │    limits, scaled by payload                     │
   └──────────────────────────────────────────────────┘
          │  /<robot>/cmd_vel_smoothed_local
          ▼
   ┌──────────────────────────────────────────────────┐
   │ safety_override_node       ← SOLE cmd_vel writer │
   │  • d_safe = k·v² + d_min                         │
   │  • zeroes translation on violation, keeps yaw    │
   │  • holds through a recovery gate before resuming │
   └──────────────────────────────────────────────────┘
          │  /<robot>/cmd_vel
          ▼
      Ignition diff-drive → wheels
```

Three consequences worth stating:

- **A yield is a controlled stop.** The traffic controller scales the *target*
  velocity before the smoother shapes it, so a yield comes out as a jerk-limited
  deceleration — measured as `speed_scale` stepping 1.0 → 0.35 → 0.0, never a
  velocity discontinuity.
- **A safety halt is not.** It bypasses the smoother, because "stop now" is
  exactly the case where a smooth ramp-down is wrong.
- **Rotation always survives a halt.** Translation is forbidden inside
  `d_safe`, but turning never is. Turning is the *only* way out of a halt: the
  monitor will not release until `min_range` exceeds `d_safe + hysteresis`, and
  a robot that may not turn cannot change what is in its forward sector.
  Gating rotation on the circumscribed radius, as this once did, wedged AMR-1
  permanently — measured sitting at `min_range` 0.30 m emitting `holding` until
  the goal timed out. It is safe unconditionally because the monitor discards
  returns below `inscribed_radius + 0.05` as self-returns, so the nearest
  obstacle it can report at the edge of the forward sector is 0.51 m from base
  centre on AMR-1 against a 0.43 m sweep radius (AMR-2: 0.40 m against 0.32 m).

**DWB's limits must agree with the smoother's.** DWB scores sampled
trajectories over `sim_time`, so the ratio of linear to angular authority
decides whether driving or turning wins. AMR-1 originally had `acc_lim_x` 0.4
against `acc_lim_theta` 1.2: a 1.7 s horizon reached 0.58 m forward but 99° of
rotation, so turning always scored better and the robot wandered instead of
tracking its path. It was also allowed `max_vel_theta` 1.2 while the smoother
clamps angular velocity to 1.0 — planning turns it could not execute.

Now 1.0 / 0.8 / `sim_time` 2.5, giving 1.25 m of forward reach per horizon.
Measured after the change: **mean cross-track error 0.05 m, max 0.21 m over
20.9 m driven**. AMR-2 needed no equivalent change — its 1.0 : 1.2 ratio was
already balanced, which is why only AMR-1 showed the symptom.

The two arbitration layers fail in deliberately opposite directions. The
traffic controller **fails open**: with fewer than two live trajectories it
publishes `PROCEED`, because a quiet traffic controller must not silently
freeze the fleet. The safety override **fails closed**: a stale scan halts the
robot.

---

## The sensor path

Nothing downstream subscribes to a raw sensor topic. Data reaches navigation
only after passing validation:

```
 /<r>/scan ──┐                        ┌──▶ /<r>/scan_validated ──▶ SLAM, local costmap, safety
             ├──▶ [ sensor_bsp ] ─────┤
 /<r>/imu  ──┘                        ├──▶ /<r>/imu_validated  ──▶ EKF
                                      └──▶ /<r>/sensor_health  ──▶ diagnostics (JSON)
```

`Severity` decides the *action* (forward / forward-and-log / withhold).
`ValidationState` records the *reason* (`VALID`, `OUT_OF_RANGE`, `STALE`,
`INVALID`). A range violation and a stale timestamp can both reject a message
but mean different faults.

IMU limits are per-axis internally even when configured as a scalar, so a
violation always names the axis that caused it:

```
[WARN] [BSP][amr1][IMU] angular velocity exceeded physical limit:
       axis=z measured=+25.000 rad/s limit=2.000 rad/s
       stamp=412.500000000 state=OUT_OF_RANGE
```

A value *exactly* at the limit is accepted — the limit is the edge of the
plausible envelope, not the first bad value.

### Peer and dynamic masking

Downstream of validation, `peer_scan_filter.py` publishes
`/<robot>/scan_no_peers`, which is what **SLAM and the planner costmaps**
consume. It blanks returns falling on fleet peers and on the pedestrians and
third-party robots, so a person who walks past does not become a permanent
wall in the fused map.

The safety scan is deliberately **left unfiltered** — the safety override
must react to anything physically in front of the robot, peer or not.

Two consequences follow from this split, and both matter operationally:

- **Nav2 never routes around the other robot**, because it cannot see it.
  Inter-robot separation is the traffic controller's job, plus the safety halt
  as the backstop. This is why spawn lanes must not aim the robots across each
  other — see *Spawn lanes*.
- **The filter fails closed.** If a peer has never been located it withholds
  the scan rather than publishing an unmasked one, because this output feeds
  SLAM and one unmasked frame becomes permanent structure. Peers resolve from
  the Gazebo truth topic rather than TF, which is what stops fail-closed from
  deadlocking against SLAM — see *Goal completion*.

---

## The warehouse

58 m × 36 m of mapped space, generated rather than hand-authored:

```bash
ros2 run amr_gazebo generate_warehouse_world.py
```

| Feature | Purpose |
|---|---|
| **Mezzanine deck**, 8 m × 12 m at z = 1.0 m | Reachable only by ramp, so a goal on the deck forces ramp traversal. |
| **Two ramps**, 10 m run for 1 m rise, 4 m wide | The graded alternative. Passable but expensive, so route choice is observable. |
| **Deck edges and ramp kerbs** | Stamped `LETHAL` by the cost layer — a 2D grid cannot express height, so without them the planner routes off the edge. |
| **Rack aisles** | Real aisle-routing problems for the global planner. |
| **8 dynamic obstacles** | Four pedestrians and four third-party robots on zoned patrol loops, with collision geometry so the LiDAR genuinely sees them. |

`platform_east` (5.0, 0.5) and `platform_west` (2.0, -0.8) are both **on the
mezzanine deck**, so sending both robots there exercises ramp traversal for the
whole fleet — verified with both at z = 1.00, AMR-1 in 98 s and AMR-2 in 104 s
from a fresh launch.

The deck's central band |y| < 2.6 is kept free of patrol traffic. `ped_4` and
`robot_4` previously patrolled to y = ±1, which put a pedestrian directly on
the approach: AMR-1 climbed the ramp, reached the deck, and was then blocked at
(3.75, 2.02) with its own cell lethal. The lanes now start at y = ±2.6.

**Goals must be verified against a fully-explored costmap.** A goal needs
`robot_radius` (0.35) + `inflation_radius` (0.55) = **0.95 m** of clearance
before a robot can finish on it. `heavy_storage` was once placed at
(-19.0, -6.0), which measured 1.3 m on a partly-built map and **0.2 m** once
the area was actually explored: AMR-1 drove to within 0.45 m, its own cell went
lethal, DWB stopped emitting commands and the goal aborted every single time.
The robot was fine; the goal was unreachable. It now sits at (-20.5, 1.5) with
1.8 m clear.

Named goals: `heavy_storage` (-20.5, 1.5), `packing_bay_4` (on the deck — every route to it
climbs a ramp), `packing_bay_4_farside` (reachable over the bridge *or* around
it on the flat, which is what makes the ramp cost observable),
`mezzanine_dock`, `ramp_side` (-5.0, 4.5 — alongside `ramp_up`, north of the
kerb, which the costmap blocks out to y = 3.2), and `rack_aisle`.

**`rack_aisle` (-15.5, -4.0) is sized for AMR-2 and nothing bigger.** The
y = -4 rack row carries racks at x = -17 and x = -14; each `pallet_rack` is
1.4 m long in x, so their faces sit at -16.3 and -14.7 and the slot between
them is **1.60 m** wide. AMR-2 parks in the middle with 0.80 m per side, which
clears the 0.55 m inflation (goal cell cost 0), clears its 0.32 m
circumscribed radius (it can still turn on the spot), and keeps the racks out
of its ±40° cone until 1.24 m — past the 0.50 m that would halt it. The
planner threads a 0.50 m zero-cost corridor.

Its `yaw` is 90°, facing *along* the slot. Facing across it would put a rack
0.80 m dead ahead. That distinction is not cosmetic: AMR-1 was first sent to
the y = -6 aisle (racks at -4 and -8, 3.1 m wide, 1.55 m *both* sides) and
wedged 0.9 m short at (-22.46, -5.29) with 46 halts at `min_range` 0.30–0.36 m
— its 0.65 m cone sweeps into a rack on either side whenever it rotates to
finish. Tight slots are AMR-2 work.

Obstacle patrol zones deliberately avoid the goals. `robot_1`'s lane once
overlapped `heavy_storage`, so a robot parked on its goal was driven into by a
patroller that has no collision avoidance of its own — measured as a 0.15 m
overlap sustained while the parked robot's safety override was correctly quiet,
because nothing was in its forward cone.

---

## Scaling the fleet

Growing the fleet is a configuration change. It is one file:

```bash
ros2 launch amr_bringup fleet_10_demo.launch.py
ros2 node list        # /amr1/sensor_bsp_node … /amr10/sensor_bsp_node
```

Verified: 10 namespaced nodes, 30 validated and diagnostic topics, no
collisions — including the `/amr1` versus `/amr10` prefix case, which is a real
source of cross-talk — and each robot loading its own configured limits
(`heavy_tugger` 2.0 rad/s, `light_scout` 3.5 rad/s) from the same
`SensorBspNode` class.

This works because nothing names a robot: the launch file reads the roster and
loops, `RobotInstance` owns one robot's components and branches on nothing, and
`FleetManager` composes rather than decides.

Adding AMR-3…AMR-10 is entries in a fleet YAML:

```yaml
robots:
  - {name: "amr3", model: "light_scout", x: -24.0, y: -10.0, yaw: 0.0,
     yield_priority: 45}      # any model field may be overridden per robot
```

Two invariants are enforced at load time, so a bad roster fails immediately
rather than misbehaving later:

1. **Unique names** — the name is used verbatim as a ROS namespace.
2. **Unique yield priorities** — equal priorities make the yield decision
   ambiguous, which is how an intermittent deadlock gets shipped.

Per-robot overrides exist because `yield_priority` is a *model* field: without
them the fleet could never exceed the size of the model library. An override
naming a field the model does not define is rejected, since a silently ignored
typo would hand the robot its model default while the config claimed otherwise.

---

## Packages

| Package | Contents |
|---|---|
| `amr_core` | `RobotConfig`, `FleetConfig`, `FleetManager`, `RobotInstance`, motion smoothing, safety, conflict, sensor validation. No `rclpy`. |
| `amr_description` | Parametric xacro; one macro renders both models. |
| `amr_gazebo` | Generated warehouse world, BSP validation gateway, dynamic-obstacle driver. |
| `amr_mapping` | Selective map filter, map fusion, EKF and SLAM configuration, ground-truth localisation aid. |
| `amr_navigation` | `RampCostLayer` (C++ costmap plugin), peer scan filter, Nav2 parameters, goal dispatch. |
| `amr_safety` | Motion smoother, traffic controller, safety override. |
| `amr_bringup` | Launch tree, RViz config, DDS profile, and the operator scripts: `fleet_ready.py`, `send_goal.py`, `stop_stack.py`. |

---

## Testing

```bash
colcon test && colcon test-result
```

**92 tests, 0 failures** — 72 unit tests plus every `ament` linter:

| Suite | Tests | Covers |
|---|---:|---|
| `test_sensor_bsp.py` | 32 | Every validator rule, per-axis attribution, boundary policy, frame ownership, rate shortfall. |
| `test_control_logic.py` | 25 | Smoother bounds and overshoot, safety envelope and recovery gate, conflict detection, yielding, deadlock escape. |
| `test_fleet_scaling.py` | 14 | Ten-robot construction, namespacing, prefix collisions, raw/validated separation, config rejection paths. |
| `test_style_pep8.py` | 1 | PEP 8 across every custom node. |

Tests assert properties the requirements depend on. A few that are
load-bearing:

- **Jerk overshoot** — caught a real defect where the limiter accelerated at
  full authority into the target and overshot 0.8 m/s to 0.92 m/s.
- **Gravity compensation** — the IMU acceleration check compares *proper*
  acceleration; testing raw magnitude flags every healthy stationary sample.
- **Self-return floor** — asserts the floor stays below `d_min`, the guard
  against a bug that made the override blind below 0.58 m/s.
- **Boundary policy** — a value exactly at the limit passes; one ulp above does
  not.
- **`/amr1` vs `/amr10`** — string-prefix collision, a real source of
  cross-talk in a namespaced fleet.

### Style

PEP 8 and PEP 257 for Python, Google C++ Style for C++, all enforced in
`colcon test` via `ament_flake8`, `ament_pep257`, `ament_cpplint`,
`ament_uncrustify` and `ament_copyright`.

The source carries **no explanatory comments** — only Apache licence headers
(required by `ament_copyright`) and one-line docstrings (required by
`ament_pep257`). Design rationale is documented here and in
[REFACTORING.md](REFACTORING.md) rather than inline.

Two exemptions, each recorded where the tool reads it:
`generate_warehouse_world.py`'s SDF templates are exempt from E501 because
wrapping them would change the generated world and `# noqa` would land inside
the string; and `D213` is disabled where it contradicts `D212`.

---

## Configuration reference

Three files, with distinct jobs.

**`amr_core/config/robot_models.yaml`** — what a *model* of robot is:

| Key | Meaning |
|---|---|
| `human_name`, `role` | descriptive; used in logs |
| `footprint_radius`, `inscribed_radius` | circumscribed and inscribed extents (m) |
| `body_half_length` | centre to front bumper; `safety_d_min` must exceed it |
| `chassis_length`, `chassis_width`, `wheel_radius` | geometry mirrored from the URDF |
| `payload_capacity_kg` | rated load |
| `max_vel_x`, `max_accel_x`, `max_jerk_x`, `max_decel_x` | linear dynamics |
| `max_vel_theta`, `max_accel_theta`, `max_jerk_theta` | angular dynamics |
| `payload_accel_scale`, `payload_jerk_scale`, `speed_jerk_gain` | load and speed scaling |
| `safety_k`, `safety_d_min` | the `d_safe = k·v² + d_min` envelope |
| `safety_sector_half_angle_deg` | forward cone the safety monitor watches |
| `imu_max_angular_velocity` | scalar **or** `{x, y, z}` per-axis limit (rad/s) |
| `imu_max_linear_acceleration` | proper, gravity-compensated (m/s²) |
| `lidar_range_min`, `lidar_range_max` | sensor envelope (m) |
| `sensor_max_age_sec`, `safety_scan_stale_sec` | freshness limits |
| `yield_priority` | higher wins; unique across the fleet |

**`amr_core/config/fleet.yaml`** (and `fleet_10.yaml`) — which robots exist:
roster, spawn poses, map bounds, ramp extents, and the traffic policy
(`horizon_seconds`, `conflict_margin`, `clear_margin`, `slow_speed_scale`).

### Spawn lanes

The two robots spawn **side by side** in the open west floor: same x, 2.5 m
apart, both facing +x — AMR-1 at (-24.0, +1.25), AMR-2 at (-24.0, -1.25).
Abreast rather than nose-to-tail is the point. The earlier pair, (-18, 0) and
(-14, 0), put AMR-2 four metres directly *ahead* of AMR-1, inside its forward
sector; a peer at 90° is outside the ±40° cone, so neither robot brakes for
the other or treats it as structure.

**Lane order is load-bearing: each robot takes the lane on the side its goal
is on.** Peers are filtered out of the planner's costmap deliberately (see
*The sensor path*), so Nav2 will never route around the other robot — only the
traffic controller and the safety halt separate them. Aim them across each
other and they collide: measured AMR-2 climbing *onto* AMR-1, 0.39 m apart at
z = 0.25, 323 safety halts, both goals lost. The shipped pairing is AMR-1
north → `packing_bay_4`, AMR-2 south → `rack_aisle`. **Re-check this whenever
a goal changes sides**, and swap the two `y` values if so.

**`amr_navigation/config/nav2_params_amr{1,2}.yaml`** — Nav2 tuning and the
`RampCostLayer` region lists. See [REFACTORING.md](REFACTORING.md) for the plan
to de-duplicate these.

The two files no longer differ only in dynamics. AMR-1 runs
`nav2_controller::PoseProgressChecker` and `nav2_controller::StoppedGoalChecker`
at 0.12 m / 0.12 rad; AMR-2 keeps the stock `Simple*` pair at 0.3. The reasons
are in *Goal completion* below, and the choice is a genuine trade: AMR-1 parks
to a few centimetres but takes minutes to settle on a long route, where AMR-2
finishes in tens of seconds to ~0.25 m.

Launch arguments:

| Argument | Default | Effect |
|---|---|---|
| `headless` | `false` | Gazebo server only |
| `goals` | `false` | dispatch the concurrent goals |
| `rviz` | `false` | open RViz |
| `obstacles` | `true` | run the dynamic obstacle field |
| `perfect_localization` | `false` | simulation aid: anchor TF to ground truth |
| `use_safety_chain` | `false` | controller publishes `cmd_vel_nav` rather than `cmd_vel` |

`fleet.launch.py` takes `headless`, `rviz` (default **true**), `obstacles` and
`perfect_localization` (default **true**, so a goal given after startup is
reachable).
| `fleet` | `fleet_10.yaml` | which roster `fleet_10_demo.launch.py` brings up |

---

## Goal completion

Both robots complete their goals from spawn. Latest measured run, default
pairing, `headless:=true`:

| | goal | result | landed | error |
|---|---|---|---|---|
| AMR-1 | `packing_bay_4` (1.0, 0.0) | REACHED 63 s | (0.845, -0.075) | 0.19 m |
| AMR-2 | `rack_aisle` (-15.5, -4.0) | REACHED 58 s | (-15.685, -3.808) | 0.27 m |

Safety halts: AMR-1 **1**, AMR-2 **14**. Planner `lethal space`: **0** on the
run that matters.

### `Starting point in lethal space` — resolved

```
GridBased: failed to create plan, invalid use:
Starting point in lethal space! Cannot create feasible plan..
```

This was long attributed to localisation drift. It was not drift. The tell is
in the timing: AMR-1's **first** plan attempt failed this way, roughly six
seconds after Nav2 came up and before the robot had moved a centimetre, and
**75 of 75** of its planner failures were this same error, while AMR-2 saw
3 — a drifting fleet does not fail asymmetrically before it moves.

The cause was `peer_scan_filter.py` **failing open**. When the peer's TF could
not be resolved it published the raw, unmasked scan — and that topic feeds
`slam_toolbox`. At startup the peer TF *cannot* resolve; the planner log says
so directly (`Could not find a connection between 'map' and
'amr1/base_footprint' … Tf has two or more unconnected trees`). So for the
first minute each robot baked the other's body into its map as permanent
structure. With the old spawns AMR-2 sat four metres directly ahead of AMR-1,
that occupancy reached the fused `/map`, fed `static_layer`, and marked
AMR-1's own start cell lethal — permanently, since nothing clears a static
layer cell. AMR-2 escaped only because it drove away before its cell mattered.

The filter now resolves peers from the Gazebo truth topic it already
subscribes to, and withholds a scan only if a peer has *never* been located.
A plain fail-closed would deadlock: `ground_truth_localization` needs SLAM's
TF, SLAM needs the filter, and the filter needs `ground_truth_localization`.
Truth poses break that cycle because they need no SLAM. After the fix:
**0 lethal-space failures**, and AMR-1 drives from spawn to its goal.

### What the goal checkers are for

Two further defects sat behind this one, both on AMR-1, both fixed in
`nav2_params_amr1.yaml`:

* `SimpleProgressChecker` counts only **translation**. During the final
  rotate-to-goal a robot legitimately does not translate, so after 30 s it was
  declared stuck and aborted **3.6 cm from its goal**. Now
  `PoseProgressChecker`, which counts rotation as progress.
* `SimpleGoalChecker` succeeds the instant the pose is inside tolerance —
  which happens mid-rotation, after which the controller stops commanding and
  the robot coasts on momentum. Measured: success declared, robot settled
  0.15 m and 70° past. Now `StoppedGoalChecker`.

DWB's own `xy_goal_tolerance` must equal the goal checker's. Left wider, the
`RotateToGoal` critic takes over early and penalises translation, so the robot
spins at the larger radius and never closes the gap.

### Localisation

`perfect_localization` defaults to **true** on `fleet.launch.py`, so
`map→{robot}/map` is anchored to Gazebo truth and the poses above are ground
truth. The underlying drift work is unchanged and unre-measured here:

* **Fixed previously** — the EKF fused absolute wheel-odometry X/Y and so
  adopted encoder drift as truth. It now fuses forward velocity only, with the
  IMU supplying orientation.
* **Not the cause** — dynamic obstacles; freezing them changed nothing.
* **Open** — on live SLAM, `map→odom` reached 8 m in magnitude rather than
  sitting near identity, with 4–5 m of position error measured after ~20 m of
  driving. The intended resolution is still to localise with AMCL against the
  fused map during navigation rather than driving on live SLAM. Building a map
  and localising in one are different jobs.

### Known rough edges

* AMR-1's final pose can drift slightly *after* success is declared, with
  `cmd_vel` at zero — seen as ~0.19 m of position error against its own 0.12 m
  tolerance in one run, and ~40° of yaw in another. Unexplained.
* The tight-tolerance settings make AMR-1 slow to finish: 511 s on a 20 m
  route, against tens of seconds for AMR-2. Relax its tolerances if cycle time
  matters more than centimetres.
* Nav2 bringup still stalls intermittently with both `bt_navigator`s stuck at
  `Configuring`. The banner now reports this honestly instead of declaring
  readiness; relaunch.

---

## Troubleshooting

**Objects flicker.** A second simulator is running — two Ignition servers on
one world publish two poses for every model and the viewer alternates between
them. It is never a rendering problem.

```bash
ros2 run amr_bringup stop_stack.py --dry-run   # what is running
ros2 run amr_bringup stop_stack.py             # stop it
```

The launch also refuses to start on top of a live simulator, printing the
offending PIDs. Two details learned the hard way: `ign gazebo` spawns
`ign gazebo server` as a **separate child** that survives its parent and is
invisible to `ros2 node list`; and Ignition re-execs with its whole command
line as a single `argv` element, so a check on `argv[0]` alone misses it.

`stop_stack.py` identifies processes structurally through `/proc` rather than by
`pkill -f` substring matching, and never kills its own ancestors. Classic
Gazebo from other projects is never touched.

**Bringup stalls at "Configuring smoother_server".** An intermittent DDS flake:
a server advertises `change_state` but its executor is unreachable, so the
lifecycle manager blocks and that robot's Nav2 never activates. The tells are
`failed to send response to /<robot>/smoother_server/change_state (timeout)` in
the log, and a missing `/<robot>/global_costmap/costmap` while the other robot
has it. Stop and relaunch; confirm `Managed nodes are active` appears **twice**
before dispatching goals.

**Robots do not move.** Check `use_safety_chain` matches what is running — with
`true` and no safety stack, nothing subscribes to `cmd_vel_nav` and the only
publisher of `cmd_vel` is the recovery behaviour server, so the robot only ever
spins.

**Robots revolve on the spot.** That is Nav2's `spin` recovery firing because
the planner cannot produce a path. No controller tuning fixes it, because
there is no path to follow — check `planner_server`'s log for the reason. If
it reads `Starting point in lethal space`, see
[Goal completion](#goal-completion); persistent occurrences from the very
first plan mean something has been written into the fused map that should not
be there, not that the controller is mistuned.

**`XMLPARSER` errors, SLAM hangs with 0 map cells.** FastDDS shared-memory
transport failing on stale `/dev/shm` locks.
`amr_bringup/config/fastdds_udp_only.xml` forces UDP; the launch files point
`FASTRTPS_DEFAULT_PROFILES_FILE` at it.

**One robot "will not accept goals".** Two different causes, worth separating.
If `send_goal.py` reports the goal *rejected*, its Nav2 is not ACTIVE yet —
the readiness banner now waits for that, so this should only appear if you
bypass it. If a goal is never *received* at all while the other robot's is,
check the dispatcher: an action server existing never implies it accepts
goals, and both `fleet_ready.py` and `send_concurrent_goals.py` therefore poll
`bt_navigator/get_state` for `PRIMARY_STATE_ACTIVE`. AMR-2 launches
`ROBOT_STAGGER_SEC` behind AMR-1, so it is always the one this bites.

**`ros2` CLI half-sees the fleet; `send_goal.py` hangs with no output.** The
running nodes are forced onto UDP-only DDS by the profile above, and a plain
CLI shell defaults to shared memory. It then matches only partially: `ros2
topic list` returns almost nothing and `ros2 action send_goal` blocks forever
without an error, which reads exactly like a dead stack. Export the same
profile in that shell first:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix amr_bringup)/share/amr_bringup/config/fastdds_udp_only.xml
```

Note also that `ros2 action send_goal` block-buffers stdout when piped — pipe
through `stdbuf -oL` or redirect to a file, or a timeout kill discards
everything it was about to print.

**A parameter seems ignored.** The YAML root key must match the node name
exactly, namespace included. On a mismatch ROS 2 falls back to the code default
with no warning.

**`No module named 'amr_core'`.** The workspace was built with
`--symlink-install`, or the shell was sourced before the build. Rebuild with
plain `colcon build` and re-source in a fresh terminal.

---

## Licence

Apache-2.0.
