# SO-100 Robot Arm — ROS2 (Tonoward fork)

ROS2 support for the SO-100 5-DOF robot arm with Feetech STS3215 smart servos.
Based on [brukg/SO-100-arm](https://github.com/brukg/SO-100-arm), which in turn
builds on the 3D-printable [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
project by The Robot Studio.

## What this fork changes

This fork is a **self-contained source of truth** — no build-time patching.

- **Vendored hardware driver.** The `so_arm_100_hardware` package (originally a
  separate repo, `brukg/so_arm_100_hardware`) now lives inside this repo. Its
  source targets the **ROS2 Humble** `ros2_control` API directly (no compat
  patch needed at build time).
- **Serial port baked in.** The arm's udev by-id path
  (`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970073059-if00`) is the
  default everywhere. Override with `serial_port:=...` if your device differs.
- **Servo calibration.** `so_arm_100_hardware/config/calibration.yaml` maps each
  servo's raw ticks to joint radians (`zero_ticks` + `direction`) and is loaded
  by default at launch. See [Calibration](#calibration).
- **Position correction + live tuning.** Per-joint closed-loop trim (measures
  real encoder error and corrects it — no physics model needed) plus an
  optional KDL-based gravity feedforward, both tunable at runtime with no
  rebuild. See [Position correction & live tuning](#position-correction--live-tuning).
- **Strict controller tolerances.** The `joint_trajectory_controller` now has
  real `goal`/`trajectory` constraints, so a move that misses its target is
  reported as `ABORTED` instead of a false `SUCCEEDED`.

## Prerequisites

- ROS2 Humble
- MoveIt2, ros2_control
- (Simulation only) Gazebo + gz_ros2_control

### Hardware

- SO-ARM-100 arm (5-DOF) with Feetech SMS/STS series servos
- USB-to-serial adapter (CH340 / `1a86:` VID)

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Tonoward/SO-100-arm.git
```

That single clone brings in every package, including the vendored
`so_arm_100_hardware`. Install ROS dependencies:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Building

Use the helper script (fixes serial permissions, sources ROS, builds up to
`so_arm_100`):

```bash
cd ~/ros2_ws
./build_so_arm_100.sh
```

The script edits `SERIAL_PORT` at the top if your device path differs. It runs
`sudo chmod 666` on the serial device, so it will prompt for your password.

Plain colcon works too:

```bash
cd ~/ros2_ws
colcon build --packages-up-to so_arm_100
source install/setup.bash
```

## Usage

Source the workspace first: `source ~/ros2_ws/install/setup.bash`

### Verify servo communication

```bash
ros2 run so_arm_100_hardware test_servo
```

Reads ping / position / voltage / temperature for servos 1–6. Use this to grab
raw tick values when calibrating.

### Launch the hardware interface

The serial port and calibration file are baked in as defaults, so the bare
command works:

```bash
ros2 launch so_arm_100_bringup hardware.launch.py
```

Useful overrides:

```bash
# Different serial device
ros2 launch so_arm_100_bringup hardware.launch.py \
  serial_port:=/dev/ttyACM0

# Disable calibration (use raw servo ticks)
ros2 launch so_arm_100_bringup hardware.launch.py \
  calibration_file:=""

# Explicit calibration file
ros2 launch so_arm_100_bringup hardware.launch.py \
  calibration_file:=$HOME/ros2_ws/src/SO-100-arm/so_arm_100_hardware/config/calibration.yaml
```

### Launch MoveIt2 (planning + RViz)

```bash
ros2 launch so_arm_100_moveit_config demo.launch.py
```

This drives the **real** hardware (the serial port default is baked into the
xacro). Plan and execute from the RViz MotionPlanning panel.

**Planning groups** (RViz MotionPlanning → *Planning Group*):

| Group | Joints | Cartesian marker | Use for |
|-------|--------|:---:|---------|
| `arm` | 5 arm joints | ✅ | Dragging the arm to a Cartesian pose |
| `gripper` | Gripper | — | Opening/closing the gripper alone |
| `arm_gripper` | all 6 | — | **One plan that moves the arm and gripper together** |

To move the arm *and* open/close the gripper in a **single** trajectory, select
`arm_gripper`, set goals in the *Joints* tab (or pick a named state —
`home_open` / `home_closed`), then Plan & Execute. MoveIt splits execution
across `arm_controller` and `gripper_controller` and sends them simultaneously.
The `arm_gripper` group plans in joint space (it has no Cartesian IK solver,
since the gripper branches off the arm chain) — for drag-the-marker Cartesian
arm goals, use the `arm` group.

> `moveit_controllers.yaml`'s `controller_names` deliberately lists only
> `arm_controller` and `gripper_controller` — **not** the `*_effort_controller`
> variants. `demo.launch.py` spawns every listed controller unconditionally as
> active (`moveit_configs_utils`'s built-in spawn logic has no "default vs.
> inactive" handling), and the effort controllers claim the same command
> interfaces (effort) as the position ones for the same joints — only one of
> each conflicting pair can actually activate. For the gripper this meant
> `gripper_effort_controller` silently won the claim, leaving
> `gripper_controller` inactive with no action server, so `arm_gripper`
> executions always failed with "Action client not connected." If you need
> effort control, load/activate `arm_effort_controller` /
> `gripper_effort_controller` manually (`ros2 control load_controller` +
> `switch_controllers`), deactivating the position-based ones first.

### Simulation / visualization only

```bash
ros2 launch so_arm_100 gz.launch.py dof:5     # Gazebo
ros2 launch so_arm_100 rviz.launch.py         # RViz (no hardware)
```

### Send a test trajectory

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [Shoulder_Rotation, Shoulder_Pitch, Elbow, Wrist_Pitch, Wrist_Roll],
    points: [
      { positions: [-0.5, -1.0, 0.5, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0} },
      { positions: [-0.5,  0.5, 0.0, 0.0, 0.0], time_from_start: {sec: 4, nanosec: 0} }
    ]
  }
}"
```

### Gripper

The gripper is driven by its own `JointTrajectoryController` (same mechanism as
the arm), so it can be **planned and executed from MoveIt** like any other
group: in the RViz MotionPlanning panel, set *Planning Group* to `gripper`, set
a target from the *Joints* tab (or the `open`/`close` named states), then Plan
& Execute.

> Originally the gripper used `parallel_gripper_action_controller` +
> `ParallelGripperCommand`, neither of which is available in this ROS2 Humble
> image — so the gripper silently failed to load and couldn't be driven from
> MoveIt. It's now a plain trajectory controller (see `ros2_controllers.yaml`
> and `moveit_controllers.yaml`).

From the CLI, publish a trajectory directly (Gripper range ≈ `-0.13` closed to
`0.79` open):

```bash
# Open
ros2 topic pub -1 /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [Gripper], points: [{positions: [0.7854], time_from_start: {sec: 1}}]}'

# Close
ros2 topic pub -1 /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [Gripper], points: [{positions: [-0.13], time_from_start: {sec: 1}}]}'
```

## Calibration

The hardware interface converts raw servo ticks (0–4095) to joint radians using:

```
radians = direction * (ticks - zero_ticks) * 2*pi / 4096
```

`so_arm_100_hardware/config/calibration.yaml` holds `zero_ticks` and `direction`
per joint, and is loaded by default (see [above](#launch-the-hardware-interface)).

**Important:** the URDF/SRDF "all joints = 0" pose is the arm fully
**extended**, not the folded rest pose. `zero_ticks` is chosen so raw ticks at
the folded pose map to the correct *non-zero* SRDF angles — it is **not**
simply "the tick reading at rest." Current values:

| Joint             | zero_ticks | direction |
|-------------------|-----------:|:---------:|
| Shoulder_Rotation |       2016 |    +1     |
| Shoulder_Pitch    |       1951 |    +1     |
| Elbow             |       2127 |    +1     |
| Wrist_Pitch       |       1988 |    +1     |
| Wrist_Roll        |       2068 |    +1     |
| Gripper           |       1844 |    +1     |

### ⚠️ If you use LeRobot on this arm

LeRobot's calibration writes directly to each servo's persistent EEPROM homing
offset. That silently invalidates every `zero_ticks` value above — the arm's
physical position won't change, but raw ticks reported for the same pose will
shift, and RViz will stop matching the physical arm. If that happens, redo
calibration below rather than assuming something is broken.

### Re-calibrating

1. Move the arm to its folded rest pose (the one that used to line up).
2. Read raw ticks: `ros2 run so_arm_100_hardware test_servo`.
3. For each joint, compute the tick delta from the table above
   (`new_reading - old_zero_ticks_reading_at_same_pose`) and add it to that
   joint's `zero_ticks`. Keep `direction` as-is — mounting orientation doesn't
   change.
4. Rebuild `so_arm_100_hardware` and re-launch; confirm RViz matches the arm.

If a single joint is off by a clean angle (e.g. ~90°) while the rest are fine,
don't guess-and-rebuild — use the **live zero_trim** parameter below to dial it
in interactively first, then bake the confirmed value into `zero_ticks`.

There are also runtime helper services on `/so_arm_100_driver/`:

```bash
ros2 service call /so_arm_100_driver/toggle_torque  std_srvs/srv/Trigger {}  # free/hold joints
ros2 service call /so_arm_100_driver/record_position std_srvs/srv/Trigger {} # dump current ticks
```

## Position correction & live tuning

Three independent, live-tunable ROS2 parameters on the `/so_arm_100_driver`
node correct for the arm under-reaching its commanded angle. **Prefer
`integral_gain`** — it's simpler, more robust, and needs no physics model.

### `integral_gain.<JointName>` (recommended)

Every `write()` cycle:

```
error = commanded_angle - actual_angle        # actual = real encoder reading
if |error| < 0.3 rad:
    integral_trim += integral_gain * error * dt   # clamped to +/-0.5 rad
corrected_command = commanded_angle + integral_trim
```

This measures the *real* error between target and actual position and grows a
correction until it disappears — no model of mass, torque, or geometry
required, so it automatically adapts to whatever the true disturbance is
(gravity, friction, backlash) at *any* pose. It's deliberately frozen (not
accumulating) outside the 0.3 rad deadband: during active fast motion, a large
error is normal trajectory-tracking lag, not steady-state sag, and integrating
that would fight the servo's own transient response and cause a jerk once the
move ends.

```bash
ros2 param set /so_arm_100_driver integral_gain.Elbow 0.4
```

**Tuning:** start around `0.3`–`0.5`. Move to a pose where the arm visibly
sags, hold it, and watch the shadow (the solid robot model, driven by real
encoder feedback) slowly creep toward the goal marker over a second or two. If
it's too slow, raise the gain; if it overshoots/oscillates, lower it.

**Current defaults** (baked into `calibration.yaml`, confirmed working):

| Joint | `integral_gain` |
|-------------------|-----:|
| Shoulder_Rotation |  0.1 |
| Shoulder_Pitch    |  0.3 |
| Elbow             |  1.0 |
| Wrist_Pitch       |  0.3 |
| Wrist_Roll        |  0.1 |
| Gripper           |  0.1 |

These load automatically on every launch — no `ros2 param set` needed unless
you want to retune. If you ever need to retune a joint, override it live first
and only edit `calibration.yaml` once you've confirmed a better value.

### `gravity_coefficient.<JointName>` (optional, physics-model-based)

```
corrected_command = commanded_angle + gravity_coefficient * gravity_torque(Nm)
```

`gravity_torque` comes from a **KDL model of the entire arm chain**, built at
activation from the URDF's real per-link mass/inertia data (received over
`/robot_description`) via `kdl_parser` + `KDL::ChainDynParam`. A joint's
compensation correctly accounts for the load every joint further out on the
chain adds — e.g. extending the Elbow increases the torque (and therefore the
correction) computed for Shoulder_Pitch, not just Shoulder_Pitch's own angle.
`Gripper` isn't part of that chain, so it (and any joint before the URDF has
arrived) falls back to `gravity_coefficient * sin(target_angle)`.

**Caveat:** a fixed coefficient applies a similarly large correction at *every*
pose with nonzero computed torque — including poses you might expect to need
none. For this arm's actual mass distribution, Elbow's gravity torque only
drops ~14% between fully extended and folded, so `gravity_coefficient` alone
can over-correct near the folded/"core" pose even while it's well-tuned for
extension. `integral_gain` doesn't have this problem, since it only applies as
much correction as the *measured* error actually calls for at each pose — use
`gravity_coefficient` only if you want a fast feedforward push in addition to
the integral trim, not as a replacement for it.

```bash
ros2 param set /so_arm_100_driver gravity_coefficient.Elbow 0.15
```

### `zero_trim.<JointName>` (live calibration nudge)

A temporary radian offset added to a joint's reported/commanded angle, for
fixing a calibration misalignment live before committing it to `zero_ticks`.

```bash
ros2 param set /so_arm_100_driver zero_trim.Wrist_Roll -1.5708
```

Once you find a value that fixes a misalignment, fold it into `zero_ticks`
permanently (`zero_ticks -= direction * round(trim * 4096 / (2*pi))`) and set
the parameter back to `0`, rather than leaving the correction live-only.

### GUI

RViz2 has no built-in slider panel for parameters — use `rqt_reconfigure`
alongside it:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```
Select `/so_arm_100_driver` from the node list; every `integral_gain.*`,
`gravity_coefficient.*`, and `zero_trim.*` shows up as a slider and updates the
arm on the next control cycle.

Once you have good values for any of these, set them as the defaults in
`calibration.yaml` (`integral_gain` / `gravity_coefficient` fields per joint)
so future launches start compensated instead of at `0.0`.

The shadow **will move** while you tune — that's expected, not a bug: these
corrections work by changing what's physically commanded to the servo, so the
true position (and therefore the shadow) shifts. The goal is for that movement
to converge onto the goal marker, not to prevent it from moving at all.

## Controller tolerances

`so_arm_100_moveit_config/config/*.yaml` (`hardware_controllers.yaml`,
`ros2_controllers.yaml`, `controllers_5dof.yaml`) define per-joint constraints
under `arm_controller`:

```yaml
constraints:
  stopped_velocity_tolerance: 0.01
  goal_time: 5.0        # seconds allowed after the trajectory to settle within goal
  Shoulder_Rotation: { goal: 0.05, trajectory: 0.0 }
  # ... same for the other joints
```

- **`goal`** (0.05 rad) — final-position accuracy, checked once the arm has
  settled. This is the tolerance that actually matters; loosen it if you get
  spurious `GOAL_TOLERANCE_VIOLATED` aborts, tighten for more precise stops.
- **`trajectory`** (path tolerance) is deliberately **`0.0` (disabled)**, not
  just loose. It checks real position against the naive, *uncompensated*
  interpolated path at every instant during motion — but gravity compensation
  (below) deliberately makes the real position deviate from that path in order
  to counteract sag, so a nonzero path tolerance fights the very feature that's
  supposed to make positioning reliable, and trips almost immediately
  (`PATH_TOLERANCE_VIOLATED`) as soon as compensation is doing anything
  meaningful. Leave it at `0.0` unless you disable gravity compensation.

MoveIt has its own separate, one-time check before execution even starts:
`trajectory_execution.allowed_start_tolerance` in `moveit_controllers.yaml`
(raised from MoveIt's default `0.01` to `0.15`) — this catches the case where
the arm hasn't finished settling under compensation between clicking Plan and
clicking Execute.

## Joint configuration (5-DOF)

| # | Joint             | Range (rad)     |
|---|-------------------|-----------------|
| 1 | Shoulder Rotation | −1.96 … 1.96    |
| 2 | Shoulder Pitch    | −1.745 … 1.745  |
| 3 | Elbow             | −1.5 … 1.5      |
| 4 | Wrist Pitch       | −1.658 … 1.658  |
| 5 | Wrist Roll        | −2.75 … 2.75    |
| 6 | Gripper           | −0.179 … 1.571  |

Note these limits are **asymmetric relative to the folded rest pose** — "0
rad" is the extended pose (see [Calibration](#calibration)), so a joint's
folded position sits well off-center within its range, not at the midpoint.

## Package layout

```
SO-100-arm/
├── so_arm_100/               # meta package, top-level launch (demo, gz, rviz)
├── so_arm_100_bringup/       # hardware.launch.py (real-robot bring-up)
├── so_arm_100_description/   # URDF / xacro, meshes
├── so_arm_100_moveit_config/ # MoveIt config, controllers, tolerances
└── so_arm_100_hardware/      # vendored ros2_control driver (Humble API)
    ├── src/so_arm_100_interface.cpp
    ├── config/calibration.yaml
    └── config/hardware_config.yaml
```

## License

Apache License — see `LICENSE`.

## Acknowledgments

- Upstream: [brukg/SO-100-arm](https://github.com/brukg/SO-100-arm) and
  [brukg/so_arm_100_hardware](https://github.com/brukg/so_arm_100_hardware) (Bruk G.)
- Based on the SO-ARM100 project by The Robot Studio.
