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
- **Gravity compensation + live tuning.** Optional per-joint feedforward to
  counter droop when extended, tunable at runtime with no rebuild. See
  [Gravity compensation & live tuning](#gravity-compensation--live-tuning).
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

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 1.57, max_effort: 50.0}}"   # open
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.0,  max_effort: 50.0}}"   # close
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

## Gravity compensation & live tuning

When the arm is extended, gravity makes the servos under-reach their commanded
angle. Before each position write, the driver applies:

```
corrected_command = command + gravity_coefficient * sin(current_angle)
```

so the servo aims slightly past the target to land on it under load.

Two values are tunable **live, with no rebuild or relaunch**, via ROS2
parameters on the `/so_arm_100_driver` node:

- **`gravity_coefficient.<JointName>`** — the sag compensation above.
- **`zero_trim.<JointName>`** — a temporary radian offset added to a joint's
  reported/commanded angle, for fixing a calibration misalignment live before
  committing it to `zero_ticks`.

Change them from the CLI:

```bash
ros2 param set /so_arm_100_driver gravity_coefficient.Elbow 0.15
ros2 param set /so_arm_100_driver zero_trim.Wrist_Roll -1.5708
```

Or with a GUI — RViz2 has no built-in slider panel for this, so use
`rqt_reconfigure` alongside it:

```bash
ros2 run rqt_reconfigure rqt_reconfigure
```
Select `/so_arm_100_driver` from the node list; every `gravity_coefficient.*`
and `zero_trim.*` shows up as a slider and updates the arm on the next control
cycle.

**Tuning gravity compensation:**

1. Command a moderately extended pose (e.g. `Shoulder_Pitch ≈ -1.0`).
2. Raise that joint's `gravity_coefficient` in `~0.05` steps while watching the
   physical arm vs. the RViz goal marker. Back off if it overshoots.
3. `Shoulder_Pitch` and `Elbow` typically need the most compensation.
4. Once you have good values, set them as the defaults in `calibration.yaml`
   (the `gravity_coefficient` field per joint) so future launches start
   compensated instead of at `0.0`.

## Controller tolerances

`so_arm_100_moveit_config/config/*.yaml` (`hardware_controllers.yaml`,
`ros2_controllers.yaml`, `controllers_5dof.yaml`) define per-joint constraints
under `arm_controller`:

```yaml
constraints:
  stopped_velocity_tolerance: 0.01
  goal_time: 5.0        # seconds allowed after the trajectory to settle within goal
  Shoulder_Rotation: { goal: 0.05, trajectory: 0.15 }
  # ... same for the other joints
```

These are read at controller load. Loosen `goal`/`trajectory` if you get
spurious `GOAL_TOLERANCE_VIOLATED` aborts; tighten for more precise stops.

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
