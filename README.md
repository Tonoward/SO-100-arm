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
- **Gravity compensation.** Optional per-joint feedforward to counter droop when
  the arm is extended. See [Gravity compensation](#gravity-compensation).
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
per joint. The current values were measured directly from `test_servo` with the
arm physically at its home pose (all joints at 0 rad):

| Joint             | zero_ticks | direction |
|-------------------|-----------:|:---------:|
| Shoulder_Rotation |       2053 |    +1     |
| Shoulder_Pitch    |        914 |    +1     |
| Elbow             |       3028 |    −1     |
| Wrist_Pitch       |       2859 |    −1     |
| Wrist_Roll        |       3063 |    +1     |
| Gripper           |       1889 |    +1     |

### Re-calibrating

1. Physically move the arm to its home / neutral pose.
2. Read the raw ticks: `ros2 run so_arm_100_hardware test_servo`.
3. Put each servo's `Position` value into `zero_ticks`.
4. If a joint moves the wrong way for a positive command, flip `direction`
   between `+1` and `-1`.
5. Rebuild (or just re-launch — the YAML is read at activation, no rebuild
   needed) and confirm `ros2 topic echo /joint_states` reads ~0 at home.

There are also runtime helper services on `/so_arm_100_driver/`:

```bash
ros2 service call /so_arm_100_driver/toggle_torque  std_srvs/srv/Trigger {}  # free/hold joints
ros2 service call /so_arm_100_driver/record_position std_srvs/srv/Trigger {} # dump current ticks
```

## Gravity compensation

When the arm is extended, gravity load makes the servos under-reach their
commanded angle. Each joint in `calibration.yaml` accepts an optional
`gravity_coefficient`. Before each position write the interface applies:

```
corrected_command = command + gravity_coefficient * sin(current_angle)
```

so the servo is told to aim slightly past the target to land on it under load.

**Tuning** (defaults are `0.0`, i.e. off):

1. Command a moderately extended pose (e.g. `Shoulder_Pitch ≈ 0.5 rad`).
2. If the arm droops below the RViz goal, raise that joint's
   `gravity_coefficient` by `0.05`.
3. Re-launch and repeat. Reduce it if the arm overshoots the goal.
4. `Shoulder_Pitch` and `Elbow` usually need the most compensation.

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

| # | Joint             | Range (rad) |
|---|-------------------|-------------|
| 1 | Shoulder Rotation | −3.14 … 3.14 |
| 2 | Shoulder Pitch    | −3.14 … 3.14 |
| 3 | Elbow             | −3.14 … 3.14 |
| 4 | Wrist Pitch       | −3.14 … 3.14 |
| 5 | Wrist Roll        | −3.14 … 3.14 |

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
