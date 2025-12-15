# SO-ARM101 MoveIt Isaac Sim Integration

## Introduction

This repository demonstrates how to integrate MoveIt with Isaac Sim using ROS2 on the SO-ARM101 dual-arm robot. The SO-ARM101 (and older SO-ARM100) is an open-source dual-arm robot created by [TheRobotStudio](https://github.com/TheRobotStudio/SO-ARM100) in collaboration with [HuggingFace's LeRobot](https://github.com/huggingface/lerobot) for robotics research and education.

[MoveIt](https://moveit.ai/) provides robot motion planning, manipulation, and control capabilities in ROS2, enabling control of real robots from simulation. This tutorial focuses on simulation-only implementation, with Sim2Real capabilities planned for future releases.

**📺 Full video tutorial available on [LycheeAI YouTube Channel](https://www.youtube.com/@LycheeAI)**

## Get Your Own SO-ARM Robot 🤖

Want the real robot? Get your own SO-ARM101 (or SO-ARM100) from [WowRobo](https://shop.wowrobo.com/?sca_ref=8879221.Q7i7RSlTAVB) using discount code **`LYCHEEAI5`**

*Make sure to "Add to cart" or choose "More payment options" to apply the discount code!*

## Prerequisites

- **Isaac Sim** - [Installation Guide: Isaac Sim & Isaac Lab](https://www.notion.so/Installation-Guide-Isaac-Sim-Isaac-Lab-1c428763942b8089ad48cf88e01ad213?pvs=21)
- **Linux Ubuntu 22.04** with **ROS2 Humble**
- **ROS2 Humble Installation** - [Official Guide](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_ros.html)
- **ROS2 Basics Knowledge** - Complete the [TurtleBot Tutorial](https://www.youtube.com/watch?v=3cWQsvpwvQU&ab_channel=Nex-dynamicsrobot)

## Quick Start (Skip Tutorial)

If you want to skip the step-by-step tutorial or encounter issues, use this quick setup:

```bash
# Clone the repository
git clone https://github.com/MuammerBay/SO-ARM101_MoveIt_IsaacSim.git
cd SO-ARM101_MoveIt_IsaacSim

# Source ROS 2
source /opt/ros/humble/setup.bash

# Update rosdep database
rosdep update

# Install ALL dependencies automatically
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Launch MoveIt demo
ros2 launch so_arm_moveit_config demo.launch.py
```

### Start Isaac Sim (Separate Terminal)

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Run Isaac Sim with ROS2 enabled
[ISAAC-SIM-4.5-FOLDER]/isaac-sim.selector.sh
```

### Final Steps
1. Open the USD file (created from URDF) in Isaac Sim
2. Start the simulation (press Play)
3. Control the robot in RViz

## Step-by-Step Tutorial

*This tutorial is based on the excellent work by [robot mania](https://www.youtube.com/@robotmania8896) - highly recommended channel for robotics topics!*

### 1. ROS2 & MoveIt Setup

Source ROS2 Humble in every terminal:
```bash
source /opt/ros/humble/setup.bash
```

Create ROS workspace:
```bash
mkdir -p ~/so-arm_moveit_isaacsim_ws/src
cd ~/so-arm_moveit_isaacsim_ws
```

Install ROS2 dependencies:
```bash
sudo apt update
sudo apt install \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gripper-controllers \
  ros-humble-topic-based-ros2-control
```

Create package directories:
```bash
cd ~/so-arm_moveit_isaacsim_ws/src
mkdir -p so_arm_moveit_config
mkdir -p so_arm_description
```

### 2. Robot Description Setup

Clone the SO-ARM URDF (adjusted for proper ROS2 format):
```bash
cd ~/so-arm_moveit_isaacsim_ws/src/so_arm_description
git clone https://github.com/MuammerBay/SO-ARM_ROS2_URDF.git .
```

Build workspace:
```bash
cd ~/so-arm_moveit_isaacsim_ws
colcon build
```

### 3. MoveIt Setup Assistant

Launch MoveIt Setup Assistant:
```bash
source ~/so-arm_moveit_isaacsim_ws/install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

After setup, rebuild:
```bash
cd ~/so-arm_moveit_isaacsim_ws
colcon build
```

### 4. Configuration Adjustments

**Fix joint_limits.yaml**: Replace Integer values with Float values

**Update moveit_controllers.yaml**: Add to arm_controller:
```yaml
action_ns: follow_joint_trajectory
default: true
```

Rebuild after changes:
```bash
colcon build
```

Test MoveIt without Isaac Sim:
```bash
ros2 launch so_arm_moveit_config demo.launch.py
```

### 5. Isaac Sim Integration

Open Isaac Sim:
```bash
source /opt/ros/humble/setup.bash
[ISAAC-SIM-4.5-FOLDER]/isaac-sim.selector.sh
```

**Import URDF** with these settings:
- Stiffness: 17.8
- Damping: 0.60
- Set articulation root to base mesh (xform)

**Create ROS2 Action Graph** (simulation only, for historical reference):
- Tools → Robotics → ROS2 Omnigraphs → Joint States
- Articulation: base mesh (xform)
- Joint States Topic: `/so_arm/hw_joint_states`
- Joint Commands Topic: `/so_arm/hw_joint_command`

### 6. Final Configuration

Update `src/so_arm_moveit_config/config/so101_new_calib.ros2_control.xacro`:

Replace:
```xml
<plugin>mock_components/GenericSystem</plugin>
```

With:
```xml
<!-- <plugin>mock_components/GenericSystem</plugin> -->
<plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
<param name="joint_states_topic">/so_arm/hw_joint_states</param>
<param name="joint_commands_topic">/so_arm/hw_joint_command</param>
```

Final rebuild:
```bash
cd ~/so-arm_moveit_isaacsim_ws
colcon build
```

### 7. Launch and Test

Start MoveIt:
```bash
source install/setup.bash
ros2 launch so_arm_moveit_config demo.launch.py
```

1. Start Isaac Sim simulation (press Play)
2. Control robot in RViz
3. Enjoy your SO-ARM MoveIt integration!

## Docker Setup

[Quentin Deyna](https://www.linkedin.com/in/quentindeyna/) is working on a Docker setup to solve "works-on-my-machine" problems:
- [Docker Repository](https://github.com/qdeyna/SO-ARM_MoveIt_IsaacSim)

## Credits

- Tutorial based on [robot mania's](https://www.youtube.com/@robotmania8896) excellent Isaac Sim + MoveIt guide
- SO-ARM robot by [TheRobotStudio](https://github.com/TheRobotStudio/SO-ARM100)
- Integration with [HuggingFace LeRobot](https://github.com/huggingface/lerobot)
- Video tutorial on [LycheeAI YouTube Channel](https://www.youtube.com/@LycheeAI)

## Repository Structure

```
├── src/
│   ├── so_arm_description/       # Robot URDF and meshes
│   ├── so_arm_moveit_config/     # MoveIt configuration
│   └── isaac_sim_usd/            # Isaac Sim USD files
├── README.md
└── .gitignore
```

## Web Teleoperation & Coordinate Frames

The repo includes nodes to teleoperate SO‑ARM101 from a web UI via WebSocket.  
This section documents the coordinate conventions and ROS topics involved.

### Web → ROS: Pose commands

- Node: `so_arm_motion_interface/websocket_pose_bridge_node.py`
- Default WebSocket URL (can be overridden via `WS_URL` in `scripts/run_interface_teleop.sh`):
  - `ws://cobot.center:8286/pang/ws/pub?channel=instant&name=so101&track=left_arm&mode=bundle`
- Incoming messages:
  - JSON pose bundles (position + quaternion) in a **web frame**.
  - The node normalizes several JSON formats into a single internal pose representation.
- Outgoing ROS message:
  - Topic: `/so_arm/pose_cmd` (`geometry_msgs/PoseStamped`)
  - Frame: `base` (parameter `reference_frame`, default `base`)

#### Position frame mapping

Let `(x_w, y_w, z_w)` be the position from the web client, and `(x_b, y_b, z_b)` the pose published in the robot `base` frame.  
The bridge applies this fixed mapping:

- `x_b = -z_w`
- `y_b = -x_w`
- `z_b =  y_w`

Intuitively:

- Web +X → robot −Y  
- Web +Y → robot +Z  
- Web +Z → robot −X

This matches the user’s camera view with the SO‑ARM base frame used in RViz/Isaac.

#### Orientation mapping

- Web sends orientation as quaternion `(x_w, y_w, z_w, w_w)` (or compatible layouts).
- The bridge multiplies by a fixed transform quaternion (parameter `transform_quaternion`, default `[0.5, 0.5, -0.5, -0.5]`, interpreted as `w,x,y,z`) to align the web frame with the robot `base` frame:
  - `q_base = q_transform ⊗ q_web`
- Optionally, when `use_web_z_as_yaw=true`, only the web Z‑axis rotation (yaw) is used and web roll/pitch are ignored before the transform.
- The resulting quaternion `q_base` is written directly into `PoseStamped.pose.orientation` on `/so_arm/pose_cmd`.

### Web → ROS: Joint jog shortcuts

To make some web gestures act as direct joint jogs (instead of full Cartesian pose control), the bridge also publishes `JointJog` commands:

- Base rotation:
  - Enabled when `map_web_z_to_rotation_joint=true` (default).
  - Uses the web Z value to send `JointJog` on:
    - `joint_names = ["Rotation"]`
    - `velocities = [rotation_joint_gain * web_z]`
  - Topic: `/servo_node/delta_joint_cmds`
- Wrist pitch (experimental, for gripper “nodding”):
  - Enabled when `map_web_pitch_to_wrist_joint=true`.
  - Uses the web pitch angle (after web→base alignment) to send `JointJog` on:
    - `joint_names = ["Wrist_Pitch"]`
    - `velocities = [wrist_pitch_joint_gain * pitch_web]`
  - Topic: `/servo_node/delta_joint_cmds`

These joint jogs are additive to MoveIt Servo’s Cartesian control and can be tuned via:

- Parameters in `websocket_pose_bridge_node.py`
- Environment variable `WRIST_PITCH_JOINT_GAIN` in `scripts/run_interface_teleop.sh`

### ROS → Web: Feedback streams

`scripts/run_interface_teleop.sh` also starts bridges for streaming robot state back to the web UI:

- Joint state bridge:
  - Node: `so_arm_motion_interface/websocket_joint_state_bridge_node.py`
  - Subscribes: `/joint_states`
  - Publishes joint arrays over WebSocket to the `robot_joint_states` track.
- Image bridge (overhead camera):
  - Node: `so_arm_motion_interface/websocket_image_bridge_node.py`
  - Subscribes: `/so_arm/overhead_camera/image_raw`
  - Encodes frames (JPEG or H.264, see script parameters) and sends binary WebSocket frames to the `head_camera` track.

All of these components are wired up in `scripts/run_interface_teleop.sh`, which is the recommended entry point for running MoveIt Servo + web teleop + Isaac Sim together.

## License

This project follows the same license as the original SO-ARM project. Please refer to the original repositories for licensing information. 
