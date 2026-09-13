# 3-dof-bot

A ROS 2 workspace for a 3-DOF robotic arm: URDF/description, Gazebo/Ignition
simulation, `ros2_control`-based control, MoveIt motion planning, and a
remote/voice (Alexa) interface.

## Packages

| Package | Description |
|---|---|
| `arduinobot_bringup` | Top-level launch files that bring up the simulated robot. |
| `arduinobot_description` | URDF/xacro robot description, meshes, and `ros2_control` hardware interface. |
| `arduinobot_controller` | Launch files and configuration for spawning `ros2_control` controllers (joint state broadcaster, arm, gripper). |
| `arduinobot_moveit` | MoveIt configuration for motion planning. |
| `arduinobot_msgs` | Custom action/message definitions (e.g. `ArduinobotTask`). |
| `arduinobot_remote` | Remote control interfaces, including an Alexa skill backend and a task action server. |
| `arduinobot_cpp_examples` | Example C++ nodes. |

## Prerequisites

- ROS 2 (Humble or newer)
- Gazebo / Ignition Gazebo, matching your ROS distro
- MoveIt 2
- Python 3 with `pip`

## Setup

```bash
cd ~/arduinobot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Alexa remote interface

`arduinobot_remote` includes a Flask-based Alexa skill backend
(`alexa_interface.py`) that talks to the robot over a ROS 2 action client.
It requires an Alexa skill ID, which is kept out of source control via a
`.env` file.

1. Install the Python dependencies:
   ```bash
   pip install flask ask-sdk-core flask-ask-sdk ask-sdk-model python-dotenv
   ```
2. Copy the example env file at the workspace root and fill in your skill ID:
   ```bash
   cp .env.example .env
   # then edit .env and set ALEXA_SKILL_ID=amzn1.ask.skill.<your-id>
   ```
3. Launch the remote interface (from the workspace root, so `.env` is found):
   ```bash
   ros2 launch arduinobot_remote remote_interface.launch.py
   ```

## Launching the simulation

```bash
ros2 launch arduinobot_bringup simulated_robot.launch.py
```

This starts Gazebo/Ignition with the robot, the `ros2_control` controllers,
and (optionally) MoveIt and the remote interface, depending on the launch
arguments used.
