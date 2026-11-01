# 3-dof-bot

A ROS 2 workspace for a 3-DOF robotic arm: URDF/description, Gazebo/Ignition
simulation, `ros2_control`-based control, MoveIt motion planning, and a
remote/voice (Alexa) interface.

## Why this exists

I started this as a straightforward "learn ROS2 by building a 3-DOF arm"
project — URDF, `ros2_control`, MoveIt, the usual pipeline. Once the arm could
move on command from Alexa, though, the 3 hardcoded poses it responded to
started bugging me. "Pick," "sleep," "wake" — fine for a demo, but it's not
actually *doing* anything with the world around it, just replaying fixed
joint angles on cue.

So I added a camera, taught it to find colored blocks, and wired that into
the same voice interface — the arm now picks up whichever block you actually
ask for, wherever it currently is, instead of reaching for a memorized spot.
Getting there took a lot more wrong turns than I expected going in: a camera
sensor that turned out to look sideways instead of down, a MoveIt planner
that happily planned the arm straight through a table because nobody ever
told it the table existed, a TCP point I had to remeasure three times because
the gripper's fingertip gap moves as the fingers close. All of that (and the
reasoning behind each fix, not just the fix itself) is written up in
[`notes.md`](notes.md) — if you're trying to learn from this repo rather than
just run it, that's the file to read.

## Packages

| Package | Description |
|---|---|
| `arduinobot_bringup` | Top-level launch files that bring up the simulated robot. |
| `arduinobot_description` | URDF/xacro robot description, meshes, Gazebo sensors (cameras), and `ros2_control` hardware interface. |
| `arduinobot_controller` | Launch files and configuration for spawning `ros2_control` controllers (joint state broadcaster, arm, gripper). |
| `arduinobot_moveit` | MoveIt configuration for motion planning. |
| `arduinobot_msgs` | Custom action/message definitions (e.g. `ArduinobotTask`, now carrying a target XYZ for dynamic picks). |
| `arduinobot_remote` | Remote control interfaces: the Alexa skill backend, the task action server (fixed poses + dynamic pick sequence), and the text command interpreter. |
| `arduinobot_vision` | Camera-based color detection — finds the red/blue blocks and publishes their real-world positions. |
| `arduinobot_cpp_examples` | Example C++ nodes. |

## Prerequisites

- ROS 2 (Humble or newer — tested on Jazzy)
- Gazebo / Ignition Gazebo, matching your ROS distro
- MoveIt 2
- Python 3 with `pip`
- `python3-opencv` and `cv_bridge` (usually already pulled in by `rosdep
  install` below — needed by `arduinobot_vision` for color detection)
- [ngrok](https://ngrok.com/) (only needed for the Alexa remote interface)
- An Alexa developer account + skill configured in the [Alexa Developer Console](https://developer.amazon.com/alexa/console/ask) (only needed for the Alexa remote interface)

On modern Debian/Ubuntu, system `pip` refuses to install packages globally
("externally-managed-environment" / PEP 668). Every `pip install` below
needs `--break-system-packages` appended, e.g.:
```bash
pip3 install flask --break-system-packages
```

## Setup

```bash
cd ~/Desktop/Bharath/coding/3-dof-bot
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### Alexa remote interface

`arduinobot_remote` includes a Flask-based Alexa skill backend
(`alexa_interface.py`) that talks to the robot over a ROS 2 action client.
It requires an Alexa skill ID, which is kept out of source control via a
`.env` file, and a public HTTPS tunnel (ngrok) so Amazon's servers can reach
your local Flask server.

1. **Install the Python dependencies.** Flask/ask-sdk alone isn't enough —
   `ask-sdk-webservice-support` pulls in `certvalidator` → `oscrypto` for
   request-signature verification, and the version of `oscrypto` on PyPI
   (1.3.0) doesn't support OpenSSL 3.x, which every modern distro ships.
   Install it from a patched commit instead of letting pip pull the broken
   PyPI release:
   ```bash
   pip3 install flask flask-ask-sdk ask-sdk-core ask-sdk-model python-dotenv --break-system-packages
   pip3 uninstall -y oscrypto --break-system-packages
   pip3 install --break-system-packages git+https://github.com/wbond/oscrypto.git@1547f535001ba568b239b8797465536759c742a3
   ```
   If you skip the `oscrypto` patch, `alexa_interface.py` will crash on
   every request with `oscrypto.errors.LibraryNotFoundError: Error
   detecting the version of libcrypto` even though it starts up fine.

2. **Create a `.env` file at the workspace root** (same directory as this
   README) with your skill ID from the Alexa Developer Console
   (**Build** tab, or the URL of the console itself):
   ```bash
   echo "ALEXA_SKILL_ID=amzn1.ask.skill.<your-id>" > .env
   ```
   `load_dotenv()` searches upward from the current working directory, so
   this only gets picked up if you `ros2 launch` from the workspace root.
   `.env` is already in `.gitignore`.

3. **Start ngrok**, pointed at port 5000 (Flask's default):
   ```bash
   ngrok http 5000
   ```
   Keep this terminal open. Copy the `https://...ngrok-free.dev` (or
   similar) forwarding URL it prints.

4. **Set the skill's endpoint** in the Alexa Developer Console
   (**Build → Endpoint**) to that ngrok HTTPS URL, with `/` as the path.
   Save it.

5. **Launch the full stack** (see below) — `remote_interface.launch.py`
   (included in `simulated_robot.launch.py`) starts `alexa_interface.py`
   as a ROS node, which runs Flask on `127.0.0.1:5000`.

## Vision-grounded pick & place

The robot isn't limited to the 3 fixed poses anymore. There's a table with a
red and a blue block in front of it; a camera watches the workspace from
above, `arduinobot_vision`'s `color_detector` node finds the blocks, and
`arduinobot_remote`'s `command_interpreter` node turns a plain-text (or
voice) command into an actual pick-and-place sequence.

- **How it finds the blocks**: HSV color thresholding on the overhead camera
  feed, then a calibrated pixel-to-world mapping (measured by hand, not
  computed analytically — see `notes.md` for why that turned out to matter).
- **How it decides what you asked for**: a small keyword/synonym matcher, on
  purpose not a paid LLM call — Claude Pro (the chat subscription) and the
  Anthropic API are separate products with separate billing, and I didn't
  want every test run to cost money. The parsing is isolated into one
  function (`extract_target_color` in `command_interpreter.py`), so swapping
  in a real LLM later is a small, contained change if I ever want it.
- **How it actually picks something up**: `task_server` runs a 5-stage
  sequence — open gripper, move above the block, descend, close, lift —
  using MoveIt Cartesian IK against the gripper's actual fingertip frame, not
  a single blind move to the block's coordinates (an early version did that,
  and it just rammed the block instead of grasping it).

Once the full stack is running (see below), try it with no Alexa setup needed
at all:
```bash
ros2 topic pub --once /voice_command std_msgs/msg/String "data: 'pick up the red one'"
```
Or, with the Alexa skill wired up, just ask it: "pick up the blue one," "grab
the red block," etc. — the skill now has a `PickColorIntent` with a `color`
slot (built on `AMAZON.Color`) feeding into the exact same text pipeline.

## Launching everything (recommended order)

Use three terminals:

**Terminal 1 — ngrok** (only if testing the Alexa interface):
```bash
ngrok http 5000
```

**Terminal 2 — the ROS 2 stack:**
```bash
cd ~/Desktop/Bharath/coding/3-dof-bot
source install/setup.bash
ros2 launch arduinobot_bringup simulated_robot.launch.py
```
This brings up Gazebo/Ignition with the robot (cameras, table, and blocks
included in the world), the `ros2_control` controllers, MoveIt + RViz, and
the remote interface — task server, Alexa Flask backend, color detector, and
command interpreter — all in one launch. Nothing extra to run by hand.

Watch this terminal's output for a few seconds after launch:
- `[alexa_interface.py-N]` should **not** print a traceback and should
  stay running. If it dies immediately, it's almost always a missing
  Python package — see "Alexa remote interface" above.
- If you ever restart this launch after changing Python dependencies
  (e.g. reinstalling `oscrypto`), the *old* process needs to actually be
  killed and relaunched — a `pip install` doesn't affect code already
  loaded in a running process.

**Terminal 3 — test it:**
- For motion planning: use the RViz "Motion Planning" panel that comes up
  automatically (Plan, then Execute).
- For the Alexa skill: either the **Alexa Simulator** in the Developer
  Console (**Test** tab, must be enabled for "Development"), or from the
  terminal with the ASK CLI (`npm install -g ask-cli` → `sudo` if you hit
  `EACCES`, then `ask configure`):
  ```bash
  ask dialog --skill-id amzn1.ask.skill.<your-id> --locale en-US --stage development
  ```
  Note: with `ask dialog` (and the real device/Alexa app), you must open
  the skill by its **invocation name** first (`open <invocation name>`)
  before any bare intent utterance like "grab the pen" will resolve — this
  is normal session behavior, not a bug.

## Troubleshooting (things that have actually gone wrong)

- **MoveIt execution fails with `Invalid Trajectory: start point deviates
  from current robot state more than 0.01`** — the `controller_manager`'s
  `update_rate` in `arduinobot_controller/config/arduinobot_controller.yaml`
  was too low (10 Hz). `/joint_states` wasn't being published often enough
  for MoveIt's execution-state check to keep up. Fixed by raising it to
  `update_rate: 100`.

- **RViz warning: `planning_scene_monitor: Maybe failed to update robot
  state, time diff: <huge number>`** — the `rviz_node` in
  `arduinobot_moveit/launch/moveit.launch.py` wasn't getting
  `use_sim_time`, so its internal clock ran on wall time while Gazebo's
  `/clock` and `/joint_states` were stamped with sim time. Fixed by adding
  `{"use_sim_time": is_sim}` to the rviz node's parameters alongside
  `move_group`.

- **Alexa simulator says "There was a problem with the requested skill's
  response" / ngrok shows `502 Bad Gateway`** — nothing is listening on
  port 5000. Either `ros2 launch` isn't running at all, or
  `alexa_interface.py` crashed on startup. Check Terminal 2's output for a
  traceback right after `[alexa_interface.py-N]: process started`.

- **`ModuleNotFoundError: No module named 'flask'`** — the Python
  environment `ros2 launch` uses doesn't have the Alexa dependencies
  installed. Install with `--break-system-packages` (see above).

- **`oscrypto.errors.LibraryNotFoundError: Error detecting the version of
  libcrypto`** — PyPI's `oscrypto` (1.3.0) doesn't support OpenSSL 3.x.
  Reinstall from the patched git commit (see above), **then restart
  `ros2 launch`** — a currently-running process won't pick up a package
  you just reinstalled.

- **A trained utterance (e.g. "grab the pen") still triggers
  `AMAZON.FallbackIntent`** — the interaction model was edited/saved but
  never *rebuilt*. In the console: **Build → Interaction Model → Intents**,
  check "Last successful build" at the top. Click **Build Model** (not
  just Save) and wait for it to finish before retesting.

- **Can't type into the Alexa Simulator's input box** — known flaky
  browser UI. Try disabling browser extensions (VPN/ad-block extensions
  especially), hard-refresh (Ctrl+Shift+R), or fall back to testing via
  the ASK CLI (`ask dialog`, see above) instead of the web simulator.

- **`ask dialog` says "This utterance did not resolve to any intent in
  your skill"** for an utterance that works fine in the browser — you
  need to open the skill with its invocation name first in that CLI
  session (`open <invocation name>`); bare intent utterances only resolve
  inside an active session.

- **The arm "successfully" plans a path straight through the table** — no
  collision error, nothing — because MoveIt's planner only avoids obstacles
  it actually knows about (its own "planning scene"), which is completely
  separate from whatever Gazebo is physically simulating. A table that only
  exists in the SDF world is invisible to MoveIt. Fix: register it as a
  `moveit_msgs::msg::CollisionObject` via `PlanningSceneInterface` (done once,
  in `task_server`'s constructor) with the same size/pose as the real table.

- **A Gazebo camera points sideways instead of where you aimed it** — a
  camera sensor's "forward" is its link's local **+X** axis by default, not
  +Z like the ROS "optical frame" convention you might expect. `rpy="0 0 0"`
  on the mounting joint leaves it pointed along +X, which is sideways
  relative to most robots. Check the actual aim with the sensor's
  `visualize:true` frustum cone in the Gazebo GUI — don't just trust the
  rotation math, a sign error here is easy to make and easy to miss.

- **The gripper's grasp point drifts between measurements** — if you're
  using RViz's "Publish Point" tool to measure a point near the fingertips
  (e.g. for a TCP frame), remember the fingers *rotate*, they don't slide —
  the visual gap between them shifts with the gripper's joint angle. Measure
  at the angle the gripper will actually be at during a real grasp, zoom in
  close before clicking, and expect to redo it at least once.
