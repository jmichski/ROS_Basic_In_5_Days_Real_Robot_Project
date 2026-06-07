# log_robot_lap

## What is this repository for?

**log_robot_lap** is a ROS Noetic action server and client package that autonomously drives a TurtleBot3 robot around the perimeter of a walled enclosure, records the odometry path of one complete lap, and returns the full position log to the action client on completion.

The package uses a wall-following laser scan algorithm to navigate, a Euclidean distance check against the starting position to detect lap completion, and the ROS actionlib framework to provide goal, feedback, and result messaging.

- **Version:** 1.0
- **License:** GNU General Public License (GPL)
- **Author:** Jeff "Mich" Michalski
- **Organization:** Airogistic

---

## Repository Structure

```
log_robot_lap/
├── action/
│   └── OdomRecord.action           # Action definition — goal (empty), result (Point32[] list_of_odoms), feedback (float32 current_total)
├── launch/
│   ├── Main.launch                 # Launches find_wall service + action server + action client
│   └── Main_Server.launch          # Launches find_wall service + action server only (use axclient to trigger)
├── src/
│   └── scripts/
│       ├── log_robot_lap_ac_srv.py # Action server — wall following, odometry recording, lap detection
│       └── log_robot_lap_client.py # Action client — sends lap goal, monitors feedback, handles result
├── CMakeLists.txt                  # Build configuration — action message generation
└── package.xml                     # Package dependencies — actionlib, geometry_msgs, nav_msgs
```

---

## Dependencies

| Package | Purpose |
|---|---|
| `rospy` | ROS Python client library |
| `actionlib` | Action server/client framework |
| `actionlib_msgs` | Base action message types |
| `geometry_msgs` | `Twist`, `Point32` message types |
| `nav_msgs` | `Odometry` message type |
| `sensor_msgs` | `LaserScan` message type |
| `find_wall` | Custom service — wall alignment before lap start |
| `message_generation` | Generates action messages at build time |

---

## Setup

**Step 1 — Clone into your catkin workspace:**
```bash
cd ~/simulation_ws/src
git clone https://github.com/jmichski/ROS_Basic_In_5_Days_Real_Robot_Project
Works on TheConstruct: ROSJect Project: ROS Basics in 5 Days Real Robot Project (Python)
```

**Step 2 — Build the package:**
```bash
cd ~/simulation_ws
catkin_make
source devel/setup.bash
```

**Step 3 — Verify action messages generated:**
```bash
rosmsg list | grep OdomRecord
```

Expected output:
```
log_robot_lap/OdomRecordAction
log_robot_lap/OdomRecordFeedback
log_robot_lap/OdomRecordGoal
log_robot_lap/OdomRecordResult
```

**Step 4 — Make scripts executable:**
```bash
chmod +x ~/simulation_ws/src/log_robot_lap/src/scripts/log_robot_lap_ac_srv.py
chmod +x ~/simulation_ws/src/log_robot_lap/src/scripts/log_robot_lap_client.py
```

---

## Configuration

### Turn Speed Parameter

The wall-following behaviour supports two turn speed modes controlled by the `turn_type` parameter:

| Value | Mode | Behaviour |
|---|---|---|
| `0` (default) | Slow turn | Robot stops forward motion during corners — more accurate wall tracking |
| `1` | Fast turn | Robot maintains forward velocity during corners — faster lap time |

The parameter is read on every scan callback so changes take effect immediately without restarting the node:
```python
turn_type = rospy.get_param('~turn_type', 0)
```

---

**Method 1 — Set default in the launch file:**

The `turn_type` argument is declared in the launch file with a default value:
```xml
<!-- ======== ARGUMENTS ======== -->
<arg name="turn_type" default="0"/>
```

It is passed to the server node as a private parameter:
```xml
<node pkg="log_robot_lap"
      type="log_robot_lap_ac_server.py"
      name="log_robot_lap_server_node"
      output="screen">
    <param name="turn_type" value="$(arg turn_type)"/>
</node>
```

---

**Method 2 — Override at launch time:**
```bash
# Launch with slow turns (default)
roslaunch log_robot_lap Main.launch turn_type:=0

# Launch with fast turns
roslaunch log_robot_lap Main.launch turn_type:=1
```

---

**Method 3 — Change live at runtime from terminal:**
```bash
# Switch to fast turns mid-lap
rosparam set /log_robot_lap_server_node/turn_type 1

# Switch back to slow turns
rosparam set /log_robot_lap_server_node/turn_type 0

# Check current value
rosparam get /log_robot_lap_server_node/turn_type
```

The change takes effect on the next scan message — no restart required.

### Lap Detection Parameters

| Constant | Default | Description |
|---|---|---|
| `LAP_CHECK_START_ODOMETRY_INDEX` | `60` | Minimum odometry readings before lap check activates |
| `LAP_CHECK_DISTANCE_DELTA` | `0.15` m | Distance threshold to start position that triggers lap complete |

### Max Lap Length (Client)

| Constant | Default | Description |
|---|---|---|
| `MAX_LAP_LENGTH` | `6.0` m | Client cancels goal if feedback distance exceeds this value |

---

## Find Wall Strategy

Before starting the lap the `find_wall` service aligns the robot so the nearest wall is to its right side. The wall detection algorithm uses a **multi-sample windowed scan with standard deviation filtering:**

- For each scan angle, multiple adjacent laser readings are averaged over a configurable window
- The **standard deviation** of each window is computed and compared against a rejection threshold (`SCAN_STD_WINDOW_REJECTION = 0.025`)
- Windows with high standard deviation are rejected — this filters out interior walls, furniture, and noisy returns that are not part of the outer perimeter
- The angle with the lowest averaged range and acceptable standard deviation is selected as the closest wall
- The robot rotates to place this wall at the forward index (360), drives to within `FORWARD_DISTANCE` of it, then rotates again to place the wall at the right index (180)

This strategy reliably locks onto the outer perimeter wall in symmetric environments where multiple walls are equidistant.

---

## Running the Project

The project requires multiple terminals running simultaneously. Open each terminal in order before proceeding to the next.

---

**Terminal 1 — Launch the simulation environment:**
```bash
source /home/user/simulation_ws/devel/setup.bash && roslaunch realrobotlab main.launch
```
This sources the workspace and launches the Gazebo simulation with the TurtleBot3 in the walled enclosure. Wait for Gazebo to fully load before proceeding.

---

**Terminal 2 — Launch teleoperation (optional — for manual positioning):**
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
Use the keyboard to manually drive the robot to a starting position if needed. Press `Ctrl+C` to stop teleop before launching the action server. Keys `i/j/l/,` control forward/left/right/back. Speed can be adjusted with `w/x`.

---

**Terminal 3 — Launch the action server:**
```bash
roslaunch log_robot_lap Main.launch
```
Launches the `find_wall` service node and the `log_robot_lap` action server node. The server waits for a goal before the robot begins moving. Use `Main_Server.launch` instead if you want to trigger the lap manually via axclient.

---

**Terminal 4 — Monitor live feedback (distance travelled):**
```bash
rostopic echo /record_lap/feedback
```
Streams live feedback from the action server showing `current_total` distance in metres as the robot travels the lap. Update rate matches the odometry callback frequency.

---

**Terminal 5 — Monitor lap result (odometry point list):**
```bash
rostopic echo /record_lap/result
```
Displays the full `list_of_odoms` Point32 array when the lap completes. Note this topic is latched — it will replay the last result to any new subscriber so verify the timestamp to confirm the result is fresh.

---

**Terminal 6 — Interactive goal control via axclient:**
```bash
rosrun actionlib_tools axclient.py /record_lap
```
Opens the actionlib GUI client. Use this when running `Main_Server.launch` for interactive debugging:
- Click in the **GOAL** text field and press **SPACE** before submitting — axclient requires at least one character in the goal field even when the goal has no parameters, otherwise the send will be rejected silently
- Click **SEND GOAL** to start a lap
- Watch **FEEDBACK** panel for live distance travelled in metres
- Click **CANCEL GOAL** to abort mid-lap and retrieve partial odometry
- View **RESULT** panel for the full odometry point list on completion

> ⚠️ **Important:** Always press **SPACE** in the goal field before clicking **SEND GOAL**. The `OdomRecord` goal has no parameters but axclient requires at least one character or the submission will be ignored.

---

**Terminal 7 — Change turn speed at runtime:**
```bash
rosparam set /log_robot_lap_server_node/turn_type 1
```
Switches the robot to fast turn mode instantly without restarting any nodes. Change takes effect on the next scan message. Switch back to slow turns with:
```bash
rosparam set /log_robot_lap_server_node/turn_type 0
```
Verify the current value with:
```bash
rosparam get /log_robot_lap_server_node/turn_type
```

---

## How to Run

### Option 1 — Server only + axclient (interactive debugging)

Launch the server and find_wall service:
```bash
roslaunch log_robot_lap Main_Server.launch
```

In a second terminal launch axclient:
```bash
rosrun actionlib_tools axclient.py /record_lap
```

In the axclient GUI:
1. Click in the **GOAL** text field and press **SPACE** — required even though the goal has no fields
2. Click **SEND GOAL** to start the lap
3. Watch **FEEDBACK** — `current_total` updates with distance travelled in metres
4. On lap completion **RESULT** shows the full `list_of_odoms` point array
5. Click **CANCEL GOAL** at any time to abort and retrieve partial odometry

> ⚠️ **axclient tip:** Always press SPACE in the goal field before clicking SEND GOAL or the submission will be ignored.

### Option 2 — Server + Client automated

Launch both server and client together:
```bash
roslaunch log_robot_lap Main.launch
```

The client will automatically:
- Connect to the action server
- Send a lap goal
- Monitor feedback and cancel if `MAX_LAP_LENGTH` is exceeded
- Log the result on completion

With fast turn type:
```bash
roslaunch log_robot_lap Main.launch turn_type:=1
```

---

## Action Interface

### Topics published by the action server

| Topic | Type | Description |
|---|---|---|
| `/record_lap/feedback` | `OdomRecordActionFeedback` | Distance travelled so far in metres |
| `/record_lap/result` | `OdomRecordActionResult` | Full list of Point32 odometry positions on lap complete |
| `/record_lap/status` | `GoalStatusArray` | Current goal status |

### Monitor from command line

```bash
# Live distance feedback
rostopic echo /record_lap/feedback

# Final lap result
rostopic echo /record_lap/result

# Robot velocity commands
rostopic echo /cmd_vel
```

---

## Running Tests

```bash
# Verify node is running
rosnode list | grep log_robot_lap

# Verify action server topics are active
rostopic list | grep record_lap

# Check action server is accepting goals
rosnode info /log_robot_lap_server_node
```

---

## Contribution Guidelines

- Follow PEP8 Python style conventions
- Add docstrings to all class methods
- Use `rospy.loginfo` for normal events, `rospy.logwarn` for cancellations, `rospy.logerr` for failures
- Test with both `turn_type:=0` and `turn_type:=1` before submitting changes
- All pull requests require code review before merge

---

## Acknowledgements

This work was developed as part of the **ROSject: ROS Basics in 5 Days Real Robot Project** offered by [The Construct](https://www.theconstruct.ai).

The Construct provides an outstanding ROS learning platform with browser-based simulation environments, structured courses, and real robot access. Their curriculum provided the foundational ROS concepts — topics, services, actions, and the TurtleBot3 simulation environment — that this project is built upon.

- **Course:** ROS Basics in 5 Days
- **Platform:** [The Construct](https://www.theconstruct.ai)
- **Robot:** TurtleBot3 (simulated via Gazebo / real robot)

---

## Who do I talk to?

- **Repository owner:** Jeff "Mich" Michalski — [michalski@airogistic.com](mailto:michalski@airogistic.com)
- **Organization:** [Airogistic](https://www.airogistic.com)
- **Project:** Drone Performance Harness / ROS Autonomous Navigation Research
