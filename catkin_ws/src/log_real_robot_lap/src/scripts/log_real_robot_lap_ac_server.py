#!/usr/bin/env python3

import rospy
import math
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from find_wall_real_robot.srv import FindWallReal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from log_real_robot_lap.msg import OdomRecordRealAction, OdomRecordRealGoal, OdomRecordRealResult, OdomRecordRealFeedback

# ══════════════════════════════════════════════════════
# SIMULATION FLAG  —  set 1 for sim, 0 for real robot
# ══════════════════════════════════════════════════════
SIMULATION = 1

# For CLAUDE >> ADD THESE NEW CONSTANTS
LOG_TUNING_CONSTANTS = 0
LOG_FUNCTION_CALLS = 0
LOG_FUNCTION_RETURNS = 0
LOG_FUNCTION_DATA = 0
LOG_TWISTS = 1
LOG_SCAN_DISTANCES = 1
LOG_ODOM_DISANCES = 0
LOG_LOGIC_STATES = 1
LOG_FINAL_RESULTS = 1
LOG_AI_DEBUGINFO = 0
LOG_DEV_DEBUGINFO = 0

# ── NODE / CLASS constants — used in every structured log line so the
# pipe-delimited output imports into Excel/CSV with consistent columns,
# matching log_real_robot_lap_server and find_wall_real_robot_svc
# (NODE | CLASS | FUNC | TAG). ───────────────────────────────────────
LOG_NODE  = "log_real_robot_lap_server"
LOG_CLASS = "Log Lap Server"


# ── Scan indexes — real robot: angle_min=0, 450 pts, CW ──
# ── Simulation:   angle_min=-π, 720 pts, CCW             ──
if SIMULATION:
    RIGHT_SCAN_INDEX   = 180   # sim —  90 deg right  (angle_min=-π, 720 pts)
    FORWARD_SCAN_INDEX = 360   # sim —   0 deg forward
    LEFT_SCAN_INDEX    = 540   # sim — -90 deg left
else:
    RIGHT_SCAN_INDEX   = 112   # real —  90 deg right  (angle_min=0, 450 pts)
    FORWARD_SCAN_INDEX = 0     # real —   0 deg forward
    LEFT_SCAN_INDEX    = 336   # real — 270 deg left

if SIMULATION:
    SCAN_AVERAGE_WINDOW = 50
else:
    SCAN_AVERAGE_WINDOW = 10

if SIMULATION:
    # ── turn_type=1 (continuous turn) ────────────────────
    CONTINUOUS_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE = 0.57
    CONTINUOUS_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE = 0.53
    CONTINUOUS_TURN_FORWARD_LINEAR_VEL                 = 0.10
    CONTINUOUS_TURN_FORWARD_CORRECTION_LINEAR_VEL      = 0.18
    CONTINUOUS_TURN_FAST_TURN_FORWARD_LINEAR_VEL       = 0.22
    CONTINUOUS_TURN_SLOW_TURN_FORWARD_LINEAR_VEL       = 0.15
    CONTINUOUS_TURN_WALL_DISTANCE_MIN                  = 0.20
    CONTINUOUS_TURN_WALL_DISTANCE_MAX                  = 0.28

    # ── turn_type=0 (discrete turn) ───────────────────────
    DISCRETE_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE   = 0.57
    DISCRETE_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE   = 0.53
    DISCRETE_TURN_FORWARD_LINEAR_VEL                   = 0.22
    DISCRETE_TURN_FORWARD_CORRECTION_LINEAR_VEL        = 0.19
    DISCRETE_TURN_FAST_TURN_FORWARD_LINEAR_VEL         = 0.15
    DISCRETE_TURN_SLOW_TURN_FORWARD_LINEAR_VEL         = 0.0
    DISCRETE_TURN_WALL_DISTANCE_MIN                    = 0.20
    DISCRETE_TURN_WALL_DISTANCE_MAX                    = 0.26
else:
    # ── turn_type=1 (continuous turn) ────────────────────
    CONTINUOUS_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE = 0.35
    CONTINUOUS_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE = 0.30
    CONTINUOUS_TURN_FORWARD_LINEAR_VEL                 = 0.15
    CONTINUOUS_TURN_FORWARD_CORRECTION_LINEAR_VEL      = 0.12
    CONTINUOUS_TURN_FAST_TURN_FORWARD_LINEAR_VEL       = 0.15
    CONTINUOUS_TURN_SLOW_TURN_FORWARD_LINEAR_VEL       = 0.10
    CONTINUOUS_TURN_WALL_DISTANCE_MIN                  = 0.20
    CONTINUOUS_TURN_WALL_DISTANCE_MAX                  = 0.28

    # ── turn_type=0 (discrete turn) ───────────────────────
    DISCRETE_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE   = 0.35
    DISCRETE_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE   = 0.30
    DISCRETE_TURN_FORWARD_LINEAR_VEL                   = 0.15
    DISCRETE_TURN_FORWARD_CORRECTION_LINEAR_VEL        = 0.12
    DISCRETE_TURN_FAST_TURN_FORWARD_LINEAR_VEL         = 0.15
    DISCRETE_TURN_SLOW_TURN_FORWARD_LINEAR_VEL         = 0.10
    DISCRETE_TURN_WALL_DISTANCE_MIN                    = 0.20
    DISCRETE_TURN_WALL_DISTANCE_MAX                    = 0.26

RIGHT_OBJECT_AVOID_DISTANCE = 0.20
LEFT_OBJECT_AVOID_DISTANCE  = 0.1

# RIGHT_INNER_AVOID_DISTANCE — threshold above which INNER_CORRECT_RIGHT fires.
# Real: raised from 0.30 to 0.50 — prevents violent overcorrection when R is large
# (e.g. at lap start or after a corner). Sim: original 0.30 preserved.
if SIMULATION:
    RIGHT_INNER_AVOID_DISTANCE = 0.30   # sim — original value
else:
    RIGHT_INNER_AVOID_DISTANCE = 0.40   # real — raised; INNER_CORRECT_RIGHT fires only
                                        # when right gap genuinely exceeds 0.50 m

# ── PD correction cap ─────────────────────────────────
# Limits wall-tracking correction to ±CORRECTION_MAX rad/s regardless of
# how far the right wall is — prevents spinning when R is large open space
if SIMULATION:
    CORRECTION_MAX = 1.00   # sim — wider cap, sim physics forgiving
else:
    CORRECTION_MAX = 0.25   # real — tight cap, prevents overcorrection spin

if SIMULATION:
    TURN_ANGULAR_VEL     = 0.88   # corner turn speed
    TURN_CORRECTION_GAIN = 0.15   # Kp — proportional tracking gain
    TURN_DERIVATIVE_GAIN = 0.12   # Kd — derivative tracking gain
else:
    TURN_ANGULAR_VEL     = 0.15   # corner turn speed
    TURN_CORRECTION_GAIN = 0.10   # Kp — proportional tracking gain
    TURN_DERIVATIVE_GAIN = 0.05   # Kd — derivative tracking gain

LAP_CHECK_START_ODOMETRY_INDEX = 60
LAP_CHECK_DISTANCE_DELTA       = 0.12
LAP_CHECK_MIN_DISTANCE         = 0.5

CMD_VEL_PUBLISH_RATE = 50.0   # Hz — continuous publish rate for real robot
LAP_TIMEOUT_SECONDS  = 60.0   # hard backstop — abort lap if not completed in time
LIDAR_RANGE_MIN      = 0.12   # LDS-01 minimum reliable range — filter zeros and floor noise

# ── PD gain ramp-in ───────────────────────────────────
# Holds PD gains at zero for the first GAIN_RAMP_SCANS scans then ramps linearly
# to full gain over the next GAIN_RAMP_SCANS scans. Prevents violent initial
# correction when robot starts misaligned or wall reading is noisy on startup.
# Set to 0 to disable (gains active from scan 1).
# Sim: disabled — sim physics are clean and PD can be active immediately.
# Real: 10 scans zero + 10 scans ramp = ~2 s at 10 Hz before full PD authority.
if SIMULATION:
    GAIN_RAMP_SCANS = 0       # sim — no ramp needed, gains active from scan 1
else:
    GAIN_RAMP_SCANS = 15       # real — disables PD correction for full lap;
                               # robot drives straight, turns at corners only


def log_constants():
    """Structured constant dump — gated by LOG_TUNING_CONSTANTS, mirrors
    find_wall_real_robot's log_constants() format for consistent grepping."""
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CALL", LOG_NODE)
    if not LOG_TUNING_CONSTANTS:
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | FUNC: log_constants() | RETURN: (disabled)", LOG_NODE)
        return
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: SIMULATION;                          %d", LOG_NODE, SIMULATION)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: RIGHT_SCAN_INDEX;                    %d", LOG_NODE, RIGHT_SCAN_INDEX)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: FORWARD_SCAN_INDEX;                  %d", LOG_NODE, FORWARD_SCAN_INDEX)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: LEFT_SCAN_INDEX;                     %d", LOG_NODE, LEFT_SCAN_INDEX)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: SCAN_AVERAGE_WINDOW;                 %d", LOG_NODE, SCAN_AVERAGE_WINDOW)

    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CONTINUOUS_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE; %.2f", LOG_NODE, CONTINUOUS_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CONTINUOUS_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE; %.2f", LOG_NODE, CONTINUOUS_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CONTINUOUS_TURN_FORWARD_LINEAR_VEL;                 %.2f", LOG_NODE, CONTINUOUS_TURN_FORWARD_LINEAR_VEL)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CONTINUOUS_TURN_FORWARD_CORRECTION_LINEAR_VEL;      %.2f", LOG_NODE, CONTINUOUS_TURN_FORWARD_CORRECTION_LINEAR_VEL)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CONTINUOUS_TURN_FAST_TURN_FORWARD_LINEAR_VEL;       %.2f", LOG_NODE, CONTINUOUS_TURN_FAST_TURN_FORWARD_LINEAR_VEL)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CONTINUOUS_TURN_SLOW_TURN_FORWARD_LINEAR_VEL;       %.2f", LOG_NODE, CONTINUOUS_TURN_SLOW_TURN_FORWARD_LINEAR_VEL)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CONTINUOUS_TURN_WALL_DISTANCE_MIN;                  %.2f", LOG_NODE, CONTINUOUS_TURN_WALL_DISTANCE_MIN)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CONTINUOUS_TURN_WALL_DISTANCE_MAX;                  %.2f", LOG_NODE, CONTINUOUS_TURN_WALL_DISTANCE_MAX)

    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: DISCRETE_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE;   %.2f", LOG_NODE, DISCRETE_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: DISCRETE_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE;   %.2f", LOG_NODE, DISCRETE_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: DISCRETE_TURN_FORWARD_LINEAR_VEL;                   %.2f", LOG_NODE, DISCRETE_TURN_FORWARD_LINEAR_VEL)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: DISCRETE_TURN_FORWARD_CORRECTION_LINEAR_VEL;        %.2f", LOG_NODE, DISCRETE_TURN_FORWARD_CORRECTION_LINEAR_VEL)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: DISCRETE_TURN_FAST_TURN_FORWARD_LINEAR_VEL;         %.2f", LOG_NODE, DISCRETE_TURN_FAST_TURN_FORWARD_LINEAR_VEL)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: DISCRETE_TURN_SLOW_TURN_FORWARD_LINEAR_VEL;         %.2f", LOG_NODE, DISCRETE_TURN_SLOW_TURN_FORWARD_LINEAR_VEL)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: DISCRETE_TURN_WALL_DISTANCE_MIN;                    %.2f", LOG_NODE, DISCRETE_TURN_WALL_DISTANCE_MIN)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: DISCRETE_TURN_WALL_DISTANCE_MAX;                    %.2f", LOG_NODE, DISCRETE_TURN_WALL_DISTANCE_MAX)

    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: RIGHT_OBJECT_AVOID_DISTANCE;         %.2f", LOG_NODE, RIGHT_OBJECT_AVOID_DISTANCE)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: LEFT_OBJECT_AVOID_DISTANCE;          %.2f", LOG_NODE, LEFT_OBJECT_AVOID_DISTANCE)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: RIGHT_INNER_AVOID_DISTANCE;          %.2f", LOG_NODE, RIGHT_INNER_AVOID_DISTANCE)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CORRECTION_MAX;                      %.2f", LOG_NODE, CORRECTION_MAX)

    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: TURN_ANGULAR_VEL;                    %.2f", LOG_NODE, TURN_ANGULAR_VEL)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: TURN_CORRECTION_GAIN;                %.2f", LOG_NODE, TURN_CORRECTION_GAIN)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: TURN_DERIVATIVE_GAIN;                %.2f", LOG_NODE, TURN_DERIVATIVE_GAIN)

    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: LAP_CHECK_START_ODOMETRY_INDEX;      %d", LOG_NODE, LAP_CHECK_START_ODOMETRY_INDEX)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: LAP_CHECK_DISTANCE_DELTA;            %.2f", LOG_NODE, LAP_CHECK_DISTANCE_DELTA)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: LAP_CHECK_MIN_DISTANCE;               %.2f", LOG_NODE, LAP_CHECK_MIN_DISTANCE)

    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: CMD_VEL_PUBLISH_RATE;                %.1f", LOG_NODE, CMD_VEL_PUBLISH_RATE)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: LAP_TIMEOUT_SECONDS;                 %.1f", LOG_NODE, LAP_TIMEOUT_SECONDS)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: LIDAR_RANGE_MIN;                     %.2f", LOG_NODE, LIDAR_RANGE_MIN)
    rospy.loginfo(" | NODE: %s | FUNC: log_constants() | CONST: GAIN_RAMP_SCANS;                     %d", LOG_NODE, GAIN_RAMP_SCANS)
    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | FUNC: log_constants() | RETURN: (dump complete)", LOG_NODE)


class LogLapClass(object):
    _odom_record_feedback = OdomRecordRealFeedback()
    _odom_record_result   = OdomRecordRealResult()
    _scan_index           = 0
    _scan_count           = 0      # counts scan_callback firings — independent of odom
    _odometry_running     = False
    _odometry_index       = 0
    _odometry_readings    = []
    _lap_completed        = False
    _running_total        = 0.0

    def __init__(self):
        """Initialise the action server class — turn_type constants set in execute_lap."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: __init__() | CALL", LOG_NODE, LOG_CLASS)
        self._turn_type                    = 0
        self._prev_wall_track_error        = 0.0    # previous error for derivative term
        self._prev_scan_time               = None   # timestamp of previous scan for dt
        self._prev_action                  = None   # last movement action — used to log only on state transition
        self._latest_cmd                   = Twist()
        self._shutdown_called              = False
        self._fast_avoid                   = DISCRETE_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE
        self._slow_avoid                   = DISCRETE_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE
        self._forward_linear_vel           = DISCRETE_TURN_FORWARD_LINEAR_VEL
        self._forward_correction_linear_vel= DISCRETE_TURN_FORWARD_CORRECTION_LINEAR_VEL
        self._fast_turn_forward_linear_vel = DISCRETE_TURN_FAST_TURN_FORWARD_LINEAR_VEL
        self._slow_turn_forward_linear_vel = DISCRETE_TURN_SLOW_TURN_FORWARD_LINEAR_VEL
        self._wall_distance_min            = DISCRETE_TURN_WALL_DISTANCE_MIN
        self._wall_distance_max            = DISCRETE_TURN_WALL_DISTANCE_MAX
        self._wall_track_center            = (DISCRETE_TURN_WALL_DISTANCE_MAX + DISCRETE_TURN_WALL_DISTANCE_MIN) / 2
        self._wall_distance_range          = DISCRETE_TURN_WALL_DISTANCE_MAX - DISCRETE_TURN_WALL_DISTANCE_MIN
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: __init__() | DEV: Initialized", LOG_NODE, LOG_CLASS)
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: __init__() | RETURN: (none)", LOG_NODE, LOG_CLASS)

    def _apply_turn_type_constants(self):
        """Select the correct set of tuning constants based on turn_type param."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: _apply_turn_type_constants() | CALL", LOG_NODE, LOG_CLASS)
        if self._turn_type == 1:
            self._fast_avoid                    = CONTINUOUS_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE
            self._slow_avoid                    = CONTINUOUS_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE
            self._forward_linear_vel            = CONTINUOUS_TURN_FORWARD_LINEAR_VEL
            self._forward_correction_linear_vel = CONTINUOUS_TURN_FORWARD_CORRECTION_LINEAR_VEL
            self._fast_turn_forward_linear_vel  = CONTINUOUS_TURN_FAST_TURN_FORWARD_LINEAR_VEL
            self._slow_turn_forward_linear_vel  = CONTINUOUS_TURN_SLOW_TURN_FORWARD_LINEAR_VEL
            self._wall_distance_min             = CONTINUOUS_TURN_WALL_DISTANCE_MIN
            self._wall_distance_max             = CONTINUOUS_TURN_WALL_DISTANCE_MAX
        else:
            self._fast_avoid                    = DISCRETE_TURN_FAST_FORWARD_OBJECT_AVOID_DISTANCE
            self._slow_avoid                    = DISCRETE_TURN_SLOW_FORWARD_OBJECT_AVOID_DISTANCE
            self._forward_linear_vel            = DISCRETE_TURN_FORWARD_LINEAR_VEL
            self._forward_correction_linear_vel = DISCRETE_TURN_FORWARD_CORRECTION_LINEAR_VEL
            self._fast_turn_forward_linear_vel  = DISCRETE_TURN_FAST_TURN_FORWARD_LINEAR_VEL
            self._slow_turn_forward_linear_vel  = DISCRETE_TURN_SLOW_TURN_FORWARD_LINEAR_VEL
            self._wall_distance_min             = DISCRETE_TURN_WALL_DISTANCE_MIN
            self._wall_distance_max             = DISCRETE_TURN_WALL_DISTANCE_MAX
        self._wall_track_center   = (self._wall_distance_max + self._wall_distance_min) / 2
        self._wall_distance_range = self._wall_distance_max - self._wall_distance_min
        if LOG_AI_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: _apply_turn_type_constants() | AI: turn_type; %d fast_avoid; %.2f slow_avoid; %.2f fwd_vel; %.2f wall_band; %.2f-%.2f correction_max; %.2f inner_avoid; %.2f", LOG_NODE, LOG_CLASS,
                          self._turn_type, self._fast_avoid, self._slow_avoid,
                          self._forward_linear_vel, self._wall_distance_min, self._wall_distance_max,
                          CORRECTION_MAX, RIGHT_INNER_AVOID_DISTANCE)
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: _apply_turn_type_constants() | RETURN: wall_track_center; %.3f wall_distance_range; %.3f", LOG_NODE, LOG_CLASS,
                          self._wall_track_center, self._wall_distance_range)

    def shutdown_handler(self):
        """Stop the robot on ROS shutdown or goal cancel — guarded against double calls."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: shutdown_handler() | CALL", LOG_NODE, LOG_CLASS)
        if self._shutdown_called:
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: shutdown_handler() | RETURN: (already called, skipped)", LOG_NODE, LOG_CLASS)
            return
        self._shutdown_called = True
        if LOG_LOGIC_STATES:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: shutdown_handler() | STATE: SHUTTING_DOWN", LOG_NODE, LOG_CLASS)
        self.robot_stop()
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: shutdown_handler() | RETURN: (shutdown complete)", LOG_NODE, LOG_CLASS)

    def goal_callback(self, goal):
        """Reset all state and execute a new lap when action goal is received."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: goal_callback() | CALL", LOG_NODE, LOG_CLASS)
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: goal_callback() | DEV: Goal Received - Executing Wall Alignment for Lap", LOG_NODE, LOG_CLASS)
        self._odom_record_feedback.current_total = 0
        server.publish_feedback(self._odom_record_feedback)
        self._odom_record_result.list_of_odoms = []
        self._lap_completed         = False
        self._odometry_running      = False
        self._odometry_readings     = []
        self._running_total         = 0.0
        self._scan_index            = 0
        self._scan_count            = 0
        self._shutdown_called       = False
        self._latest_cmd            = Twist()
        self._prev_wall_track_error = 0.0
        self._prev_scan_time        = None
        self._prev_action           = None
        self.execute_lap()
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: goal_callback() | RETURN: (none)", LOG_NODE, LOG_CLASS)

    def odometry_callback(self, msg):
        """Receive odometry messages and record position — logs every 10th reading."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: odometry_callback() | CALL", LOG_NODE, LOG_CLASS)
        if self._odometry_running:
            self._scan_index += 1
            if self._scan_index % 10 == 0 and LOG_ODOM_DISANCES:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: odometry_callback() | ODOM: reading; %d running_total; %.3f", LOG_NODE, LOG_CLASS,
                              self._scan_index, self._running_total)
            self.record_odometry(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            )
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: odometry_callback() | RETURN: (none)", LOG_NODE, LOG_CLASS)

    def scan_callback(self, laserscan_data):
        """Compute wall-following velocity command with PD correction and store in _latest_cmd.
        The cmd_vel timer publishes it at CMD_VEL_PUBLISH_RATE Hz."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: scan_callback() | CALL", LOG_NODE, LOG_CLASS)
        move_cmd = Twist()
        self._scan_count += 1

        right_range   = self.averaged_range(laserscan_data.ranges, RIGHT_SCAN_INDEX)
        forward_range = self.averaged_range(laserscan_data.ranges, FORWARD_SCAN_INDEX)
        left_range    = self.averaged_range(laserscan_data.ranges, LEFT_SCAN_INDEX)

        wall_track_error            = right_range - self._wall_track_center
        normalized_wall_track_error = wall_track_error / self._wall_distance_range

        # guard — if right scan is inf (dropout) skip PD and hold last good command
        if math.isinf(right_range):
            rospy.logwarn(" | NODE: %s | CLASS: %s | FUNC: scan_callback() | WARN: scan_count; %d right_range; inf - scan dropout, holding last cmd", LOG_NODE, LOG_CLASS,
                          self._scan_count)
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: scan_callback() | RETURN: (scan dropout, cmd held)", LOG_NODE, LOG_CLASS)
            return

        # derivative term — rate of change of wall tracking error between scans
        # On the first scan _prev_scan_time is None — d_error is forced to 0.0 and
        # _prev_wall_track_error is seeded from the actual reading so that scan 2
        # sees a real delta rather than a jump from the initialised 0.0 value.
        # This zeros the derivative gain on lap start regardless of starting position.
        now = rospy.Time.now()
        if self._prev_scan_time is not None:
            dt = (now - self._prev_scan_time).to_sec()
            d_error = (wall_track_error - self._prev_wall_track_error) / dt if dt > 0.0 else 0.0
        else:
            d_error = 0.0   # first scan — derivative undefined, force zero
        self._prev_wall_track_error = wall_track_error   # seed from actual reading
        self._prev_scan_time        = now

        normalized_d_error = d_error / self._wall_distance_range

        # ── PD gain ramp-in ────────────────────────────────
        # Scale gains from 0 to 1 over the ramp window so the robot eases into
        # wall tracking rather than spiking on the first scan.
        # gain_scale = 0 for first GAIN_RAMP_SCANS scans,
        #              ramps 0→1 linearly over the next GAIN_RAMP_SCANS scans,
        #              = 1 (full gain) thereafter.
        if GAIN_RAMP_SCANS > 0 and self._scan_count <= 2 * GAIN_RAMP_SCANS:
            if self._scan_count <= GAIN_RAMP_SCANS:
                gain_scale = 0.0
            else:
                gain_scale = (self._scan_count - GAIN_RAMP_SCANS) / float(GAIN_RAMP_SCANS)
        else:
            gain_scale = 1.0

        raw_correction = -1.0 * gain_scale * (TURN_CORRECTION_GAIN * normalized_wall_track_error +
                                               TURN_DERIVATIVE_GAIN  * normalized_d_error)

        # ── PD correction cap ──────────────────────────────
        # Clamps correction to ±CORRECTION_MAX rad/s — prevents violent overcorrection
        # when the right wall is far away (e.g. at lap start, inner track, or after a corner)
        wall_turn_correction = max(-CORRECTION_MAX, min(CORRECTION_MAX, raw_correction))

        if self._turn_type == 0:
            forward_object_avoid_distance = self._slow_avoid
        else:
            forward_object_avoid_distance = self._fast_avoid

        if (not math.isinf(forward_range) and forward_range < forward_object_avoid_distance):
            action = 'LEFT_TURN'
            if self._turn_type == 0:
                move_cmd.linear.x  =  self._slow_turn_forward_linear_vel
            else:
                move_cmd.linear.x  =  self._fast_turn_forward_linear_vel
            move_cmd.angular.z =  TURN_ANGULAR_VEL
        elif math.isinf(forward_range):
            action = 'FWD_INF_CORRECTION'
            move_cmd.linear.x  = -1 * self._forward_correction_linear_vel
            move_cmd.angular.z =  TURN_ANGULAR_VEL
        elif (not math.isinf(right_range) and right_range < RIGHT_OBJECT_AVOID_DISTANCE):
            action = 'WALL_CORRECT_LEFT'
            move_cmd.linear.x  =  self._forward_correction_linear_vel
            move_cmd.angular.z =  wall_turn_correction
        elif (not math.isinf(left_range) and left_range < LEFT_OBJECT_AVOID_DISTANCE) or \
             (right_range > RIGHT_INNER_AVOID_DISTANCE):
            action = 'INNER_CORRECT_RIGHT'
            move_cmd.linear.x  =  self._forward_correction_linear_vel
            move_cmd.angular.z =  wall_turn_correction
        else:
            action = 'FORWARD'
            move_cmd.linear.x  =  self._forward_linear_vel
            move_cmd.angular.z =  0

        # ── movement state transition logging ─────────────
        # Fires only when action changes from the previous scan — gives a clean
        # state-machine trace (FORWARD -> LEFT_TURN -> FORWARD, etc.) without
        # flooding the log every scan like LOG_SCAN_DISTANCES/LOG_TWISTS do.
        if LOG_LOGIC_STATES and action != self._prev_action:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: scan_callback() | STATE: %s -> %s", LOG_NODE, LOG_CLASS,
                          self._prev_action, action)
        self._prev_action = action

        # ── scan distance logging ─────────────────────────
        # Suppressed during LEFT_TURN and FWD_INF_CORRECTION — only logs while
        # the robot is actively wall-following/approaching (FORWARD, WALL_CORRECT_LEFT,
        # INNER_CORRECT_RIGHT) so corner turns don't flood the log with distance noise.
        if LOG_SCAN_DISTANCES and action not in ('LEFT_TURN', 'FWD_INF_CORRECTION'):
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: scan_callback() | SCAN: action; %s right; %.3f forward; %.3f left; %.3f", LOG_NODE, LOG_CLASS,
                          action, right_range, forward_range, left_range)

        if LOG_AI_DEBUGINFO:
            rospy.loginfo(
                " | NODE: %s | CLASS: %s | FUNC: scan_callback() | AI: sc; %d R; %.3f F; %.3f L; %.3f "
                "err; %.3f d_err; %.4f gs; %.2f raw; %.3f corr; %.3f action; %s lin; %.3f ang; %.3f tt; %d",
                LOG_NODE, LOG_CLASS,
                self._scan_count,
                right_range, forward_range, left_range,
                wall_track_error, d_error, gain_scale, raw_correction, wall_turn_correction,
                action,
                move_cmd.linear.x, move_cmd.angular.z,
                self._turn_type
            )

        if LOG_TWISTS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: scan_callback() | TWIST: action; %s linear.x; %.3f angular.z; %.3f", LOG_NODE, LOG_CLASS,
                          action, move_cmd.linear.x, move_cmd.angular.z)

        self._latest_cmd = move_cmd   # store — timer will publish at CMD_VEL_PUBLISH_RATE Hz
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: scan_callback() | RETURN: action; %s lin; %.3f ang; %.3f", LOG_NODE, LOG_CLASS,
                          action, move_cmd.linear.x, move_cmd.angular.z)

    def cmd_vel_timer_callback(self, event):
        """Publish the latest stored cmd_vel at CMD_VEL_PUBLISH_RATE Hz."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: cmd_vel_timer_callback() | CALL", LOG_NODE, LOG_CLASS)
        if not rospy.is_shutdown():
            pub.publish(self._latest_cmd)
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: cmd_vel_timer_callback() | RETURN: (none)", LOG_NODE, LOG_CLASS)

    def reset_odometry(self):
        """Clear the odometry readings list and running total for a fresh lap."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: reset_odometry() | CALL", LOG_NODE, LOG_CLASS)
        self._odometry_readings = []
        self._running_total     = 0.0
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: reset_odometry() | RETURN: (none)", LOG_NODE, LOG_CLASS)

    def record_odometry(self, x, y, z):
        """Append a new Point32 position and update feedback and lap check."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: record_odometry() | CALL", LOG_NODE, LOG_CLASS)
        point   = Point32()
        point.x = x
        point.y = y
        point.z = z
        self._odometry_readings.append(point)
        self.get_odometry_total()
        self.check_lap_complete()
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: record_odometry() | RETURN: (none)", LOG_NODE, LOG_CLASS)

    def get_odometry_total(self):
        """Add only the latest segment to running total — O(1) not O(n)."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: get_odometry_total() | CALL", LOG_NODE, LOG_CLASS)
        if len(self._odometry_readings) >= 2:
            prev = self._odometry_readings[-2]
            curr = self._odometry_readings[-1]
            dx = curr.x - prev.x
            dy = curr.y - prev.y
            segment_delta = math.sqrt(dx**2 + dy**2)
            self._running_total += segment_delta
            if LOG_ODOM_DISANCES:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: get_odometry_total() | ODOM: segment_delta; %.4f running_total; %.3f", LOG_NODE, LOG_CLASS,
                              segment_delta, self._running_total)
        self._odom_record_feedback.current_total = self._running_total
        server.publish_feedback(self._odom_record_feedback)
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: get_odometry_total() | RETURN: running_total; %.3f", LOG_NODE, LOG_CLASS, self._running_total)
        return self._running_total

    def check_lap_complete(self):
        """Check if robot has returned within LAP_CHECK_DISTANCE_DELTA of start position."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: check_lap_complete() | CALL", LOG_NODE, LOG_CLASS)
        if len(self._odometry_readings) < LAP_CHECK_START_ODOMETRY_INDEX:
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: check_lap_complete() | RETURN: (not enough readings yet)", LOG_NODE, LOG_CLASS)
            return
        if self._running_total < LAP_CHECK_MIN_DISTANCE:
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: check_lap_complete() | RETURN: (running_total below min distance)", LOG_NODE, LOG_CLASS)
            return
        start    = self._odometry_readings[0]
        last     = self._odometry_readings[-1]
        distance = math.sqrt(
            (last.x - start.x)**2 +
            (last.y - start.y)**2 +
            (last.z - start.z)**2
        )
        if distance < LAP_CHECK_DISTANCE_DELTA:
            self._lap_completed    = True
            self._odometry_running = False
            if LOG_LOGIC_STATES:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: check_lap_complete() | STATE: LAP_COMPLETED distance_to_start; %.3f", LOG_NODE, LOG_CLASS,
                              distance)
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: check_lap_complete() | RETURN: distance; %.3f lap_completed; %s", LOG_NODE, LOG_CLASS,
                          distance, self._lap_completed)

    def execute_lap(self):
        """Call find_wall_real_robot service, start wall following and wait for lap completion."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | CALL", LOG_NODE, LOG_CLASS)
        rate = rospy.Rate(10)
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | DEV: Waiting for clock", LOG_NODE, LOG_CLASS)
        while rospy.Time.now() == rospy.Time(0):
            rate.sleep()
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | DEV: Clock ready", LOG_NODE, LOG_CLASS)

        if self._odometry_index == 0 and LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | DEV: Lap Started", LOG_NODE, LOG_CLASS)
        rate = rospy.Rate(10)

        # fetch turn_type and apply matching constant set
        self._turn_type = rospy.get_param('~turn_type', 0)
        self._apply_turn_type_constants()
        log_constants()

        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | DEV: Calling find_wall_real_robot service", LOG_NODE, LOG_CLASS)
        try:
            rospy.wait_for_service('find_wall_real_robot', timeout=10.0)
        except rospy.ROSException:
            rospy.logerr(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | ERROR: find_wall_real_robot service not available after 10s - aborting", LOG_NODE, LOG_CLASS)
            server.set_aborted()
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | RETURN: (aborted, service unavailable)", LOG_NODE, LOG_CLASS)
            return
        find_wall_client = rospy.ServiceProxy('find_wall_real_robot', FindWallReal)
        response         = find_wall_client()

        if not response.wallfound:
            rospy.logerr(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | ERROR: Find_Wall Failed", LOG_NODE, LOG_CLASS)
            server.set_aborted()
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | RETURN: (aborted, wall not found)", LOG_NODE, LOG_CLASS)
            return

        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | DEV: Find_Wall Complete OK", LOG_NODE, LOG_CLASS)

        # ── wait for find_wall cmd_vel timer to fully stop ───
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | DEV: Waiting 5s for find_wall cmd_vel to clear", LOG_NODE, LOG_CLASS)
        rospy.sleep(5.0)

        # register cmd_vel publisher now — find_wall has unregistered its publisher
        global pub
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | DEV: cmd_vel publisher registered, starting lap cmd_vel timer", LOG_NODE, LOG_CLASS)

        self.reset_odometry()
        self._odometry_running = True
        lap_start_time         = rospy.Time.now()

        scan_sub  = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        cmd_timer = rospy.Timer(
            rospy.Duration(1.0 / CMD_VEL_PUBLISH_RATE),
            self.cmd_vel_timer_callback
        )
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | DEV: cmd_vel timer started at %.0f Hz", LOG_NODE, LOG_CLASS, CMD_VEL_PUBLISH_RATE)

        while not rospy.is_shutdown() and not self._lap_completed:

            if server.is_preempt_requested():
                rospy.logwarn(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | WARN: Goal Cancelled", LOG_NODE, LOG_CLASS)
                self._odometry_running = False
                cmd_timer.shutdown()
                scan_sub.unregister()
                self.shutdown_handler()
                self._odom_record_result.list_of_odoms = self._odometry_readings
                server.set_preempted(self._odom_record_result)
                if LOG_FUNCTION_RETURNS:
                    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | RETURN: (preempted, goal cancelled)", LOG_NODE, LOG_CLASS)
                return

            elapsed = (rospy.Time.now() - lap_start_time).to_sec()
            if elapsed > LAP_TIMEOUT_SECONDS:
                rospy.logerr(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | ERROR: Lap timeout; %.0fs - aborting", LOG_NODE, LOG_CLASS, LAP_TIMEOUT_SECONDS)
                self._odometry_running = False
                cmd_timer.shutdown()
                scan_sub.unregister()
                self.robot_stop()
                self._odom_record_result.list_of_odoms = self._odometry_readings
                server.set_aborted(self._odom_record_result)
                if LOG_FUNCTION_RETURNS:
                    rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | RETURN: (aborted, lap timeout)", LOG_NODE, LOG_CLASS)
                return

            rate.sleep()

        cmd_timer.shutdown()
        scan_sub.unregister()
        self.robot_stop()
        self._odom_record_result.list_of_odoms = self._odometry_readings
        server.set_succeeded(self._odom_record_result)
        if LOG_FINAL_RESULTS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | FINAL_RESULT: total_distance; %.3f readings_logged; %d", LOG_NODE, LOG_CLASS,
                          self._odom_record_feedback.current_total, len(self._odom_record_result.list_of_odoms))
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: execute_lap() | RETURN: (lap succeeded, distance; %.3f)", LOG_NODE, LOG_CLASS,
                          self._odom_record_feedback.current_total)

    def averaged_range(self, ranges, index, window=SCAN_AVERAGE_WINDOW):
        """Return averaged laser range at index — filters zeros, floor, inf and NaN (real LDS-01)."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: averaged_range() | CALL", LOG_NODE, LOG_CLASS)
        samples   = []
        inf_count = 0
        for i in range(index - window, index + window + 1):
            val = ranges[i % len(ranges)]
            if math.isinf(val):
                inf_count += 1
            elif math.isnan(val):
                pass
            elif val < LIDAR_RANGE_MIN:
                pass
            else:
                samples.append(val)
        if inf_count >= 4 or not samples:
            if LOG_FUNCTION_RETURNS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: averaged_range() | RETURN: inf (inf_count; %d samples; %d)", LOG_NODE, LOG_CLASS, inf_count, len(samples))
            return float('inf')
        mean     = sum(samples) / len(samples)
        stddev   = math.sqrt(sum((x - mean)**2 for x in samples) / len(samples))
        filtered = [x for x in samples if abs(x - mean) <= 2.0 * stddev]
        result   = sum(filtered) / len(filtered) if filtered else float('inf')
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: averaged_range() | RETURN: mean; %.3f stddev; %.3f result; %.3f", LOG_NODE, LOG_CLASS, mean, stddev, result)
        return result

    def robot_stop(self):
        """Zero _latest_cmd so timer publishes stop, then send explicit stop and reset counters."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: robot_stop() | CALL", LOG_NODE, LOG_CLASS)
        if LOG_LOGIC_STATES:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: robot_stop() | STATE: %s -> ROBOT_STOPPING", LOG_NODE, LOG_CLASS, self._prev_action)
        self._latest_cmd       = Twist()
        if LOG_TWISTS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: robot_stop() | TWIST: _latest_cmd zeroed linear.x; %.3f angular.z; %.3f", LOG_NODE, LOG_CLASS,
                          self._latest_cmd.linear.x, self._latest_cmd.angular.z)
        stop_cmd               = Twist()
        stop_cmd.linear.x      = 0.0
        stop_cmd.angular.z     = 0.0
        if LOG_TWISTS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: robot_stop() | TWIST: stop_cmd linear.x; %.3f angular.z; %.3f", LOG_NODE, LOG_CLASS,
                          stop_cmd.linear.x, stop_cmd.angular.z)
        if pub is not None:
            pub.publish(stop_cmd)
        rospy.Rate(2).sleep()
        self._scan_index     = 0
        self._odometry_index = 0
        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: robot_stop() | RETURN: (none)", LOG_NODE, LOG_CLASS)


rospy.init_node('log_real_robot_lap_server_node')
pub = None   # registered in execute_lap after find_wall unregisters its cmd_vel publisher

log_lap_object = LogLapClass()

rospy.Subscriber('/odom', Odometry, log_lap_object.odometry_callback)
rospy.on_shutdown(log_lap_object.shutdown_handler)

server = actionlib.SimpleActionServer(
    'record_lap_real',
    OdomRecordRealAction,
    execute_cb=log_lap_object.goal_callback,
    auto_start=False
)

server.start()
if LOG_DEV_DEBUGINFO:
    rospy.loginfo(" | NODE: %s | FUNC: __main__ | DEV: Launched", LOG_NODE)
rospy.spin()