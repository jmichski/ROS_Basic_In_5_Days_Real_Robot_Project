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

# ── Scan indexes — real robot: angle_min=0, 450 pts, CW ──
# ── Simulation:   angle_min=-π, 720 pts, CCW             ──
if SIMULATION:
    RIGHT_SCAN_INDEX   = 540   # sim —  90 deg right  (angle_min=-π, 720 pts)
    FORWARD_SCAN_INDEX = 360   # sim —   0 deg forward
    LEFT_SCAN_INDEX    = 180   # sim — -90 deg left
else:
    RIGHT_SCAN_INDEX   = 112   # real —  90 deg right  (angle_min=0, 450 pts)
    FORWARD_SCAN_INDEX = 0     # real —   0 deg forward
    LEFT_SCAN_INDEX    = 336   # real — 270 deg left

SCAN_AVERAGE_WINDOW = 10

# ── Object avoidance distances ────────────────────────
if SIMULATION:
    FAST_FORWARD_OBJECT_AVOID_DISTANCE = 0.53  # sim
    SLOW_FORWARD_OBJECT_AVOID_DISTANCE = 0.49  # sim
else:
    FAST_FORWARD_OBJECT_AVOID_DISTANCE = 0.65  # real
    SLOW_FORWARD_OBJECT_AVOID_DISTANCE = 0.60  # real

RIGHT_OBJECT_AVOID_DISTANCE = 0.22
RIGHT_INNER_AVOID_DISTANCE  = 0.26
LEFT_OBJECT_AVOID_DISTANCE  = 0.1

# ── Velocity and turn tuning ──────────────────────────
FORWARD_LINEAR_VEL            = 0.15   # safe for TheConstruct real robot lab
FORWARD_CORRECTION_LINEAR_VEL = 0.12
FAST_TURN_FORWARD_LINEAR_VEL  = 0.15   # safe for TheConstruct real robot lab
SLOW_TURN_FORWARD_LINEAR_VEL  = 0.0

if SIMULATION:
    TURN_ANGULAR_VEL     = 0.82   # sim
    TURN_CORRECTION_GAIN = 0.19   # sim Kp
    TURN_DERIVATIVE_GAIN = 0.00   # Kd — derivative gain (same for sim and real — tune if needed)
else:
    TURN_ANGULAR_VEL     = 0.30   # real
    TURN_CORRECTION_GAIN = 0.15   # real Kp
    TURN_DERIVATIVE_GAIN = 0.05   # Kd — derivative gain (same for sim and real — tune if needed)

WALL_DISTANCE_MIN = 0.22
WALL_DISTANCE_MAX = 0.26

WALL_TRACK_CENTER   = (WALL_DISTANCE_MAX + WALL_DISTANCE_MIN) / 2
WALL_DISTANCE_RANGE = WALL_DISTANCE_MAX - WALL_DISTANCE_MIN

LAP_CHECK_START_ODOMETRY_INDEX = 60
LAP_CHECK_DISTANCE_DELTA       = 0.12
LAP_CHECK_MIN_DISTANCE         = 0.5   # minimum distance travelled before lap complete check fires

CMD_VEL_PUBLISH_RATE = 50.0   # Hz — continuous publish rate for real robot
LAP_TIMEOUT_SECONDS  = 60.0   # hard backstop — abort lap if not completed in time
LIDAR_RANGE_MIN      = 0.12   # LDS-01 minimum reliable range — filter zeros and floor noise


class LogLapClass(object):
    _odom_record_feedback = OdomRecordRealFeedback()
    _odom_record_result   = OdomRecordRealResult()
    _scan_index           = 0
    _scan_count           = 0      # counts scan_callback firings — independent of odom
    _odometry_running     = False
    _odometry_index       = 0
    _odometry_readings    = []
    _lap_completed        = False
    _shutdown_called      = False  # guard against double shutdown_handler calls
    _turn_type            = 0      # cached param — set once in execute_lap()
    _running_total        = 0.0    # incremental odometry total — avoids O(n) recompute

    def __init__(self):
        """Initialise the action server class and log startup."""
        self._latest_cmd            = Twist()   # instance-level — not shared across instances
        self._prev_wall_track_error = 0.0       # previous error for derivative term
        self._prev_scan_time        = None      # timestamp of previous scan for dt calculation
        rospy.loginfo('Log Real Robot Lap - Odom Record Action Server: Initialized ...')

    def shutdown_handler(self):
        """Stop the robot on ROS shutdown or goal cancel — guarded against double calls."""
        if self._shutdown_called:
            return
        self._shutdown_called = True
        rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: Shutting Down ...")
        self.robot_stop()

    def goal_callback(self, goal):
        """Reset all state and execute a new lap when action goal is received."""
        rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: Goal Received — Executing Wall Alignment for Lap ...")
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
        self._prev_wall_track_error = 0.0   # reset PD state for new lap
        self._prev_scan_time        = None
        self.execute_lap()

    def odometry_callback(self, msg):
        """Receive odometry messages and record position — logs every 10th reading."""
        if self._odometry_running:
            self._scan_index += 1
            if self._scan_index % 10 == 0:
                rospy.loginfo('Log Real Robot Lap - Odom Record Action Server: Odom Recording — reading %d', self._scan_index)
            self.record_odometry(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            )

    def scan_callback(self, laserscan_data):
        """Compute wall-following velocity command with PD correction and store in _latest_cmd.
        The cmd_vel timer publishes it at CMD_VEL_PUBLISH_RATE Hz."""
        move_cmd  = Twist()
        turn_type = self._turn_type
        self._scan_count += 1

        right_range   = self.averaged_range(laserscan_data.ranges, RIGHT_SCAN_INDEX)
        forward_range = self.averaged_range(laserscan_data.ranges, FORWARD_SCAN_INDEX)
        left_range    = self.averaged_range(laserscan_data.ranges, LEFT_SCAN_INDEX)

        wall_track_error            = right_range - WALL_TRACK_CENTER
        normalized_wall_track_error = wall_track_error / WALL_DISTANCE_RANGE

        # derivative term — rate of change of wall tracking error between scans
        now = rospy.Time.now()
        if self._prev_scan_time is not None:
            dt = (now - self._prev_scan_time).to_sec()
            if dt > 0.0:
                d_error = (wall_track_error - self._prev_wall_track_error) / dt
            else:
                d_error = 0.0
        else:
            d_error = 0.0
        self._prev_wall_track_error = wall_track_error
        self._prev_scan_time        = now

        normalized_d_error   = d_error / WALL_DISTANCE_RANGE
        wall_turn_correction = -1.0 * (TURN_CORRECTION_GAIN * normalized_wall_track_error +
                                       TURN_DERIVATIVE_GAIN  * normalized_d_error)

        if turn_type == 0:
            forward_object_avoid_distance = SLOW_FORWARD_OBJECT_AVOID_DISTANCE
        else:
            forward_object_avoid_distance = FAST_FORWARD_OBJECT_AVOID_DISTANCE

        # ── movement decision ─────────────────────────────
        if (not math.isinf(forward_range) and forward_range < forward_object_avoid_distance):
            action = 'LEFT_TURN'
            if turn_type == 0:
                move_cmd.linear.x  =  SLOW_TURN_FORWARD_LINEAR_VEL
            else:
                move_cmd.linear.x  =  FAST_TURN_FORWARD_LINEAR_VEL
            move_cmd.angular.z =  TURN_ANGULAR_VEL
        elif math.isinf(forward_range):
            action = 'FWD_INF_CORRECTION'
            move_cmd.linear.x  = -1 * FORWARD_CORRECTION_LINEAR_VEL
            move_cmd.angular.z =  TURN_ANGULAR_VEL
        elif (not math.isinf(right_range) and right_range < RIGHT_OBJECT_AVOID_DISTANCE):
            action = 'WALL_CORRECT_LEFT'
            move_cmd.linear.x  =  FORWARD_CORRECTION_LINEAR_VEL
            move_cmd.angular.z =  wall_turn_correction
        elif (not math.isinf(left_range) and left_range < LEFT_OBJECT_AVOID_DISTANCE) or \
             (right_range > RIGHT_INNER_AVOID_DISTANCE):
            action = 'INNER_CORRECT_RIGHT'
            move_cmd.linear.x  =  FORWARD_CORRECTION_LINEAR_VEL
            move_cmd.angular.z =  wall_turn_correction
        else:
            action = 'FORWARD'
            move_cmd.linear.x  =  FORWARD_LINEAR_VEL
            move_cmd.angular.z =  0

        # ── scan log — every scan at INFO so visible without DEBUG launch ──
        rospy.loginfo(
            'Lap sc:%d  R:%.3f F:%.3f L:%.3f  err:%.3f d_err:%.4f  corr:%.3f  '
            'action:%-20s  lin:%.3f ang:%.3f  tt:%d',
            self._scan_count,
            right_range, forward_range, left_range,
            wall_track_error, d_error, wall_turn_correction,
            action,
            move_cmd.linear.x, move_cmd.angular.z,
            turn_type
        )

        self._latest_cmd = move_cmd   # store — timer will publish at 50 Hz

    def cmd_vel_timer_callback(self, event):
        """Publish the latest stored cmd_vel at CMD_VEL_PUBLISH_RATE Hz."""
        pub.publish(self._latest_cmd)

    def reset_odometry(self):
        """Clear the odometry readings list and running total for a fresh lap."""
        self._odometry_readings = []
        self._running_total     = 0.0

    def record_odometry(self, x, y, z):
        """Append a new Point32 position and update feedback and lap check."""
        point   = Point32()
        point.x = x
        point.y = y
        point.z = z
        self._odometry_readings.append(point)
        self.get_odometry_total()
        self.check_lap_complete()

    def get_odometry_total(self):
        """Add only the latest segment to running total — O(1) not O(n)."""
        if len(self._odometry_readings) >= 2:
            prev = self._odometry_readings[-2]
            curr = self._odometry_readings[-1]
            dx = curr.x - prev.x
            dy = curr.y - prev.y
            self._running_total += math.sqrt(dx**2 + dy**2)
        self._odom_record_feedback.current_total = self._running_total
        server.publish_feedback(self._odom_record_feedback)
        return self._running_total

    def check_lap_complete(self):
        """Check if robot has returned within LAP_CHECK_DISTANCE_DELTA of start position."""
        if len(self._odometry_readings) < LAP_CHECK_START_ODOMETRY_INDEX:
            return
        if self._running_total < LAP_CHECK_MIN_DISTANCE:
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
            rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: Lap Completed")

    def execute_lap(self):
        """Call find_wall_real_robot service, start wall following and wait for lap completion."""

        rate = rospy.Rate(10)
        rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: Waiting for clock...")
        while rospy.Time.now() == rospy.Time(0):
            rate.sleep()
        rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: Clock ready")

        if self._odometry_index == 0:
            rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: Lap Started ...")
        rate = rospy.Rate(10)

        self._turn_type = rospy.get_param('~turn_type', 0)
        rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: turn_type = %d", self._turn_type)

        rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: Calling find_wall_real_robot service...")
        try:
            rospy.wait_for_service('find_wall_real_robot', timeout=10.0)
        except rospy.ROSException:
            rospy.logerr("Log Real Robot Lap - Odom Record Action Server: find_wall_real_robot service not available after 10 s — aborting!")
            server.set_aborted()
            return
        find_wall_client = rospy.ServiceProxy('find_wall_real_robot', FindWallReal)
        response         = find_wall_client()

        if not response.wallfound:
            rospy.logerr("Log Real Robot Lap - Odom Record Action Server: Find_Wall Failed Error!")
            server.set_aborted()
            return

        rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: Find_Wall Complete OK")

        self.reset_odometry()
        self._odometry_running = True
        lap_start_time         = rospy.Time.now()

        scan_sub  = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        cmd_timer = rospy.Timer(
            rospy.Duration(1.0 / CMD_VEL_PUBLISH_RATE),
            self.cmd_vel_timer_callback
        )
        rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: cmd_vel timer started at %.0f Hz", CMD_VEL_PUBLISH_RATE)

        while not rospy.is_shutdown() and not self._lap_completed:

            if server.is_preempt_requested():
                rospy.logwarn("Log Real Robot Lap - Odom Record Action Server: Goal Cancelled!")
                self._odometry_running = False
                cmd_timer.shutdown()
                scan_sub.unregister()
                self.shutdown_handler()
                self._odom_record_result.list_of_odoms = self._odometry_readings
                server.set_preempted(self._odom_record_result)
                return

            elapsed = (rospy.Time.now() - lap_start_time).to_sec()
            if elapsed > LAP_TIMEOUT_SECONDS:
                rospy.logerr("Log Real Robot Lap - Odom Record Action Server: Lap timeout (%.0f s) — aborting!", LAP_TIMEOUT_SECONDS)
                self._odometry_running = False
                cmd_timer.shutdown()
                scan_sub.unregister()
                self.robot_stop()
                self._odom_record_result.list_of_odoms = self._odometry_readings
                server.set_aborted(self._odom_record_result)
                return

            rate.sleep()

        cmd_timer.shutdown()
        scan_sub.unregister()
        self.robot_stop()
        self._odom_record_result.list_of_odoms = self._odometry_readings
        server.set_succeeded(self._odom_record_result)
        rospy.loginfo('Log Real Robot Lap - Odom Record Action Server: Robot Travelled %.3f m', self._odom_record_feedback.current_total)
        rospy.loginfo('Log Real Robot Lap - Odom Record Action Server: Logged %d odometry readings', len(self._odom_record_result.list_of_odoms))

    def averaged_range(self, ranges, index, window=SCAN_AVERAGE_WINDOW):
        """Return averaged laser range at index — filters zeros, floor, inf and NaN (real LDS-01)."""
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
            return float('inf')
        mean     = sum(samples) / len(samples)
        stddev   = math.sqrt(sum((x - mean)**2 for x in samples) / len(samples))
        filtered = [x for x in samples if abs(x - mean) <= 2.0 * stddev]
        return sum(filtered) / len(filtered) if filtered else float('inf')

    def robot_stop(self):
        """Zero _latest_cmd so timer publishes stop, then send explicit stop and reset counters."""
        rospy.loginfo('Log Real Robot Lap - Odom Record Action Server: Robot stopping ...')
        self._latest_cmd       = Twist()
        stop_cmd               = Twist()
        stop_cmd.linear.x      = 0.0
        stop_cmd.angular.z     = 0.0
        pub.publish(stop_cmd)
        rospy.Rate(2).sleep()
        self._scan_index     = 0
        self._odometry_index = 0


rospy.init_node('log_real_robot_lap_server_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

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
rospy.loginfo("Log Real Robot Lap - Odom Record Action Server: Launched")
rospy.spin()