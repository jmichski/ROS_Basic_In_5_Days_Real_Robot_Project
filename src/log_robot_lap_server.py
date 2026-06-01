#!/usr/bin/env python3

import rospy
import math
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from find_wall.srv import FindWall
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
from log_robot_lap.msg import OdomRecordAction, OdomRecordGoal, OdomRecordResult, OdomRecordFeedback

RIGHT_SCAN_INDEX = 180
FORWARD_SCAN_INDEX = 360
LEFT_SCAN_INDEX = 540
SCAN_AVERAGE_WINDOW = 10

FAST_FORWARD_OBJECT_AVOID_DISTANCE = 0.57
SLOW_FORWARD_OBJECT_AVOID_DISTANCE = 0.53

RIGHT_OBJECT_AVOID_DISTANCE = 0.22
RIGHT_INNER_AVOID_DISTANCE = 0.26
LEFT_OBJECT_AVOID_DISTANCE = 0.1

FORWARD_LINEAR_VEL = 0.22
FORWARD_CORRECTION_LINEAR_VEL = 0.19

FAST_TURN_FORWARD_LINEAR_VEL = 0.22
SLOW_TURN_FORWARD_LINEAR_VEL = 0.0

TURN_ANGULAR_VEL = 0.82
TURN_CORRECTION_GAIN = 0.19

WALL_DISTANCE_MIN = 0.2
WALL_DISTANCE_MAX = 0.3

WALL_TRACK_CENTER  = (WALL_DISTANCE_MAX + WALL_DISTANCE_MIN) / 2
WALL_DISTANCE_RANGE = WALL_DISTANCE_MAX - WALL_DISTANCE_MIN

LAP_CHECK_START_ODOMETRY_INDEX = 60
LAP_CHECK_DISTANCE_DELTA       = 0.12

class LogLapClass(object):
    _odom_record_feedback = OdomRecordFeedback()
    _odom_record_result   = OdomRecordResult()
    _scan_index           = 0
    _odometry_running     = False
    _odometry_index       = 0
    _odometry_readings    = []
    _lap_completed        = False

    def __init__(self):
        """Initialise the action server class and log startup."""
        rospy.loginfo('Log Robot Lap - Odom Record Action Server: Initialized ...')

    def shutdown_handler(self):
        """Stop the robot on ROS shutdown or goal cancel."""
        rospy.loginfo("Log Robot Lap - Odom Record Action Server: Shutting Down ...")
        self.robot_stop()

    def goal_callback(self, goal):
        """Reset all state and execute a new lap when action goal is received."""
        rospy.loginfo("Log Robot Lap - Odom Record Action Server:  Goal Received: Executing Wall Alignment for Lap ...")
        self._odom_record_feedback.current_total = 0
        server.publish_feedback(self._odom_record_feedback)
        self._odom_record_result.list_of_odoms =  []
        self._lap_completed     = False
        self._odometry_running  = False
        self._odometry_readings = []
        self._scan_index        = 0    # reset odom recording state counter
        self.execute_lap()

    def odometry_callback(self, msg):
        """Receive odometry messages and record position — logs every 10th reading."""
        if self._odometry_running:
            self._scan_index += 1                          # increment on every reading
            if self._scan_index % 10 == 0:                 # log every 10th reading
                rospy.loginfo('Log Robot Lap - Odom Record Action Server: Odom Recording — reading %d', self._scan_index)
            self.record_odometry(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            )

    def scan_callback(self, laserscan_data):
        """Process laser scan data and publish wall-following velocity commands."""
        move_cmd  = Twist()
        turn_type = rospy.get_param('~turn_type', 0)

        right_range   = self.averaged_range(laserscan_data.ranges, RIGHT_SCAN_INDEX)
        forward_range = self.averaged_range(laserscan_data.ranges, FORWARD_SCAN_INDEX)
        left_range    = self.averaged_range(laserscan_data.ranges, LEFT_SCAN_INDEX)

        #print(f"Scan Data: {self._scan_index}")
        #print(f"Right Wall Range: {right_range}")
        #print(f"Forward Wall Range: {forward_range}")
        #print(f"Left Wall Range: {left_range}")

        wall_track_error            = right_range - WALL_TRACK_CENTER
        normalized_wall_track_error = wall_track_error / WALL_DISTANCE_RANGE
        wall_turn_correction        = -1 * normalized_wall_track_error * TURN_CORRECTION_GAIN

        #print(f"Wall Track Error: {wall_track_error}")
        #print(f"Normalized Wall Track Error: {normalized_wall_track_error}")
        #print(f"Wall Turn Correction Velocity: {wall_turn_correction}")

        if turn_type == 0:
            forward_object_avoid_distance = SLOW_FORWARD_OBJECT_AVOID_DISTANCE
        else:
            forward_object_avoid_distance = FAST_FORWARD_OBJECT_AVOID_DISTANCE

        if (not math.isinf(forward_range) and forward_range < forward_object_avoid_distance):
            #print("Left Turn")
            if turn_type == 0:
                move_cmd.linear.x  =  SLOW_TURN_FORWARD_LINEAR_VEL
                move_cmd.angular.z =  TURN_ANGULAR_VEL
            else:
                move_cmd.linear.x  =  FAST_TURN_FORWARD_LINEAR_VEL
                move_cmd.angular.z =  TURN_ANGULAR_VEL
        elif math.isinf(forward_range):
            move_cmd.linear.x  = -1 * FORWARD_CORRECTION_LINEAR_VEL
            move_cmd.angular.z =  TURN_ANGULAR_VEL
        elif (not math.isinf(right_range) and right_range < RIGHT_OBJECT_AVOID_DISTANCE):
            #print("Left Turn Wall Correction")
            move_cmd.linear.x  =  FORWARD_CORRECTION_LINEAR_VEL
            move_cmd.angular.z =  wall_turn_correction
        elif (not math.isinf(left_range) and left_range < LEFT_OBJECT_AVOID_DISTANCE) or \
             (right_range > RIGHT_INNER_AVOID_DISTANCE):
            #print("Right Turn Inner Track Correction")
            move_cmd.linear.x  =  FORWARD_CORRECTION_LINEAR_VEL
            move_cmd.angular.z =  wall_turn_correction
        else:
            #print("Forward Move")
            move_cmd.linear.x  =  FORWARD_LINEAR_VEL
            move_cmd.angular.z =  0

        #print(f"Move Forward Vel: {move_cmd.linear.x}")
        #print(f"Move Angular Vel: {move_cmd.angular.z}")
        #print(f"Turn Type: {turn_type}")
        #print()
        pub.publish(move_cmd)

    def reset_odometry(self):
        """Clear the odometry readings list for a fresh lap."""
        self._odometry_readings = []

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
        """Calculate cumulative Euclidean distance and publish as action feedback."""
        total = 0.0
        for i in range(1, len(self._odometry_readings)):
            dx = self._odometry_readings[i].x - self._odometry_readings[i-1].x
            dy = self._odometry_readings[i].y - self._odometry_readings[i-1].y
            total += math.sqrt(dx**2 + dy**2)
        self._odom_record_feedback.current_total = total
        server.publish_feedback(self._odom_record_feedback)
        return total

    def check_lap_complete(self):
        """Check if robot has returned within LAP_CHECK_DISTANCE_DELTA of start position."""
        if len(self._odometry_readings) < LAP_CHECK_START_ODOMETRY_INDEX:
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
            rospy.loginfo("Log Robot Lap - Odom Record Action Server: Lap Completed")

    def execute_lap(self):
        """Call find_wall service, start wall following and wait for lap completion."""

        # wait for simulator clock to run  
        rate = rospy.Rate(10)
        rospy.loginfo("Log Robot Lap - Odom Record Action Server: Waiting for simulation clock...")
        while rospy.Time.now() == rospy.Time(0):
            rate.sleep()
        rospy.loginfo("Log Robot Lap - Odom Record Action Server: Simulation clock ready")

        if self._odometry_index == 0:
            rospy.loginfo("Log Robot Lap - Odom Record Action Server: Lap Started ...")
        rate = rospy.Rate(10)

        rospy.loginfo("Log Robot Lap - Odom Record Action Server: Calling find_wall service...")
        rospy.wait_for_service('find_wall')
        find_wall_client = rospy.ServiceProxy('find_wall', FindWall)
        response         = find_wall_client()

        if not response.wallfound:
            rospy.logerr("Log Robot Lap - Odom Record Action Server: Find_Wall Failed Error!")
            server.set_aborted()
            return

        rospy.loginfo("Log Robot Lap - Odom Record Action Server: Find_Wall Complete OK")

        self.reset_odometry()
        self._odometry_running = True

        scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        while not rospy.is_shutdown() and not self._lap_completed:

            if server.is_preempt_requested():
                rospy.logwarn("Log Robot Lap - Odom Record Action Server: Goal Cancelled!")
                self._odometry_running = False
                self.shutdown_handler()
                scan_sub.unregister()
                self._odom_record_result.list_of_odoms = self._odometry_readings
                server.set_preempted(self._odom_record_result)
                return

            rate.sleep()

        scan_sub.unregister()
        self.robot_stop()
        self._odom_record_result.list_of_odoms = self._odometry_readings
        server.set_succeeded(self._odom_record_result)
        rospy.loginfo(f'Log Robot Lap - Odom Record Action Server: Robot Travelled {self._odom_record_feedback.current_total} m ')
        rospy.loginfo(f'Log Robot Lap - Odom Record Action Server: Logged {len(self._odom_record_result.list_of_odoms)} odometry readings')

    def averaged_range(self, ranges, index, window=SCAN_AVERAGE_WINDOW):
        """Return averaged laser range at index over window samples, inf if mostly invalid."""
        samples   = []
        inf_count = 0
        for i in range(index - window, index + window + 1):
            val = ranges[i % len(ranges)]
            if math.isinf(val):
                inf_count += 1
            elif not math.isnan(val):
                samples.append(val)
        if inf_count >= 4:
            return float('inf')
        return sum(samples) / len(samples) if samples else float('inf')

    def robot_stop(self):
        """Publish zero velocity to stop the robot and reset scan and odometry counters."""
        rospy.loginfo(f'Log Robot Lap - Odom Record Action Server: Robot stopping ...')
        stop_cmd = Twist()
        stop_cmd.linear.x  = 0.0
        stop_cmd.angular.z = 0.0
        pub.publish(stop_cmd)
        rospy.sleep(0.5)
        self._scan_index     = 0
        self._odometry_index = 0


rospy.init_node('log_robot_lap_server_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

log_lap_object = LogLapClass()

rospy.Subscriber('/odom', Odometry, log_lap_object.odometry_callback)
rospy.on_shutdown(log_lap_object.shutdown_handler)

server = actionlib.SimpleActionServer(
    'record_lap',
    OdomRecordAction,
    execute_cb=log_lap_object.goal_callback,
    auto_start=False
)

server.start()
rospy.loginfo("Log Robot Lap - Odom Record Action Server: Launched")
rospy.spin()
