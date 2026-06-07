#!/usr/bin/env python3

import rospy
import math
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from find_wall.srv import FindWall
from follow_outer_wall_action_server.msg import OdometerDistanceAction
from follow_outer_wall_action_server.msg import OdometerDistanceFeedback
from follow_outer_wall_action_server.msg import OdometerDistanceResult

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
WALL_TRACK_CENTER = (WALL_DISTANCE_MAX + WALL_DISTANCE_MIN) / 2
WALL_DISTANCE_RANGE = WALL_DISTANCE_MAX - WALL_DISTANCE_MIN

Scan_Index = 0

# Read from /odometer topic published by client — no local distance math needed
odometer_distance = 0.0

def shutdown_handler():
    rospy.loginfo("Shutting down — stopping robot")
    stop_cmd = Twist()
    stop_cmd.linear.x  = 0.0
    stop_cmd.angular.z = 0.0
    pub.publish(stop_cmd)
    rospy.sleep(0.5)

# Receive distance from client — single source of truth
def odometer_callback(msg):
    global odometer_distance
    odometer_distance = msg.data

def scan_callback(laserscan_data):
    global Scan_Index
    move_cmd = Twist()

    turn_type = rospy.get_param('~turn_type', 0)

    right_range   = averaged_range(laserscan_data.ranges, RIGHT_SCAN_INDEX)
    forward_range = averaged_range(laserscan_data.ranges, FORWARD_SCAN_INDEX)
    left_range    = averaged_range(laserscan_data.ranges, LEFT_SCAN_INDEX)

    print(f"Scan Data: {Scan_Index}")
    print(f"Right Wall Range: {right_range}")
    print(f"Forward Wall Range: {forward_range}")
    print(f"Left Wall Range: {left_range}")

    wall_track_error            = right_range - WALL_TRACK_CENTER
    normalized_wall_track_error = wall_track_error / WALL_DISTANCE_RANGE
    wall_turn_correction        = -1 * normalized_wall_track_error * TURN_CORRECTION_GAIN

    print(f"Wall Track Error: {wall_track_error}")
    print(f"Normalized Wall Track Error: {normalized_wall_track_error}")
    print(f"Wall Turn Correction Velocity: {wall_turn_correction}")

    if turn_type == 0:
        forward_object_avoid_distance = SLOW_FORWARD_OBJECT_AVOID_DISTANCE
    else:
        forward_object_avoid_distance = FAST_FORWARD_OBJECT_AVOID_DISTANCE

    if (not math.isinf(forward_range) and forward_range < forward_object_avoid_distance):
        print("Left Turn")
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
        print("Left Turn Wall Correction")
        move_cmd.linear.x  =  FORWARD_CORRECTION_LINEAR_VEL
        move_cmd.angular.z =  wall_turn_correction
    elif (not math.isinf(left_range) and left_range < LEFT_OBJECT_AVOID_DISTANCE) or \
         (right_range > RIGHT_INNER_AVOID_DISTANCE):
        print("Right Turn Inner Track Correction")
        move_cmd.linear.x  =  FORWARD_CORRECTION_LINEAR_VEL
        move_cmd.angular.z =  wall_turn_correction
    else:
        print("Forward Move")
        move_cmd.linear.x  =  FORWARD_LINEAR_VEL
        move_cmd.angular.z =  0

    print(f"Move Forward Vel: {move_cmd.linear.x}")
    print(f"Move Angular Vel: {move_cmd.angular.z}")
    print(f"Turn Type: {turn_type}")
    print()
    pub.publish(move_cmd)

def execute_callback(goal):
    global odometer_distance

    rospy.loginfo("Goal received: follow wall for %.2f m", goal.target_distance)

    # Snapshot odometer at goal start — track relative distance
    start_distance = odometer_distance

    feedback = OdometerDistanceFeedback()
    result   = OdometerDistanceResult()
    rate     = rospy.Rate(10)

    # Wait for wall alignment
    rospy.loginfo("Calling find_wall service...")
    rospy.wait_for_service('find_wall')
    find_wall_client = rospy.ServiceProxy('find_wall', FindWall)
    response = find_wall_client()

    if not response.wallfound:
        rospy.logerr("find_wall returned wallfound=False — aborting goal")
        server.set_aborted()
        return

    rospy.loginfo("Wall found and aligned — starting follow loop")

    # Start laser scan subscriber
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

    while not rospy.is_shutdown():

        # Check for cancel
        if server.is_preempt_requested():
            rospy.loginfo("Goal CANCELLED")
            scan_sub.unregister()
            pub.publish(Twist())
            server.set_preempted()
            return

        # Calculate distance relative to goal start
        travelled = odometer_distance - start_distance

        # Check if goal distance reached
        if travelled >= goal.target_distance:
            rospy.loginfo("Goal REACHED — %.3f m travelled", travelled)
            scan_sub.unregister()
            pub.publish(Twist())
            break

        # Publish feedback using relative distance
        feedback.current_distance   = travelled
        feedback.distance_remaining = goal.target_distance - travelled
        server.publish_feedback(feedback)
        rate.sleep()

    result.total_distance = odometer_distance - start_distance
    server.set_succeeded(result)

def averaged_range(ranges, index, window=SCAN_AVERAGE_WINDOW):
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

rospy.init_node('node_follow_outer_wall')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

# Subscribe to odometer topic — client is single source of distance truth
rospy.Subscriber('/odometer', Float32, odometer_callback)

rospy.on_shutdown(shutdown_handler)

server = actionlib.SimpleActionServer(
    'odometer_distance',
    OdometerDistanceAction,
    execute_cb=execute_callback,
    auto_start=False
)
server.start()
rospy.loginfo("Follow Wall Action Server Ready")

rospy.spin()