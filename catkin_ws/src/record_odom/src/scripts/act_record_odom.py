#!/usr/bin/env python3

import rospy
import actionlib
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from follow_outer_wall_action_server.msg import OdometerDistanceAction
from follow_outer_wall_action_server.msg import OdometerDistanceGoal

prev_x         = None
prev_y         = None
robot_odometer = 0.0
last_publish_time = None
odometer_pub   = None

def odom_callback(odom_msg):
    global prev_x, prev_y, robot_odometer, last_publish_time, odometer_pub

    cur_x = odom_msg.pose.pose.position.x
    cur_y = odom_msg.pose.pose.position.y

    if (prev_x is not None) and (prev_y is not None):
        dist = math.sqrt((cur_x - prev_x)**2 + (cur_y - prev_y)**2)
        robot_odometer += dist

    prev_x = cur_x
    prev_y = cur_y

    # Publish odometer once per second — server reads this topic
    now = rospy.Time.now()
    if (now - last_publish_time).to_sec() >= 1.0:
        odometer_pub.publish(Float32(robot_odometer))
        rospy.loginfo("Odometer: %.3f m", robot_odometer)
        last_publish_time = now

def action_feedback_callback(feedback):
    rospy.loginfo("FEEDBACK — distance: %.3f m  remaining: %.3f m",
                  feedback.current_distance,
                  feedback.distance_remaining)

def main():
    global last_publish_time, odometer_pub

    rospy.init_node('record_odom_action')

    last_publish_time = rospy.Time.now()
    odometer_pub      = rospy.Publisher('/odometer', Float32, queue_size=1)

    # Subscribe to odom — this node is the single source of distance truth
    rospy.Subscriber('/odom', Odometry, odom_callback)

    client = actionlib.SimpleActionClient('odometer_distance', OdometerDistanceAction)

    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Server connected!")

    goal = OdometerDistanceGoal()
    goal.target_distance = 10.0

    rospy.loginfo("Sending Travel Distance: %.2f m", goal.target_distance)
    client.send_goal(goal, feedback_cb=action_feedback_callback)

    client.wait_for_result()
    result = client.get_result()
    state  = client.get_state()

    rospy.loginfo("RESULT — total distance: %.3f m  state: %d",
                  result.total_distance, state)

if __name__ == '__main__':
    main()