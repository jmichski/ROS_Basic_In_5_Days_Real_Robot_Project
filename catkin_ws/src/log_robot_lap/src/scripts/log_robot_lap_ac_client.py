#!/usr/bin/env python3
 
import rospy
import actionlib
from log_robot_lap.msg import OdomRecordAction, OdomRecordGoal, OdomRecordResult, OdomRecordFeedback
 
MAX_LAP_LENGTH = 6.0  # maximum lap distance in meters before goal is cancelled
 
class LogLapClassClient(object):
    _client             = None   # actionlib client handle
    _action_server_name = None   # action server topic name
    _total_distance     = 0.0    # running distance updated by feedback callback
 
    def __init__(self):
        """Initialise ROS node and action client."""
        rospy.init_node('log_robot_lap_client_node')
        rospy.loginfo('Log Robot Lap - Odom Record Action Client: Initialized ...')
        self._action_server_name = '/record_lap'
        self._client = actionlib.SimpleActionClient(self._action_server_name, OdomRecordAction)
 
    def run_server(self):
        """Connect to action server, send lap goal and handle final state."""
        rospy.loginfo('Log Robot Lap - Odom Record Action Client: Waiting for Server ' + self._action_server_name)
        
        rate = rospy.Rate(10)

        rospy.loginfo('Log Robot Lap - Odom Record Action Client: Waiting for simulation clock...')
        while rospy.Time.now() == rospy.Time(0):
            rospy.sleep(0.1)
        rospy.loginfo('Log Robot Lap - Odom Record Action Client: Simulation clock ready')

        
        self._client.wait_for_server()
        rospy.loginfo('Log Robot Lap - Odom Record Action Client: Action Server Found ' + self._action_server_name)
 
        goal = OdomRecordGoal()
        self._client.send_goal(goal, feedback_cb=self.feedback_callback)
        self._client.wait_for_result()
 
        state = self._client.get_state()
 
        if state == actionlib.GoalStatus.SUCCEEDED:
            result = self._client.get_result()
            rospy.loginfo('Log Robot Lap - Odom Record Action Client: Lap Complete — %d points — total: %.3f m',
                          len(result.list_of_odoms), self._total_distance)
 
        elif state == actionlib.GoalStatus.PREEMPTED:
            rospy.logwarn('Log Robot Lap - Odom Record Action Client: Goal CANCELLED — preempted by server or axclient')
 
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.logerr('Log Robot Lap - Odom Record Action Client: Goal ABORTED — find_wall failed or server error')
 
        else:
            rospy.logwarn('Log Robot Lap - Odom Record Action Client: Goal ended with unexpected state: %d', state)
 
    def feedback_callback(self, feedback):
        """Receive distance feedback and cancel goal if max lap length exceeded."""
        self._total_distance = feedback.current_total
        rospy.loginfo('Log Robot Lap - Odom Record Action Client: Distance so far: %.3f m', self._total_distance)
        if self._total_distance > MAX_LAP_LENGTH:
            rospy.logwarn('Log Robot Lap - Odom Record Action Client: Max Lap Length Exceeded! Cancelling Goal!')
            self._client.cancel_goal()
 
 
if __name__ == '__main__':
    log_lap_client_object = LogLapClassClient()
    log_lap_client_object.run_server()
    rospy.spin()