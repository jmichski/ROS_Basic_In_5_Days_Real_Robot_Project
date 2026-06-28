#!/usr/bin/env python3

import rospy
import actionlib
from log_real_robot_lap.msg import OdomRecordRealAction, OdomRecordRealGoal, OdomRecordRealResult, OdomRecordRealFeedback

MAX_LAP_LENGTH = 6.0  # maximum lap distance in meters before goal is cancelled

LOG_FUNCTION_CALLS = 0
LOG_FUNCTION_RETURNS = 0
LOG_ODOM_DISANCES = 0
LOG_LOGIC_STATES = 1
LOG_FINAL_RESULTS = 1
LOG_AI_DEBUGINFO = 0
LOG_DEV_DEBUGINFO = 0

# ── NODE / CLASS constants — used in every structured log line so the
# pipe-delimited output imports into Excel/CSV with consistent columns,
# matching log_real_robot_lap_server and find_wall_real_robot_svc
# (NODE | CLASS | FUNC | TAG). ───────────────────────────────────────
LOG_NODE  = "log_real_robot_lap_client"
LOG_CLASS = "Log Lap Client"


class LogLapRealClientClass(object):
    _client             = None    # actionlib client handle
    _action_server_name = None    # action server topic name
    _total_distance     = 0.0     # running distance updated by feedback callback
    _prev_logic_state   = None    # last announced LOG_LOGIC_STATES label — logs only on change

    def _log_state(self, new_state):
        """Emit a LOG_LOGIC_STATES transition line only when the state actually
        changes — mirrors the STATE: prev -> curr pattern used by the lap server
        and find_wall service."""
        if LOG_LOGIC_STATES and new_state != self._prev_logic_state:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: _log_state() | STATE: %s -> %s",
                          LOG_NODE, LOG_CLASS, self._prev_logic_state, new_state)
        self._prev_logic_state = new_state

    def __init__(self):
        """Initialise ROS node and action client."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: __init__() | CALL", LOG_NODE, LOG_CLASS)

        rospy.init_node('log_real_robot_lap_client_node')
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: __init__() | DEV: Initialized", LOG_NODE, LOG_CLASS)
        self._action_server_name = '/record_lap_real'
        self._client = actionlib.SimpleActionClient(self._action_server_name, OdomRecordRealAction)

        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: __init__() | RETURN: (none)", LOG_NODE, LOG_CLASS)

    def run_server(self):
        """Connect to action server, send lap goal and handle final state."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: run_server() | CALL", LOG_NODE, LOG_CLASS)

        self._log_state("WAITING_FOR_SERVER")
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: run_server() | DEV: Waiting for Server %s",
                          LOG_NODE, LOG_CLASS, self._action_server_name)

        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: run_server() | DEV: Waiting for clock", LOG_NODE, LOG_CLASS)
        while rospy.Time.now() == rospy.Time(0):
            rospy.sleep(0.1)
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: run_server() | DEV: Clock ready", LOG_NODE, LOG_CLASS)

        self._client.wait_for_server()
        if LOG_DEV_DEBUGINFO:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: run_server() | DEV: Action Server Found %s",
                          LOG_NODE, LOG_CLASS, self._action_server_name)

        goal = OdomRecordRealGoal()
        self._log_state("GOAL_SENT")
        self._client.send_goal(goal, feedback_cb=self.feedback_callback)
        self._client.wait_for_result()

        state = self._client.get_state()

        if state == actionlib.GoalStatus.SUCCEEDED:
            result = self._client.get_result()
            self._log_state("GOAL_SUCCEEDED")
            if LOG_FINAL_RESULTS:
                rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: run_server() | FINAL_RESULT: lap_complete points; %d total_distance; %.3f",
                              LOG_NODE, LOG_CLASS, len(result.list_of_odoms), self._total_distance)

        elif state == actionlib.GoalStatus.PREEMPTED:
            self._log_state("GOAL_PREEMPTED")
            rospy.logwarn(" | NODE: %s | CLASS: %s | FUNC: run_server() | WARN: Goal CANCELLED - preempted by server or axclient",
                          LOG_NODE, LOG_CLASS)

        elif state == actionlib.GoalStatus.ABORTED:
            self._log_state("GOAL_ABORTED")
            rospy.logerr(" | NODE: %s | CLASS: %s | FUNC: run_server() | ERROR: Goal ABORTED - find_wall_real_robot failed or server error",
                        LOG_NODE, LOG_CLASS)

        else:
            self._log_state("GOAL_UNKNOWN_STATE")
            rospy.logwarn(" | NODE: %s | CLASS: %s | FUNC: run_server() | WARN: Goal ended with unexpected state; %d",
                          LOG_NODE, LOG_CLASS, state)

        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: run_server() | RETURN: state; %d", LOG_NODE, LOG_CLASS, state)

    def feedback_callback(self, feedback):
        """Receive distance feedback and cancel goal if max lap length exceeded."""
        if LOG_FUNCTION_CALLS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: feedback_callback() | CALL", LOG_NODE, LOG_CLASS)

        self._total_distance = feedback.current_total
        if LOG_ODOM_DISANCES:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: feedback_callback() | ODOM: distance_so_far; %.3f",
                          LOG_NODE, LOG_CLASS, self._total_distance)
        if self._total_distance > MAX_LAP_LENGTH:
            self._log_state("MAX_LAP_LENGTH_EXCEEDED")
            rospy.logwarn(" | NODE: %s | CLASS: %s | FUNC: feedback_callback() | WARN: Max Lap Length Exceeded (%.3f > %.3f) - Cancelling Goal",
                          LOG_NODE, LOG_CLASS, self._total_distance, MAX_LAP_LENGTH)
            self._client.cancel_goal()

        if LOG_FUNCTION_RETURNS:
            rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: feedback_callback() | RETURN: (none)", LOG_NODE, LOG_CLASS)


if __name__ == '__main__':
    if LOG_FUNCTION_CALLS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: main() | CALL", LOG_NODE, LOG_CLASS)

    log_lap_real_client_object = LogLapRealClientClass()
    log_lap_real_client_object.run_server()
    rospy.spin()

    if LOG_FUNCTION_RETURNS:
        rospy.loginfo(" | NODE: %s | CLASS: %s | FUNC: main() | RETURN", LOG_NODE, LOG_CLASS)