#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import os

class StopPointNavigator:
    def __init__(self, stop_points_file):
        self.stop_points_file = stop_points_file
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.read_stop_points()

    def read_stop_points(self):
        self.stop_points = []
        if os.path.exists(self.stop_points_file):
            with open(self.stop_points_file, 'r') as f:
                for line in f:
                    x, y, z, qx, qy, qz, qw = map(float, line.strip().split())
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = x
                    goal.target_pose.pose.position.y = y
                    goal.target_pose.pose.position.z = z
                    goal.target_pose.pose.orientation.x = qx
                    goal.target_pose.pose.orientation.y = qy
                    goal.target_pose.pose.orientation.z = qz
                    goal.target_pose.pose.orientation.w = qw
                    self.stop_points.append(goal)
        else:
            rospy.logerr(f"Stop points file not found: {self.stop_points_file}")

    def navigate_to_stop_points(self):
        for goal in self.stop_points:
            self.client.send_goal(goal)
            self.client.wait_for_result()
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal reached!")
            else:
                rospy.loginfo("Goal failed!")

if __name__ == '__main__':
    rospy.init_node('stop_point_navigator')
    stop_points_file = rospy.get_param('~stop_points_file')
    navigator = StopPointNavigator(stop_points_file)
    navigator.navigate_to_stop_points()