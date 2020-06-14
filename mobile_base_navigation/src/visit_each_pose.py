#! /usr/bin/env python

import rospy
#from my_summit_localization.srv import MyServiceMsg, MyServiceMsgResponse
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseResult, MoveBaseGoal
from std_srvs.srv import Empty
import actionlib
import time
import os
import rosparam
import pandas as pd
import rospkg
import sys


class Visit_each_pose(object):
    def __init__(self, file_name):
        self.visit_next = rospy.Service('/visit_next', Empty , self.visit_next_pose) 
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        rate = rospy.Rate(1)

        goal=MoveBaseGoal()
        goal_tmp = Pose()

        self._ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.spots_file_name = file_name
        #use position to store all the x and y in csv
        self.poselist = []
        self.read_csv()
        self.index = 0

    def read_csv(self):
        # use rospack to get package info
        rospack = rospkg.RosPack()
        # We clean up spots folder
        spots_folder_path = os.path.join(rospack.get_path('mobile_base_navigation'), "spots")

        csv_path = os.path.join(spots_folder_path,self.spots_file_name)
        data = pd.read_csv(csv_path,names=['index','x','y'])
        for i in range(len(data)):
            self.poselist.append((data.iloc[i]['index'], data.iloc[i]['x'],data.iloc[i]['y']))

    def visit_pose(self, x, y):
        
        goal=MoveBaseGoal()
        goal_tmp = Pose()
        goal_tmp.position.x=x
        goal_tmp.position.y=y
        goal_tmp.position.z=0.0
        goal_tmp.orientation.x=0.0
        goal_tmp.orientation.y=0.0
        goal_tmp.orientation.z=1.0
        goal_tmp.orientation.w=0.0

        goal.target_pose.pose=goal_tmp
        goal.target_pose.header.frame_id='map'

        self.client.wait_for_server()
        self.client.send_goal(goal, feedback_cb=self.callback)
        self.client.wait_for_result()
        result=self.client.get_state()
        if result==3:
            print('successfuly reached point')

    def visit_next_pose(self, request):
        self.index += 1
        if self.index >= len(self.poselist):
            # if finish all the spot, just ctrl_C
            self.shutdownhook()
        else:
            x = self.poselist[self.index][1]
            y = self.poselist[self.index][2]
            rospy.loginfo('Going to spot='+str(self.poselist[self.index][0]))
            self.visit_pose(x, y)


    def shutdownhook(self):

        rospy.loginfo("shutdown time!")
        self._ctrl_c = True

    def callback(self, data):
        return


if __name__ == "__main__":
    rospy.init_node('send_coordinates_node', log_level=rospy.INFO)
    if len(sys.argv) < 2:
        print("usage: visit_each_pose.py spots_file_name")
    else:
        spots_file_name = str(sys.argv[1])+".csv"
        Visitor = Visit_each_pose(spots_file_name)
        rospy.spin() # mantain the service open.