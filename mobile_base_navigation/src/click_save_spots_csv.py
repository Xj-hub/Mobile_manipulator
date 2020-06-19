#! /usr/bin/env python
import os
import rospy
import sys
from mobile_base_navigation.srv import GoalPose, GoalPoseResponse, GoalPoseRequest
from geometry_msgs.msg import PointStamped
import rospkg
import csv
import os
class PoseRetriever(object):
    def __init__(self, spots_file_name="spots_saved.csv"):
        rospy.Subscriber("/clicked_point", PointStamped, self.click_pose_callback)
        # initialize a pose object;
        self.pose_now = PointStamped()
        # use rospack to get package info
        rospack = rospkg.RosPack()
        # We clean up spots folder
        spots_folder_path = os.path.join(rospack.get_path('mobile_base_navigation'), "spots")

        self.spot_file_path = os.path.join(spots_folder_path, spots_file_name)
        if os.path.exists(self.spot_file_path):
            try:
                os.remove(self.spot_file_path)
            except:
                rospy.loginfo("File Not Found "+str(self.spot_file_path) )
            #Init File for Spot saving
        rospy.loginfo("PoseRetriever READY...")
        self.csvfile = open(self.spot_file_path, "a")
        self.index = 0

    def start_loop_service(self):
        rospy.spin() # mantain the service open.  

    def click_pose_callback(self, message):
        self.pose_now = message
        spot_index = self.index
        spot_pose = self.pose_now.point
        spot_x = spot_pose.x
        spot_y = spot_pose.y
        with open(self.spot_file_path, "a") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([spot_index,spot_x,spot_y])
        self.index += 1

def startPoseService():
    rospy.init_node('pose_service_server', log_level=rospy.INFO)
    if len(sys.argv) < 2:
        print("usage: click_save_spots_csv.py csv_file_name")
    else:
        spots_file_name = str(sys.argv[1])+".csv"

        pose_rtrv = PoseRetriever(spots_file_name=spots_file_name)
        pose_rtrv.start_loop_service()

if __name__ == "__main__":
    startPoseService()