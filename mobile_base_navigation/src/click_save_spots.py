#! /usr/bin/env python
import os
import rospy
import sys
from mobile_base_navigation.srv import GoalPose, GoalPoseResponse, GoalPoseRequest
from geometry_msgs.msg import PointStamped
import rospkg
from shutil import rmtree

class PoseRetriever(object):
    def __init__(self, spots_file_name="spots_saved.yaml"):
        rospy.Subscriber("/clicked_point", PointStamped, self.click_pose_callback)
        # initialize a pose object;
        self.pose_now = PointStamped()
        # use rospack to get package info
        rospack = rospkg.RosPack()
        # We clean up spots folder
        spots_folder_path = os.path.join(rospack.get_path('mobile_base_navigation'), "spots")

        self.spot_file_path = os.path.join(spots_folder_path, spots_file_name)

        file2write=open(self.spot_file_path,'w')
        file2write.write("### This is the Spot Saving File ###\n")
        file2write.close()
        rospy.loginfo("PoseRetriever READY...")

        self.index = 0
    def start_loop_service(self):
        rospy.spin() # mantain the service open.  

    def click_pose_callback(self, message):
        self.pose_now = message
        spot_name = "pose" + str(self.index)
        tab_str = "    "
        with open(self.spot_file_path, "a") as myfile:
            spot_pose = self.pose_now.point
            spot_x = spot_pose.x
            spot_y = spot_pose.y

            str_spot_x = tab_str+"x: "+str(spot_x)+"\n"
            str_spot_y = tab_str+"y: "+str(spot_y)+"\n"

            myfile.write(spot_name+":\n"+str_spot_x+str_spot_y + '\n')
        self.index += 1

def startPoseService():
    rospy.init_node('pose_service_server', log_level=rospy.INFO)
    if len(sys.argv) < 2:
        print("usage: spots_to_file.py spots_file_name")
    else:
        spots_file_name = str(sys.argv[1])+".yaml"

        pose_rtrv = PoseRetriever(spots_file_name=spots_file_name)
        pose_rtrv.start_loop_service()

if __name__ == "__main__":
    startPoseService()