#! /usr/bin/env python
from suii_control_msgs.msg import *
from std_msgs.msg import String
from yaml import dump, load
from utils.geometry_data import Pose, Coordinate
from utils.file_handler import FileHandler

class Geometry:
    def __init__(self):
        
        rospy.loginfo("initializing Geometry...")
        
        self.waypoints   = self.load_waypoints()
        self.coordinates = self.load_coordinates()

    def load_waypoints(self):
        file_handler = FileHandler()
        if rospy.has_param('/suii_task_manager_node/path'):
            file_name = rospy.get_param('/suii_task_manager_node/path')
            return file_handler.load(file_name)
            # print yaml.dump(self.navigation_tasks_config)
        else:
            print 'Could not find waypoints parameter!'
        return None

    def load_coordinates(self):
        file_handler = FileHandler()
        if rospy.has_param('/suii_task_manager_node/coordinate_path'):
            file_name = rospy.get_param('/suii_task_manager_node/coordinate_path')
            return file_handler.load(file_name)
            # print yaml.dump(self.navigation_tasks_config)
        else:
            print 'Could not find coordinates parameter!'
        return None

    def get_waypoint_pose(self, area_type, instance_id):
        pose = Pose()

        for i in range(len(self.waypoints)):
             if ((area_type == 5) or (area_type == 3)): # precision place service area
                    if ((waypoints[i].destination.area_type == area_type)):
                        pose.position    = self.waypoints[i].destination.position
                        pose.orientation = self.waypoints[i].destination.orientation
                        break
                else:
                    if ((waypoints[i].destination.area_type == area_type) and  (waypoints[i].destination.instance_id == instance_id)):
                        pose.position    = self.waypoints[i].destination.position
                        pose.orientation = self.waypoints[i].destination.orientation
                        break
        return pose


    def get_orientation(self, direction):
        """
            This is use for basic navigation task only
        """
        for i in range(len(self.coordinates)):
            if ((self.coordinates[i].direction == direction)):
                return self.coordinates[i].orientation
        return None