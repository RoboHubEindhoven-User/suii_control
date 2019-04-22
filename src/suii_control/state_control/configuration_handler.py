#!/usr/bin/env python
import rospy 
from suii_control.msg import *
from suii_control.utils.object_data import ObjectData
from suii_control.utils.service_area import ServiceArea
from suii_control.utils.task_specification import TaskSpecification, NavigationTask
from suii_control.utils.time_data import Time
from suii_control.utils.geometry_data import Pose2D, Position, Orientation
from suii_control.utils.file_handler import FileHandler

class ConfigurationHandler:
    def __init__(self):
        rospy.init_node("configuration_handler_node")
        rospy.loginfo("Starting Configuration Handler Node")
        rospy.Subscriber("/roboway/waypoint", WaypointData, self.waypoints_callback)
        self.waypoint_data = WaypointData()
        self.should_save = False
        self.filename   =  self.get_filename()
        rospy.loginfo("Configuration Handler Node started succesfully")
        
    def get_filename(self):
        if rospy.has_param('/configuration_handler_node/path'):
            return rospy.get_param('/configuration_handler_node/path')
        else:
            print 'Could not find parameter!'
        return ""
        


    def waypoints_callback(self, waypoint_data_msg):
        rospy.loginfo("Received Waypoints Configuration")
        self.waypoint_data = waypoint_data_msg
        self.should_save   = True 

    def save_data(self):
        if (self.should_save):
            rospy.loginfo("Saving waypoints configuration...")
            navigation_waypoints = []

            for i in range(len(self.waypoint_data.waypoints)):
                waypoint = self.waypoint_data.waypoints[i]
                navigation_waypoints.append(
                    NavigationTask(
                        ServiceArea(
                            waypoint.type,
                            self.get_location_str(waypoint.type), 
                            waypoint.instance_id, waypoint.description, 
                            0, 
                            "",
                            Position(waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z),
                            Orientation(waypoint.pose.orientation.x, waypoint.pose.orientation.y, waypoint.pose.orientation.z, waypoint.pose.orientation.w)
                        ),
                        Time(3, 0)
                    )
                )

            self.should_save = False
            file_handler = FileHandler()
            file_handler.save(self.filename, navigation_waypoints)
            rospy.loginfo("Waypoints configuration saved")

    def get_location_str(self, location):
        if (location == 1):
            return "SH"
        if (location == 2):
            return "WS"
        if (location == 3):
            return "CB"
        if (location == 4):
            return "WP"
        if (location == 5):
            return "PP"
        return ""

