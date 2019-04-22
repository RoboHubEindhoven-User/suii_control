#!/usr/bin/env python

'''
'''

__author__      = "Zinkeng Thierry"

from yaml import load, dump, safe_load
from suii_control.utils.object_data import ObjectData
import io 
from suii_control.utils.service_area import ServiceArea
from suii_control.utils.task_specification import TaskSpecification, NavigationTask
from suii_control.utils.time_data import Time
from suii_control.utils.geometry_data import Pose2D, Position, Orientation

class FileHandler:
    def __init__(self):
        self.r = ""

    def save(self, filename, data):
        with open(filename, 'w') as yaml_file:
            dump(data, yaml_file, default_flow_style=False)

    def load(self, filename, is_safe_load = False):
        read_file = None
        with open(filename, 'r') as ymlfile:
            read_file = load(ymlfile) if is_safe_load else safe_load(ymlfile)
        return read_file;

    def load_service_area(self):
        filename = "../config/navigation_waypoints.yaml"
        file = self.load(filename)

    

if __name__ == '__main__':
    handler = FileHandler()

    pose1 = Pose2D(0.785, 0.527, 0.329)
    pose2 = Pose2D(0.254, 0.145, 0.325)
    pose3 = Pose2D(0.952, 0.175, 0.725)
    pose4 = Pose2D(0.958, 0.542, 0.357)

    obj1 = ObjectData("Bolt", 1.02, 2.01, 5.24, 3.265, 45, True)
    obj2 = ObjectData("Plastic Tube", 1.02, 2.01, 5.24, 3.785, 65, True)
    cont1 = ObjectData("Container 1", 0.25, 2.781, 5.24, 1.235, 15, True)
    cont2 = ObjectData("Container 2", 1.93, 3.25, 5.24, 3.265, 90, True)
    s1 = ServiceArea(2, "WS", 6, "Workstation 6", pose1)
    s2 = ServiceArea(2, "WS", 4, "Workstation 4", pose2)
    s3 = ServiceArea(2, "SH", 5, "Shelf 5", pose3)
    s4 = ServiceArea(2, "SH", 8, "Shelf 8", pose4)

    s1 = ServiceArea(2, "WS", 6, "Workstation 6", pose1)
    
    navigation_waypoints = [
        NavigationTask(ServiceArea(4, "WP", 6, "Waypoint 6", 4, "WEST", Position(0.785, 0.527, 7.329), Orientation(0.254, 0.145, 0.325, 1.254)), Time(3, 0)),
        NavigationTask(ServiceArea(5, "PP", 1, "Precision Platform", 1, "NORTH", Position(0.548, 8.527, 1.329), Orientation(0.254, 0.145, 0.325, 1.254)), Time(3, 0)),
        NavigationTask(ServiceArea(2, "WS", 10, "Workstation 10", 4, "WEST", Position(0.125, 0.527, 0.329), Orientation(0.254, 0.145, 0.325, 1.254)), Time(3, 0)),
        NavigationTask(ServiceArea(2, "WS", 2, "Workstation 2", 2, "EAST", Position(0.852, 0.527, 0.975), Orientation(0.254, 0.145, 0.325, 1.254)), Time(3, 0)),
        NavigationTask(ServiceArea(2, "WS", 12, "Workstation 12", 3, "SOUTH", Position(0.357, 1.527, 0.329), Orientation(0.254, 0.145, 0.325, 1.254)), Time(3, 0)),
        NavigationTask(ServiceArea(1, "SH", 2, "Shelf 2", 2, "WEST", Position(0.785, 0.452, 0.329), Orientation(0.254, 0.145, 0.325, 1.254)), Time(3, 0)),
        NavigationTask(ServiceArea(4, "WP", 9, "Waypoint 9", 3, "SOUTH", Position(0.753, 0.527, 1.329), Orientation(0.254, 0.145, 0.325, 1.254)), Time(3, 0)),
        NavigationTask(ServiceArea(2, "WS", 4, "Workstation 4", 4, "WEST", Position(0.159, 0.527, 2.329), Orientation(0.254, 0.145, 0.325, 1.254)), Time(3, 0))
    ]

    
    # obj = [TaskSpecification(obj1, cont1, s1, s2), TaskSpecification(obj2, cont2, s3, s4)]
    handler.save("navigation_waypoints.yaml", navigation_waypoints)
    print("reading yaml file\n")
    f = handler.load("navigation_waypoints.yaml")
    # print ("Theta: " + str(f[0].position.angle))

    print dump(f)

