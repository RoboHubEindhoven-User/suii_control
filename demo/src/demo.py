#!/usr/bin/env python
import rospy
from suii_control_msgs.msg import NavigationTask
from yaml import dump, load, safe_load
from geometry_msgs.msg import *
import io 

class Demo:
    def __init__(self):
        rospy.init_node("demo_node")
        rospy.loginfo("Demo Node")
        move = NavigationTask()
        move_task = NavigationTask()
        move.location.type = 1
        move.location.instance_id = 2
        move.location.description = "Workstation"
        move.pose.position.x = 1.0
        move.pose.position.y = 1.0
        move.pose.position.z = 1.0
        move.pose.orientation.x = 1.0
        move.pose.orientation.y = 1.0
        move.pose.orientation.z = 1.0
        move.pose.orientation.w = 1.0
        msg = dump(move)
        print msg
        print "Move type: " + str(type(msg))
        if type(move) == NavigationTask:
            print ("Task is a Navigation task")
            # self.save("config.yaml", move)
        move_task = load(msg)
        print "Task type: " + str(type(move_task))
        print move_task.location.description

    def save(self, filename, data):
        with open(filename, 'w') as yaml_file:
            dump(data, yaml_file, default_flow_style=False)

    def load(self, filename, is_safe_load = False):
        read_file = None
        with open(filename, 'r') as ymlfile:
            read_file = load(ymlfile) if is_safe_load else safe_load(ymlfile)
        return read_file



if __name__ == "__main__":
    handler = Demo()
    print("Reading file ...\n")
    # print dump(handler.load("config.yaml"))
    rospy.spin()
    # rate    = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     try:
    #         handler.save_data()
    #         rate.sleep()
    #     except rospy.ROSInterruptException:
    #         pass

