#!/usr/bin/env python
import rospy
import sys
from suii_control.state_control import ConfigurationHandler

if __name__ == '__main__':
    handler = ConfigurationHandler()
    rate    = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            handler.save_data()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass