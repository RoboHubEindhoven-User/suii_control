#!/usr/bin/env python
import rospy
import yaml

class JointPose(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!JointPose'

    def __init__(self, name, pose):
      self.pose = pose

class RoboticArmPose:

    def __init__(self, file_name):
        self.joint_poses = []
        self.age = age
        self.read_poses()
        
    def read_poses(self):
        file_handler = FileHandler()
        if rospy.has_param('/suii_control_node/pose_path'):
            file_name = rospy.get_param('/suii_control_node/pose_path')
            self.joint_poses = file_handler.load(file_name)
        else:
            print 'Could not find parameter!'
    
    def save_poses(self, poses_msg):
        poses = []
        file_handler = FileHandler()
        file_handler.save(self.filename, poses)
    
