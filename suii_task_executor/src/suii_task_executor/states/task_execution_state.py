#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty
from suii_msgs.srv import ItemPick, ItemFindhole, ItemPlace, NavigationGoal

from suii_protocol.task_protocol import TaskProtocol
from suii_protocol.protocol.enum_task_action import TaskActionType
from suii_protocol.protocol.enum_location_identifier import LocationIdentifierType
from geometry_msgs.msg import Pose

from zinki_smachine import *
from state_properties import StateName, TransitionName
from suii_mux_manager_comm.yaml_handler import YAMLHandler
from enum import Enum

# The location where the robot needs to find holes 
IN_HOLE_LOCATION = LocationIdentifierType.PP

class ItemFindHoleClient:
    @staticmethod
    def call_serv(itemID):
        # Test code
        rospy.loginfo("Calling 'ItemFindhole' with:")
        data = ItemFindhole()
        data.itemID = itemID 
        rospy.loginfo("data: itemID = %d" % (data.itemID))
        return 1, 0
        
        # try:
        #     proxy = rospy.ServiceProxy('ItemFindhole', ItemFindhole)
        #     res = proxy(itemID)
        #     return res.sucess, res.error
        # except rospy.ServiceException:
        #     rospy.logerr("Service call failed")

class ItemDriveClient:
    @staticmethod
    def call_serv():
        # Test code
        rospy.loginfo("Calling 'ItemDrive' with:")
        data = Empty()
        rospy.loginfo(data)
        return 1, 0

        # try:
        #     proxy = rospy.ServiceProxy('ItemDrive', ItemDrive)
        #     res = proxy(Empty())
        #     return res.sucess, res.error
        # except rospy.ServiceException:
        #     rospy.logerr("Service call failed")

class NavigationGoalClient:
    @staticmethod
    def call_serv(pose):
        # Test code
        rospy.loginfo("Calling 'to_goal'")
        rospy.loginfo(pose)
        return 0

        # try:
        #     proxy = rospy.ServiceProxy('to_goal', NavigationGoal)
        #     res = proxy(pose)
        #     return res.status
        # except rospy.ServiceException:
        #     rospy.logerr("Service call failed")

class ItemPickClient:
    @staticmethod
    def call_serv(itemID, onRobot):
        # Test code
        rospy.loginfo("Calling 'ItemPick'")
        data = ItemPick()
        data.itemID = itemID 
        data.onRobot = onRobot
        rospy.loginfo("With data: itemID = %d, onRobot = %i" % (data.itemID, data.onRobot))
        return 1, 0

        # try:
        #     proxy = rospy.ServiceProxy('ItemPick', ItemPick)
        #     res = proxy(itemID, onRobot)
        #     return res.sucess, res.error
        # except rospy.ServiceException:
        #     rospy.logerr("Service call failed")

class ItemPlaceClient:
    @staticmethod
    def call_serv(itemID, onRobot, inHole):
        # Test code
        rospy.loginfo("Calling 'ItemPlace'")
        data = ItemPlace()
        data.itemID = itemID 
        data.onRobot = onRobot
        data.inHole = inHole
        rospy.loginfo("With data: itemID = %d, onRobot = %i, inHole = %i" % (data.itemID, data.onRobot, data.inHole))
        return 1, 0

        # try:
        #     proxy = rospy.ServiceProxy('ItemPlace', ItemPlace)
        #     res = proxy(itemID, onRobot, inHole)
        #     return res.sucess, res.error
        # except rospy.ServiceException:
        #     rospy.logerr("Service call failed")

class TaskExecutionState(State):
    def __init__(self, fsm):
        super(TaskExecutionState, self).__init__(fsm)
        self.task = None 
        self.yaml_path = rospy.get_param('~yaml_path')
        self.yaml_handler = YAMLHandler(path=self.yaml_path)
        self.yaml_handler.load()

    def execute(self):
        rospy.loginfo("* Received task: %s" % self.task)
        self.dispatch()
        self.fsm.transition(TransitionName.TASK_SELECT)
    
    def error_handler(self):
        rospy.logerr('UH OH')
        self.fsm.states[StateName.TASK_SELECT].error = True

    def format_nav_msg(self, orientation, position):
        msg = Pose()
        msg.orientation.w = orientation[0]
        msg.orientation.x = orientation[1]
        msg.orientation.y = orientation[2]
        msg.orientation.z = orientation[3]

        msg.position.x = position[0]
        msg.position.y = position[1]
        msg.position.z = position[2]
        return msg

    def dispatch(self):
        if (self.task.action == int(TaskActionType.PICK)):
            rospy.loginfo('Action Type: Pick')
            success, error = ItemPickClient.call_serv(self.task.object, False)
            if not success: self.error_handler()
        elif (self.task.action == int(TaskActionType.PICK_FROM_ROBOT)):
            rospy.loginfo('Action Type: Pick from robot')
            success, error = ItemPickClient.call_serv(self.task.object, True)
            if not success: self.error_handler()
        elif (self.task.action == int(TaskActionType.PLACE)):
            rospy.loginfo('Action Type: Place')
            if (self.task.destination == IN_HOLE_LOCATION):
                success, error = ItemPlaceClient.call_serv(self.task.object, False, True)
            else:
                success, error = ItemPlaceClient.call_serv(self.task.object, False, False)
            if not success: self.error_handler()
        elif (self.task.action == int(TaskActionType.PLACE_TO_ROBOT)):
            rospy.loginfo('Action Type: Place to robot')
            success, error = ItemPlaceClient.call_serv(self.task.object, True, False)
            if not success: self.error_handler()
        elif (self.task.action == int(TaskActionType.DRIVE)):
            rospy.loginfo('Action type: Drive')
            rospy.loginfo('Driving to: %s' % self.task.destination_str)

            # index of the data: orientation [w,x,y,z]  - position [x,y,z]
            orientation, position = self.yaml_handler.get_pose_for(self.task.destination_str)
            
            status = NavigationGoalClient.call_serv(self.format_nav_msg(orientation, position))
            if not status: self.error_handler()
        elif (self.task.action == int(TaskActionType.MOVE_TO_DRIVE)):
            rospy.loginfo('Action type: Move to drive')
            success, error = ItemDriveClient.call_serv()
            if not success: self.error_handler() # i hope it never gets here
        elif (self.task.action == int(TaskActionType.FIND_HOLE)):
            rospy.loginfo('Action type: Find hole')
            success, error = ItemFindHoleClient.call_serv(self.task.object)
            if not success: self.error_handler()