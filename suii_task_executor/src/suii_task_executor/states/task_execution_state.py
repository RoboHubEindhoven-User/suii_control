#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32
from suii_msgs.srv import ItemPick, ItemDrive, ItemFindhole, ItemPlace, NavigationGoal

from suii_protocol.task_protocol import TaskProtocol
from suii_protocol.protocol.enum_task_action import TaskActionType
from suii_protocol.protocol.enum_location_identifier import LocationIdentifierType
from suii_protocol.protocol.enum_orientation import OrientationIdentifierType
from geometry_msgs.msg import Pose

from zinki_smachine import *
from state_properties import StateName, TransitionName
from suii_mux_manager_comm.yaml_handler import YAMLHandler
from enum import Enum

## ===== TaskExecutionState ===== ##
# Action: muxing
class TaskExecutionState(State):
    def __init__(self, fsm):
        super(TaskExecutionState, self).__init__(fsm)
        self.task = None 
        self.yaml_path = rospy.get_param('~yaml_path')
        self.yaml_handler = YAMLHandler(path=self.yaml_path)
        self.yaml_handler.load()
        self.table_height = -1
        self.height_pub = rospy.Publisher('/table_height', Float32, queue_size=10, latch=True)
        self.find_hole_success = False

    def execute(self):
        rospy.loginfo("* Received task: %s" % self.task)
        self.dispatch()
        self.fsm.transition(TransitionName.TASK_SELECT)
    
    def error_handler(self):
        rospy.logerr('UH OH')
        # Propagate up to TaskSelect so it knows
        self.fsm.states[StateName.TASK_SELECT].error = True

    def get_orientation_arr_from_id(self, orientation_id):
         # index of the data: orientation [w,x,y,z]  - position [x,y,z]
        orientation = [0,0,0,0]
        if (orientation_id == int(OrientationIdentifierType.NORTH)):
            orientation[0] = 0
            orientation[1] = 0
            orientation[2] = -1
            orientation[3] = 0
        elif (orientation_id == int(OrientationIdentifierType.EAST)):
            orientation[0] = -1
            orientation[1] = 0
            orientation[2] = 0
            orientation[3] = 0
        elif (orientation_id == int(OrientationIdentifierType.SOUTH)):
            orientation[0] = 0
            orientation[1] = -1
            orientation[2] = 0
            orientation[3] = 0
        elif (orientation_id == int(OrientationIdentifierType.WEST)):
            orientation[0] = 0
            orientation[1] = 0
            orientation[2] = 0
            orientation[3] = -1
        return orientation


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
        # Dispatch by type
        if (self.task.action == int(TaskActionType.PICK)):
            rospy.loginfo('Action Type: Pick')
            res = ItemPickClient.call_serv(self.task.object, False)
        
        elif (self.task.action == int(TaskActionType.PICK_FROM_ROBOT)):
            rospy.loginfo('Action Type: Pick from robot')
            res =  ItemPickClient.call_serv(self.task.object, True)
            # if (self.task.object == 4):
            #     if res == None or not res.succes:
            #         self.error_handler()
        
        elif (self.task.action == int(TaskActionType.PLACE)):
            rospy.loginfo('Action Type: Place')
            if (self.find_hole_success):
                res =  ItemPlaceClient.call_serv(self.task.object, False, True)
                self.find_hole_success = False # Reset
            else:
                res =  ItemPlaceClient.call_serv(self.task.object, False, False)
            # if not success: self.error_handler()
        
        elif (self.task.action == int(TaskActionType.PLACE_TO_ROBOT)):
            rospy.loginfo('Action Type: Place to robot')
            res =  ItemPlaceClient.call_serv(self.task.object, True, False)
            # if not success: self.error_handler()

        elif (self.task.action == int(TaskActionType.DRIVE)):
            rospy.loginfo('Action type: Drive')
            rospy.loginfo('Driving to: %s' % self.task.destination_str)

            # index of the data: orientation [w,x,y,z]  - position [x,y,z]
            orientation, position = self.yaml_handler.get_pose_for(self.task.destination_str)
            if (self.task.orientation != -1):
                orientation = self.get_orientation_arr_from_id(self.task.orientation)

            if orientation != None and position != None:
                status = NavigationGoalClient.call_serv(self.format_nav_msg(orientation, position))
                self.table_height = self.yaml_handler.get_table_height_for(self.task.destination_str)
                rospy.loginfo("Publishing table height: %f", self.table_height)
                self.height_pub.publish(self.table_height)
            else:
                rospy.logfatal('Cannot find location in yaml file')
                rospy.logfatal('YAML at: %s' % self.yaml_path)
        
        elif (self.task.action == int(TaskActionType.MOVE_TO_DRIVE)):
            rospy.loginfo('Action type: Move to drive')
            res =  ItemDriveClient.call_serv()
        
        elif (self.task.action == int(TaskActionType.FIND_HOLE)):
            rospy.loginfo('Action type: Find hole')
            res =  ItemFindHoleClient.call_serv(self.task.object)
            if (res != None):
                self.find_hole_success = res.sucess
            rospy.loginfo("Find hole success: %r" % self.find_hole_success)

## ===== Service Clients ===== ##
class ItemFindHoleClient:
    @staticmethod
    def call_serv(itemID):
        rospy.loginfo("Calling 'ItemFindhole' with: itemID = %d" % (itemID))

        # Test code
        # data = ItemFindhole()
        # data.itemID = itemID 
        # rospy.loginfo("data: itemID = %d" % (data.itemID))
        # return 0, 0
        
        try:
            proxy = rospy.ServiceProxy('ItemFindhole', ItemFindhole)
            res = proxy(itemID)
            return res
        except rospy.ServiceException:
            rospy.logerr("Service call failed")

class ItemDriveClient:
    @staticmethod
    def call_serv():
        # Test code
        rospy.loginfo("Calling 'ItemDrive'")
        # data = Empty()
        # rospy.loginfo(data)
        # return 1, 0

        try:
            proxy = rospy.ServiceProxy('ItemDrive', ItemDrive)
            res = proxy()
            return res
        except rospy.ServiceException:
            rospy.logerr("Service call failed")

class NavigationGoalClient:
    @staticmethod
    def call_serv(pose):
        # Test code
        rospy.loginfo("Calling 'move_to_goal'")
        rospy.loginfo(pose)
        # return 1

        try:
            proxy = rospy.ServiceProxy('move_to_goal', NavigationGoal)
            res = proxy(pose)
            return not res.status # Flip retval because nav returns 0 for success
        except rospy.ServiceException:
            rospy.logerr("Service call failed")

class ItemPickClient:
    @staticmethod
    def call_serv(itemID, onRobot):
        rospy.loginfo("Calling 'ItemPick' with data: itemID = %d, onRobot = %r" % (itemID, onRobot))
        # data = ItemPick()
        # data.itemID = itemID 
        # data.onRobot = onRobot
        # return 1, 0

        try:
            proxy = rospy.ServiceProxy('ItemPick', ItemPick)
            res = proxy(itemID, onRobot)
            return res
        except rospy.ServiceException:
            rospy.logerr("Service call failed")

class ItemPlaceClient:
    @staticmethod
    def call_serv(itemID, onRobot, inHole):
        # Test code
        rospy.loginfo("Calling 'ItemPlace' with data: itemID = %d, onRobot = %r, inHole = %r" % (itemID, onRobot, inHole))
        # data = ItemPlace()
        # data.itemID = itemID 
        # data.onRobot = onRobot
        # data.inHole = inHole
        # return 1, 0

        try:
            proxy = rospy.ServiceProxy('ItemPlace', ItemPlace)
            res = proxy(itemID, onRobot, inHole)
            return res
        except rospy.ServiceException:
            rospy.logerr("Service call failed")
