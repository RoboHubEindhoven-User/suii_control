#!/usr/bin/env python

'''bmt_state.py: 
'''

__author__      = "Zinkeng Thierry"

import rospy
from actionlib import SimpleActionClient, GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8
from suii_control.state_machines import *
from suii_control.utils.light_mode import LightMode
from enum import Enum
import time
from yaml import dump

class BNT_StateName(Enum):
    TASK = "task_state"
    MOVE = "move_state"
    WAIT = "wait_state"
    EXIT = "exit_state"
    EMRG = "emergency_state" 

class BNT_TransTo(Enum):
    TASK = "to_task"
    MOVE = "to_move"
    WAIT = "to_wait"
    EXIT = "to_exit"
    EMRG = "to_emergency" 

class TaskState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm, navigation_tasks, current_tasks, last_tasks):
        super(TaskState, self).__init__(fsm)
        self.navigation_tasks = navigation_tasks
        self.current_tasks    = current_tasks
        self.last_tasks       = last_tasks
        self.current_tasks_nr = 0
        self.nr_of_task       = 0
        
    def enter(self):
        super(TaskState, self).enter()
        self.nr_of_task = len(self.navigation_tasks)
    
    def execute(self):
        super(TaskState, self).execute()

        if(self.navigation_tasks):
            self.current_tasks.append(self.navigation_tasks[0])
            self.current_tasks_nr += 1
            rospy.loginfo("Current Task Number: " + str(self.current_tasks_nr))
            del self.navigation_tasks[0]
            if not self.navigation_tasks:
                self.current_tasks_nr = 0
            print dump(self.current_tasks)
            self.fsm.transition(BNT_TransTo.MOVE)
        else:
            while(self.current_tasks):
                self.current_tasks.pop(0)

    def exit(self):
        super(TaskState, self).exit()

class MoveState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm, current_tasks):
        super(MoveState, self).__init__(fsm)
        self.current_tasks      = current_tasks
        self.task               = None
        self.next_goal          = MoveBaseGoal()
        self.move_base_client   = SimpleActionClient('/move_base', MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        # allow up to 5 seconds for the action server to come up
        # self.move_base_client.wait_for_server(rospy.Duration(5))
        
    def enter(self):
        super(MoveState, self).enter()
        self.task = self.current_tasks[0]
    
    def execute(self):
        super(MoveState, self).execute()

        self.next_goal.target_pose.header.frame_id    = "/map"
        self.next_goal.target_pose.header.stamp       = rospy.Time.now()
        self.next_goal.target_pose.pose.position.x    = self.task.destination.position.x
        self.next_goal.target_pose.pose.position.y    = self.task.destination.position.y
        self.next_goal.target_pose.pose.orientation.z = self.task.destination.orientation.z
        self.next_goal.target_pose.pose.orientation.w = self.task.destination.orientation.w

        # # rospy.loginfo("moving to goal pos X: " + str(self.task.destination.position.x) + ", pos Y: " + str(self.task.destination.position.y))
        # # start moving
        # self.move_base_client.send_goal(self.next_goal)

        # # allow Faraday up to 60 seconds to complete task
        # status = self.move_base_client.wait_for_result(rospy.Duration(60))

        # if not status:
        #     self.move_base_client.cancel_goal()
        #     rospy.logerr("SUII base failed to move to it's goal for some reason!")
        #     # self.fsm.transition("to_exit")
        # else:
        #     # We made it!
        #     state = self.move_base_client.get_state()
        #     if state == GoalStatus.SUCCEEDED:
        #         rospy.loginfo("SUII moved to it's goal successfully!")
        #         self.fsm.transition(BNT_TransTo.WAIT)
        self.fsm.transition(BNT_TransTo.WAIT)

    def exit(self):
        super(MoveState, self).exit()

    
class WaitState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm, current_tasks, last_tasks):
        super(WaitState, self).__init__(fsm)
        self.light_pub     = rospy.Publisher("/command_e_stop", Int8, queue_size=10)
        self.light_msg     = Int8()
        self.current_tasks = current_tasks
        self.last_tasks    = last_tasks
        
    def enter(self):
        super(WaitState, self).enter()
    
    def execute(self):
        super(WaitState, self).execute()
        # Set animation lights
        self.light_msg.data = int(LightMode.POLICE)
        self.light_pub.publish(self.light_msg)
        # print(str(self.current_tasks[0].wait_time.data.secs))
        time.sleep(self.current_tasks[0].wait_time.data.secs)

        # self.fsm.transition(BNT_TransTo.EXIT)

         # Reset animation lights
        self.light_msg.data = int(LightMode.NONE)
        self.light_pub.publish(self.light_msg)

        while(self.current_tasks):
            self.current_tasks.pop(0)
        
        self.fsm.transition(BNT_TransTo.TASK)
        
    def exit(self):
        super(WaitState, self).exit()

class ExitState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(ExitState, self).__init__(fsm)
        
    def enter(self):
        super(ExitState, self).enter()
    
    def execute(self):
        super(ExitState, self).execute()
        self.fsm.transition(BNT_TransTo.MOVE)

    def exit(self):
        super(ExitState, self).exit()

class EmergencyStopState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(EmergencyStopState, self).__init__(fsm)
        
    def enter(self):
        super(EmergencyStopState, self).enter()
    
    def execute(self):
        super(EmergencyStopState, self).execute()
        self.fsm.transition(BNT_TransTo.MOVE)

    def exit(self):
        super(EmergencyStopState, self).exit()

#********************************************************************************************
from yaml import dump
class BasicNavigationState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm, navigation_tasks, test_completed):
        super(BasicNavigationState, self).__init__(fsm)
        
        self.sub_fsm = StateMachine(self);
        self.rate = rospy.Rate(10)
        self.navigation_tasks = navigation_tasks
        self.current_tasks = []
        self.last_tasks   = []
        self.test_completed = test_completed
        self.end_task      = None
        # States
        self.sub_fsm.add_state(BNT_StateName.TASK, TaskState(self.sub_fsm, navigation_tasks, self.current_tasks, self.last_tasks))
        self.sub_fsm.add_state(BNT_StateName.MOVE, MoveState(self.sub_fsm, self.current_tasks))
        self.sub_fsm.add_state(BNT_StateName.WAIT, WaitState(self.sub_fsm, self.current_tasks, self.last_tasks))
        self.sub_fsm.add_state(BNT_StateName.EMRG, EmergencyStopState(self.sub_fsm))

        # Transitions
        self.sub_fsm.add_transition(BNT_TransTo.TASK, Transition(BNT_StateName.TASK))
        self.sub_fsm.add_transition(BNT_TransTo.MOVE, Transition(BNT_StateName.MOVE))
        self.sub_fsm.add_transition(BNT_TransTo.WAIT, Transition(BNT_StateName.WAIT))
        self.sub_fsm.add_transition(BNT_TransTo.EMRG, Transition(BNT_StateName.EMRG))
        
        # set default state
        self.sub_fsm.set_state(BNT_StateName.TASK)
        
    def enter(self):
        super(BasicNavigationState, self).enter()
    
    def execute(self):
        super(BasicNavigationState, self).execute()
        # print 'sssssssssssss: ' + dump(self.navigation_tasks)
        # self.end_task      = self.navigation_tasks[4]
        print("Nav ------------------: " + str(len(self.navigation_tasks)))
        self.end_task      = self.navigation_tasks[len(self.navigation_tasks) - 1]
        while True:
            self.sub_fsm.execute()
            if (self.current_tasks):
                if (self.current_tasks[0] == self.end_task):
                    # break
                    self.test_completed.append("Completed")
                    # self.test_completed.append(True)
                    break
        
        self.fsm.transition(Trans_To.TSL)

    def exit(self):
        super(BasicNavigationState, self).exit()