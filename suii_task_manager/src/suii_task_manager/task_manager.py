#! /usr/bin/env python
import rospy
from suii_msgs.srv import DistanceToGoal
from functools import cmp_to_key

from suii_mux_manager_comm.yaml_handler import YAMLHandler

from suii_mux_manager_comm.task_with_action import TaskWithAction 
from suii_mux_manager_comm.task import Task, TaskStatus
from suii_mux_manager_comm.task_list import TaskList
from suii_mux_manager_comm.action_list import ActionList
from suii_protocol.task_protocol import TaskProtocol
from suii_protocol.protocol.enum_task_action import TaskActionType
from suii_protocol.protocol.enum_task_type import TaskType
from suii_protocol.protocol.enum_location_identifier import LocationIdentifierType
from suii_protocol.protocol.enum_orientation import OrientationIdentifierType
from enum import Enum

EXIT_KEY = TaskProtocol.look_up_value(TaskProtocol.location_dict, LocationIdentifierType.EX.fullname)

## ===== Main class ==== ##
class TaskManager(object):
    MAX_HOLDING_CAPACITY = 3

    def __init__(self, holding_capacity=MAX_HOLDING_CAPACITY, yaml_path="", verbose=False):
        # Basic config
        self.holding_capacity = holding_capacity 
        self.yaml_path = yaml_path
        self.verbose = verbose
        self.MAX_REPLAN_ATTEMPTS = 10

        # Here come the lists
        self.task_list = TaskList()                 # The current one
        self.task_list_last_updated = TaskList()    # Last replanned task list

        self.replan_list = ActionList()             # Action list from mux
        self.error_index = -1                       # Error index

        # Output
        self.output_list = ActionList()             # The output list to send to mux

        self.task_list_type = -1                    # Type of the list (TRANSPORTATION/NAVIGATION)

        # Helpers
        self.transport_optimizer = TaskTransportOptimizer(holding_capacity, yaml_path, verbose)
        self.navigation_optimizer = TaskNavigationOptimizer(yaml_path, verbose)
        self.task_replanner = TaskReplanner(yaml_path, verbose)

    ## Call to initialize task_list when received data from refbox
    def initialize_list(self, task_list):
        # Save list type as the type of the first task
        self.task_list_type = task_list.task_list[0].type

        # Save the original + set the current one
        self.task_list_last_updated = task_list.make_duplicate()
        self.task_list = task_list.make_duplicate()

    ## Initializing necessary data and then calls helpers
    def optimize_transport(self):
        # Make duplicates so i won't lose the task_list
        self.transport_optimizer.reinitialize_input(self.task_list.make_duplicate())
        self.transport_optimizer.optimize()
        # Points to output
        self.output_list = self.transport_optimizer.output_list

    def optimize_navigation(self):
        # Make duplicates so i won't lose the task_list
        self.navigation_optimizer.reinitialize_input(self.task_list.make_duplicate())
        self.navigation_optimizer.optimize()
        # Points to output
        self.output_list = self.navigation_optimizer.output_list

    def call_replanner(self):
        self.task_replanner.reinitialize_input(self.replan_list, self.task_list_last_updated, self.error_index)
        success, additional_action = self.task_replanner.replan()
        if (not success):
            return False, AdditionalAction.NONE

        # The new task_list
        self.task_list = self.task_replanner.output_list.make_duplicate()
        # Save the last replanned ones
        self.task_list_last_updated = self.task_replanner.output_list.make_duplicate()
        return True, additional_action
    
    def handle_additional_action(self, additional_action, error_task):
        if (additional_action == AdditionalAction.DUMP_ITEM_ON_HAND):
            additional_actions = ActionList()
            rospy.logwarn("Dumping object [%s] to the current table" % (error_task.object_str))
            TaskFormatter.format_place_task(error_task, additional_actions.task_list)
            rospy.loginfo("Additional actions:")
            print(additional_actions)
            return additional_actions
        elif (additional_action == AdditionalAction.DUMP_ITEM_ON_HAND_AND_PICK_TO_CAP):
            additional_actions = ActionList()
            rospy.logwarn("Dumping object [%s] to the current table" % (error_task.object_str))
            TaskFormatter.format_place_task(error_task, additional_actions.task_list)
            current_cap = self.holding_capacity - self.task_replanner.items_on_hand
            rospy.logwarn("Current capacity is: ", current_cap)

            rospy.loginfo("Additional actions:")
            print(additional_actions)
            return additional_actions
        elif (additional_action == AdditionalAction.PICK_AND_DUMP_ON_TABLE):
            additional_actions = ActionList()
            rospy.logwarn("Picking object [%s] from ROBOT" % (error_task.object_str))
            TaskFormatter.format_pick_from_robot(error_task, additional_actions.task_list)
            rospy.logwarn("Dumpinggggg. Please ignore random destination.")
            error_task.set_destination(1) # Set a random destination so mux doesn't call precision place
            TaskFormatter.format_place_task(error_task, additional_actions.task_list)
            rospy.loginfo("Additional actions:")
            print(additional_actions)
            return additional_actions
        return None

    ## Interface that task_manager_handler calls to optimize list
    def optimize_list(self):
        if (self.task_list_type == int(TaskType.TRANSPORTATION)):
            self.optimize_transport()
            return True
        elif (self.task_list_type == int(TaskType.NAVIGATION)):
            self.optimize_navigation()
            return True
        return False

    ## Interface that task_manager_handler calls when received error from mux
    def replan(self):
        # Loggity log
        rospy.logwarn("Error @ index [%d] with task: " % self.error_index)
        rospy.logwarn(self.replan_list.task_list[self.error_index])

        # Get new task list
        rospy.loginfo("Making new task list...")
        success, additional_action = self.call_replanner()
        attempt = 1

        while (not success):
            if (attempt > self.MAX_REPLAN_ATTEMPTS):
                # we are slightly f*cked
                rospy.logerr("Max attempt reached. Cannot replan.")
                rospy.logerr("Please check if there is error case and error handling behavior defined for this error")
                rospy.logfatal("Abort mission.")
                return False
            rospy.logerr("Cannot replan. Will retry: %d/%d times" % (attempt, self.MAX_REPLAN_ATTEMPTS))
            success, additional_action = self.call_replanner()
            attempt = attempt + 1

        rospy.loginfo("New list made successfully!")

        # Optimize
        if (self.task_list_type == int(TaskType.TRANSPORTATION)):
            rospy.loginfo("Sending new task list to transport optimizer...")
            self.optimize_transport()
        elif (self.task_list_type == int(TaskType.NAVIGATION)):
            rospy.loginfo("Sending new task list to navigation optimizer...")
            self.optimize_navigation()

        # Handle additional actions
        rospy.loginfo(additional_action)
        if (additional_action != AdditionalAction.NONE):
            additional_action_list = self.handle_additional_action(additional_action, self.replan_list.task_list[self.error_index])
            if (additional_action_list != None):
                # I'm appending to the top of the list now
                self.output_list.task_list = additional_action_list.task_list + self.output_list.task_list
        
        # Loggity log
        rospy.loginfo("Replan finished")
        rospy.loginfo("##################")
        print(self.output_list)
        return True
    
    ## Clear all the lists
    def clear(self):
        self.task_list.clear_task()
        self.task_list_last_updated.clear_task()
        self.output_list.clear_task()

## ===== Service clients ===== ##
class DistanceToGoalClient:
    @staticmethod
    def call_serv(x, y):
        # rospy.loginfo("Waiting for service...")
        # rospy.wait_for_service('get_distance')
        try:
            # rospy.loginfo("Calling 'get_distance'")
            distance_to_goal = rospy.ServiceProxy('get_distance', DistanceToGoal)
            res = distance_to_goal(x, y)
            return res.distance
        except rospy.ServiceException:
            return -1

## ===== TaskFormatter ===== ##
# Input: Data to make a TaskWithAction, The list to append to
# Output: A TaskWithAction object appended to the list
class TaskFormatter(object):
    @staticmethod
    def format_drive(dest, result):
        TaskFormatter.format_move_to_drive(result)
        TaskFormatter.format_actual_drive(dest, result)

    @staticmethod
    def format_drive_with_orientation(dest, orientation, result):
        TaskFormatter.format_move_to_drive(result)
        twa = TaskWithAction()
        twa.set_destination(dest)
        twa.set_orientation(orientation)
        twa.set_action(int(TaskActionType.DRIVE))
        result.append(twa)

    @staticmethod
    def format_actual_drive(dest, result):
        twa = TaskWithAction()
        twa.set_destination(dest)
        twa.set_action(int(TaskActionType.DRIVE))
        result.append(twa)

    @staticmethod
    def format_pick_task(task, result):
        twa = TaskWithAction()
        # twa.copy_from_task(task)
        twa.set_object(task.object)
        twa.set_container(task.container)
        twa.set_action(int(TaskActionType.PICK))
        result.append(twa)
    
    @staticmethod
    def format_pick_from_robot(task, result):
        twa = TaskWithAction()
        # twa.copy_from_task(task)
        twa.set_object(task.object)
        twa.set_container(task.container)
        twa.set_action(int(TaskActionType.PICK_FROM_ROBOT))
        result.append(twa)

    @staticmethod
    def format_place_task (task, result):
        twa = TaskWithAction()
        # twa.copy_from_task(task)
        twa.set_object(task.object)
        twa.set_container(task.container)
        twa.set_action(int(TaskActionType.PLACE))
        result.append(twa)
    
    @staticmethod
    def format_place_to_robot(task, result):
        twa = TaskWithAction()
        # twa.copy_from_task(task)
        twa.set_object(task.object)
        twa.set_container(task.container)
        twa.set_action(int(TaskActionType.PLACE_TO_ROBOT))
        result.append(twa)

    @staticmethod
    def format_move_to_drive(result):
        twa = TaskWithAction()
        twa.set_action(int(TaskActionType.MOVE_TO_DRIVE))
        result.append(twa)

    @staticmethod
    def format_find_hole(task, result):
        twa = TaskWithAction()
        # twa.copy_from_task(task)
        twa.set_object(task.object)
        twa.set_container(task.container)
        twa.set_action(int(TaskActionType.FIND_HOLE))
        result.append(twa)

## ===== TaskSorter ===== ##
# Sort list based on distances (and other things, later)
# (This object makes service calls to update distances)
class TaskSorter(object):
    def __init__(self, yaml_path, verbose=False):
        self.yaml_handler = YAMLHandler(path=yaml_path)
        self.yaml_handler.load()
        self.verbose = verbose
    
    def cmp_distance_src_dest(self, t1, t2):
        # -1 appears first, then 0, then 1
        # sorting order:
        #   - distance from source (asc)
        #   - distance from dest (asc)
        if ((t1.source_dist < t2.source_dist) or 
            ((t1.source_dist == t2.source_dist) and (t1.dest_dist < t2.dest_dist))):
            return -1
        elif ((t1.source_dist == t2.source_dist) and (t1.dest_dist == t2.dest_dist)):
            return 0
        else:
            return 1

    def cmp_distance_dest(self, t1, t2):
        if (t1.dest_dist < t2.dest_dist):
            return -1
        elif (t1.dest_dist == t2.dest_dist):
            return 0
        return 1

    def update_distances(self, task_list):
        for item in task_list.task_list:
            # index of the data: orientation [w,x,y,z]  - position [x,y,z]
            # Dist. from source
            if self.verbose:
                rospy.loginfo("Updating source_dist for task @ '%s'", item.source_str)
                rospy.loginfo("Calling service with x=%f, y=%f" % (position[0], position[1]))
            orientation, position = self.yaml_handler.get_pose_for(item.source_str)
            if position != None:
                item.source_dist = DistanceToGoalClient.call_serv(position[0], position[1])
                if (item.source_dist == -1):
                    return False

            # Dist. from dest.
            if self.verbose:
                rospy.loginfo("Updating dest_dist for task @ '%s'", item.destination_str)
                rospy.loginfo("Calling service with x=%f, y=%f" % (position[0], position[1]))
            orientation, position = self.yaml_handler.get_pose_for(item.destination_str)
            if position != None:
                item.dest_dist = DistanceToGoalClient.call_serv(position[0], position[1])
                if (item.dest_dist == -1):
                    return False
        return True

    def sort_list_by_distance_src_dest(self, sort_list):
        if (self.update_distances(sort_list)):
            sort_list.task_list.sort(key=cmp_to_key(self.cmp_distance_src_dest))    
            return True
        return False

    def sort_list_by_distance_dest(self, sort_list):
        if (self.update_distances(sort_list)):
            sort_list.task_list.sort(key=cmp_to_key(self.cmp_distance_dest)) 
            return True   
        return False

## ===== TaskTransportOptimizer ===== ##
# Optimizer list of TRANSPORTATION tasks
# Precision placing included 
class TaskTransportOptimizer(object):
    def __init__(self, holding_capacity, yaml_path, verbose=False):
        self.holding_capacity = holding_capacity
        self.input_list = TaskList()
        self.output_list = ActionList()
        self.task_sorter = TaskSorter(yaml_path)
        self.verbose = verbose
    
    def reinitialize_input(self, input):
        self.input_list = input 
        self.output_list.clear_task()

    def prioritize_task_list(self, sort_list):
        if self.verbose: rospy.loginfo("Prioritizing task list...")
        # sort_list.sort_by_src_and_dest() # may not be necessary since we're sorting by distances
        self.task_sorter.sort_list_by_distance_src_dest(sort_list)

    def add_task_to_capacity (self, holding_list):
        if not holding_list.is_full():
            (status, index, task) = self.input_list.get_next_obj_to_pick()
            if not status: # picked everything
                return True 
            # check if source changed
            if (len(holding_list.task_list) != 0 and
                task.source != holding_list.task_list[0].source): 
                return True 
            holding_list.add_task(task) 
            self.input_list.task_list[index].status_to_scheduled()
            return False
        else:
            return True
    
    def pick_from_sources (self, task_list):
        unique_sources = task_list.get_unique_source()

        if (len(unique_sources) > 0):
            for key, value in unique_sources.items():
                # Drive to location (included the empty call to move arm)
                TaskFormatter.format_drive(key, self.output_list.task_list)      
                #  List of items to pick up there         
                pick_up = task_list.get_tasks_by_source(key)   
                # Schedule a pick 
                for task in pick_up.task_list:
                    if task.is_dest_same_as_src():
                        # Pick and place right back
                        TaskFormatter.format_pick_task(task, self.output_list.task_list)
                        TaskFormatter.format_place_task(task, self.output_list.task_list)
                        # Remove it from list so a drop-off will not be scheduled
                        self.input_list.remove_task(task) 
                    else:
                        # Pick and place to robot
                        TaskFormatter.format_pick_task(task, self.output_list.task_list)
                        TaskFormatter.format_place_to_robot(task, self.output_list.task_list)

    def drop_off_precision (self, drop_off_list):
        for task in drop_off_list.task_list:
            # Look for the hole
            TaskFormatter.format_find_hole(task, self.output_list.task_list)
            # Pick and place as usual
            TaskFormatter.format_pick_from_robot(task, self.output_list.task_list)
            TaskFormatter.format_place_task(task, self.output_list.task_list)

    def drop_off_normal (self, drop_off_list):   
        for task in drop_off_list.task_list:
            TaskFormatter.format_pick_from_robot(task, self.output_list.task_list)
            TaskFormatter.format_place_task(task, self.output_list.task_list) 

    def drop_off_at_destinations (self, task_list):
        unique_destinations = task_list.get_unique_destination()

        if len(unique_destinations) > 0:
            for key, value in unique_destinations.items():
                # Drive to location (included the empty call to move arm)
                TaskFormatter.format_drive(key, self.output_list.task_list)         
                # List of all items to drop off there   
                drop_off_list = task_list.get_tasks_by_destination(key)   
                # Schedule a drop off
                # LocationIdentifierType.PP = Type of the precision platform
                if (value == LocationIdentifierType.PP.fullname):
                    self.drop_off_precision(drop_off_list)
                else:
                    self.drop_off_normal(drop_off_list)

    def optimize(self):
        # Clear things
        holding_list = TaskList(capacity=self.holding_capacity)
        holding_list_ok = False

        self.prioritize_task_list(self.input_list)
        if self.verbose: print(self.input_list)

        # While there is still tasks to schedule
        while not self.input_list.is_empty():
            holding_list_ok = self.add_task_to_capacity(holding_list)

            if (holding_list_ok):
                # Schedule pick from the current source
                self.pick_from_sources(holding_list)
                # Schedule drop off for those items
                self.drop_off_at_destinations(holding_list)

                # Clean up holding list
                self.input_list.remove_task_in_list(holding_list.task_list)
                holding_list.clear_task()
                holding_list_ok = False # Reset flag
        
        # Check for any remaining items
        if not holding_list.is_empty():
            # Schedule pick from the current source
            self.pick_from_sources(holding_list)
            # Schedule drop off for those items
            self.drop_off_at_destinations(holding_list)

        # Clean up holding list
        self.input_list.remove_task_in_list(holding_list.task_list)
        holding_list.clear_task()

        TaskFormatter.format_drive(EXIT_KEY, self.output_list.task_list)  

        return True # well i should hope it doesn't error

## ===== TaskTransportOptimizer ===== ##
# Optimizer list of NAVIGATION tasks
class TaskNavigationOptimizer(object):
    def __init__(self, yaml_path, verbose=False):
        self.input_list = TaskList()
        self.output_list = ActionList()
        self.task_sorter = TaskSorter(yaml_path)
        self.verbose = verbose

    def reinitialize_input(self, input):
        self.input_list = input
        self.output_list.clear_task()

    def prioritize_task_list(self, sort_list):
        if self.verbose: rospy.loginfo("Prioritizing task list...")
        return self.task_sorter.sort_list_by_distance_dest(sort_list)

    def optimize(self):
        if not self.prioritize_task_list(self.input_list):
            rospy.logerr("Failed to sort by distance. Check 'get_distance' service.")
        if self.verbose: print(self.input_list)
        for item in self.input_list.task_list:
            TaskFormatter.format_drive_with_orientation(item.destination, item.orientation, self.output_list.task_list)
        TaskFormatter.format_drive(EXIT_KEY, self.output_list.task_list)  
        return True

## =============== NOT RELEVANT FOR THIS VERSION ==================== ##

## ===== ErrorCase Enum ===== ##
# To keep track of what went wrong this time
class ErrorCase(Enum):
    UNDEFINED = -1,
    CANNOT_PICK = 1,
    CANNOT_PICK_FROM_ROBOT = 2,
    CANNOT_PLACE = 3,
    CANNOT_PLACE_TO_ROBOT = 4,
    CANNOT_DRIVE = 5,
    CANNOT_MOVE_TO_DRIVE = 6,
    CANNOT_FIND_HOLE = 7

## ===== ErrorCase Enum ===== ##
# To keep track of additional actions
class AdditionalAction(Enum):
    NONE = 0,
    DUMP_ITEM_ON_HAND = 1,
    DUMP_ITEM_ON_HAND_AND_PICK_TO_CAP = 2,
    PICK_AND_DUMP_ON_TABLE = 3

## ===== TaskReplanner ===== ##
# Output a new TaskList() given the error list
# For TaskManager to make a new ActionList() 
class TaskReplanner(object):
    def __init__(self, yaml_path, verbose=False):
        self.input_list = ActionList()
        self.original_task_list = None
        self.in_progress_list = ActionList()
        self.output_list = TaskList()
        self.items_on_hand = -1
        self.task_sorter = TaskSorter(yaml_path)
        self.error_index = -1
        self.error_case = -1
        self.verbose = verbose

    def convert_task_to_actions(self, task):
        actions = ActionList()
        if (task.type == int(TaskType.NAVIGATION)):
            TaskFormatter.format_drive(task.destination, actions.task_list)
        elif (task.type == int(TaskType.TRANSPORTATION)):
            TaskFormatter.format_drive(task.source, actions.task_list)

            if (task.destination_str == LocationIdentifierType.PP.fullname): # PPT
                TaskFormatter.format_pick_task(task, actions.task_list)
                TaskFormatter.format_place_to_robot(task, actions.task_list)
                TaskFormatter.format_drive(task.destination, actions.task_list)
                TaskFormatter.format_find_hole(task, actions.task_list)
                TaskFormatter.format_pick_from_robot(task, actions.task_list)
                TaskFormatter.format_place_task(task, actions.task_list)
            else:
                if (task.is_dest_same_as_src()):
                    TaskFormatter.format_pick_task(task, actions.task_list)
                    TaskFormatter.format_place_task(task, actions.task_list)
                else:
                    TaskFormatter.format_pick_task(task, actions.task_list)
                    TaskFormatter.format_place_to_robot(task, actions.task_list)
                    TaskFormatter.format_drive(task.destination, actions.task_list)
                    TaskFormatter.format_pick_from_robot(task, actions.task_list)
                    TaskFormatter.format_place_task(task, actions.task_list)
        return actions

    def reinitialize_input(self, input, original_list, error_index):
        self.input_list = input
        self.original_task_list = original_list
        self.error_index = error_index
        self.output_list.clear_task()
        self.error_case = None
        self.items_on_hand = -1

    def check_error_case(self, task):
        if (task.action == int(TaskActionType.PICK)):
            return ErrorCase.CANNOT_PICK
        elif (task.action == int(TaskActionType.PICK_FROM_ROBOT)):
            return ErrorCase.CANNOT_PICK_FROM_ROBOT
        elif (task.action == int(TaskActionType.PLACE)):
            return ErrorCase.CANNOT_PLACE    
        elif (task.action == int(TaskActionType.PLACE_TO_ROBOT)):
            return ErrorCase.CANNOT_PLACE_TO_ROBOT
        elif (task.action == int(TaskActionType.DRIVE)):
            return ErrorCase.CANNOT_DRIVE
        elif (task.action == int(TaskActionType.MOVE_TO_DRIVE)):
            return ErrorCase.CANNOT_MOVE_TO_DRIVE
        elif (task.action == int(TaskActionType.FIND_HOLE)):
            return ErrorCase.CANNOT_FIND_HOLE 
        return ErrorCase.UNDEFINED

    def not_done_tasks(self, todo_list, done_list):
        not_done = ActionList()
        for item in todo_list:
            if not (item in done_list):
                not_done.task_list.append(item) 
        return not_done

    def check_success(self):
        rospy.loginfo("Original list")
        print(self.original_task_list)

        successful_actions = self.input_list.task_list[:(self.error_index-1)]

        rospy.loginfo("Successfully done:")

        for task in self.original_task_list.task_list:
            task_actions = self.convert_task_to_actions(task)
            not_done = self.not_done_tasks(task_actions.task_list, successful_actions)
                    
            if (len(not_done.task_list) == 0):
                print(task)
                self.original_task_list.remove_task(task)

        rospy.loginfo("To do:")
        print(self.original_task_list)
    
    def check_in_progress(self, check_list):
        successful_actions = self.input_list.task_list[:(self.error_index-1)]

        for task in check_list.task_list:
            task_actions = self.convert_task_to_actions(task)
            not_done = self.not_done_tasks(task_actions.task_list, successful_actions)
            for item in not_done.task_list:
                self.in_progress_list.task_list.append(item)

        rospy.loginfo("Leftovers:")
        print(self.in_progress_list)

    def count_current_items_on_back(self):
        successful_actions = self.input_list.task_list[:(self.error_index-1)]
        sum_item = 0
        for item in successful_actions:
            if (item.action == int(TaskActionType.PLACE_TO_ROBOT)):
                sum_item = sum_item + 1
            elif (item.action == int(TaskActionType.PLACE)):
                sum_item = sum_item - 1
        self.items_on_hand = sum_item 
        rospy.loginfo("Nr. of items on back: %d" % self.items_on_hand)

    def replan(self):
        # THIS FEATURE IS IN PROGRESS
        return True, AdditionalAction.NONE 
        
        # Check success
        self.count_current_items_on_back()
        self.check_success()

        # Check error
        error_task = self.input_list.task_list[self.error_index]
        self.error_case = self.check_error_case(error_task)
        rospy.logwarn(self.error_case)

        # Make a duplicate of the original list
        # Then later remove the unnecessary ones
        self.output_list = self.original_task_list.make_duplicate()
        if (self.error_case == ErrorCase.CANNOT_PICK or
            self.error_case == ErrorCase.CANNOT_PICK_FROM_ROBOT):
            rospy.logwarn('Error object: [%d] - %s' % (error_task.object, error_task.object_str))
            rospy.logwarn('Removing all tasks related to that object...')  
            self.remove_all_tasks_with_obj(error_task.object, self.output_list)
            rospy.loginfo("Final todo list:")
            print(self.output_list)
            rospy.loginfo("Checking leftovers:")
            self.check_in_progress(self.output_list)
            return True, AdditionalAction.NONE
        
        # elif (self.error_case == ErrorCase.CANNOT_PLACE):
        #     rospy.logwarn('Error object: [%d] - %s' % (error_task.object, error_task.object_str))
        #     # God i hope this does not happen
        #     return True, AdditionalAction.NONE
        
        # elif (self.error_case == ErrorCase.CANNOT_PLACE_TO_ROBOT):
        #     rospy.logwarn('Error object: [%d] - %s' % (error_task.object, error_task.object_str))
        #     rospy.logwarn('Removing all tasks related to that object...')  
        #     self.remove_all_tasks_with_obj(error_task.object, self.output_list)
        #     rospy.loginfo("Task list after removing")
        #     print(self.output_list)
        #     return True, AdditionalAction.DUMP_ITEM_ON_HAND
        
        # elif (self.error_case == ErrorCase.CANNOT_DRIVE):
        #     rospy.logwarn('Error location: [%d] - %s' % (error_task.destination, error_task.destination_str))
        #     rospy.logwarn('Removing all tasks related to that location...')  
        #     self.remove_all_tasks_with_loc(error_task.destination, self.output_list)
        #     rospy.loginfo("Task list after removing")
        #     print(self.output_list)
        #     return True, AdditionalAction.NONE 
        
        # elif (self.error_case == ErrorCase.CANNOT_FIND_HOLE):
        #     rospy.logwarn('Error object: [%d] - %s' % (error_task.object, error_task.object_str))
        #     rospy.logwarn('Removing all tasks related to that object...')  
        #     self.remove_all_tasks_with_obj(error_task.object, self.output_list)
        #     rospy.loginfo("Task list after removing")
        #     print(self.output_list)
        #     return True, AdditionalAction.PICK_AND_DUMP_ON_TABLE
        return False, AdditionalAction.NONE # undefined error case

    def remove_all_tasks_with_loc(self, destID, remove_list):
        src_tasks = remove_list.get_tasks_by_source(destID)
        dest_tasks = remove_list.get_tasks_by_destination(destID)
        remove_list.remove_task_in_list(src_tasks.task_list)
        remove_list.remove_task_in_list(dest_tasks.task_list)
    
    def remove_all_tasks_with_obj(self, objectID, remove_list):
        obj_tasks = remove_list.get_tasks_by_object(objectID)
        remove_list.remove_task_in_list(obj_tasks.task_list)
