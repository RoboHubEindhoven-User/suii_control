#! /usr/bin/env python
from atwork_ros_msgs.msg._BenchmarkState import *
from atwork_ros_msgs.msg._BenchmarkScenario import *
from atwork_ros_msgs.msg._Inventory import *
from atwork_ros_msgs.msg._TriggeredConveyorBeltStatus import *
from atwork_ros_msgs.msg._TaskInfo import *
from suii_task_manager.benchmark import *
from suii_control_msgs.msg import *
from suii_task_manager.geometry import * 
from utils.constants import PickPlaceDirection, HolderTray
from std_msgs.msg import String
from yaml import dump, load

class TaskManager:
    def __init__(self):
        
        rospy.loginfo("initializing Task Manager...")

        rospy.Subscriber("/suii_refbox_client/benchmark_state", BenchmarkState, self.benchmark_callback)

        rospy.Subscriber("/suii_refbox_client/conveyor_belt_status", TriggeredConveyorBeltStatus, self.conveyor_callback)

        rospy.Subscriber("/suii_refbox_client/inventory", Inventory, self.inventory_callback)

        rospy.Subscriber("/suii_refbox_client/task_info", TaskInfo, self.task_callback)

        self.task_pub = rospy.Publisher("/suii_task_manager/tasks", String, queue_size=1)

        rospy.loginfo("Refbox node initializing complete!")


        self.geo             = Geometry()
        self.benchmark      = BenchmarkState()
        self.conveyor_belt  = TriggeredConveyorBeltStatus()
        self.inventory      = Inventory()
        self.task_info      = TaskInfo()
        self.tasks_received  = False
        self.task_msg        = None
        self.scenario        = None
        self.holder_count    = 1

    def benchmark_callback(self, benchmark_state_msg):
        self.benchmark = benchmark_state_msg
        self.scenario  = self.benchmark.scenario.type.data
        # rospy.loginfo("benchmark received, Description: " + self.benchmark.scenario.description.data)

    def conveyor_callback(self, conveyor_belt_status_msg):
        # rospy.loginfo("Triggered Conveyor Belt Status received")
        self.conveyor_belt = conveyor_belt_status_msg

    def inventory_callback(self, inventory_msg):
        # rospy.loginfo("Inventory received")
        # rospy.loginfo("Test: " + str(self.benchmark.scenario.type.data) + ", Description: " + self.benchmark.scenario.description.data)
        self.inventory = inventory_msg

    def task_callback(self, task_info_msg):
        # rospy.loginfo("Task Info received")
        self.task_info = task_info_msg
        

    def shutdown_hook(self):
        print("@work Receiver shutting down...")
    
    def get_benchmark_state(self):
        phase_type = BenchmarkPhaseProp.PREPARATION
        state_type = BenchmarkStateProp.STOPPED

        if(self.benchmark.phase.data == BenchmarkState.EXECUTION):
            phase_type = BenchmarkPhaseProp.EXECUTION
        if(self.benchmark.phase.data == BenchmarkState.CALIBRATION):
            phase_type = BenchmarkPhaseProp.CALIBRATION
        if(self.benchmark.phase.data == BenchmarkState.PREPARATION):
            phase_type = BenchmarkPhaseProp.PREPARATION


        if(self.benchmark.state.data == BenchmarkState.RUNNING):
            state_type = BenchmarkStateProp.RUNNING
        if(self.benchmark.state.data == BenchmarkState.PAUSED):
            state_type = BenchmarkStateProp.PAUSED
        if(self.benchmark.state.data == BenchmarkState.FINISHED):
            state_type = BenchmarkStateProp.FINISHED
        if(self.benchmark.state.data == BenchmarkState.STOPPED):
            state_type = BenchmarkStateProp.STOPPED

        return phase_type, state_type


    def get_optimized_tasks(self):
        task_list  = []
        recv_tasks = self.task_msg.tasks
        move_task = NavigationTask()
        count = 0

        if (self.scenario == BNT or self.scenario == NAVIGATION):
            for i in range(len(recv_tasks)):
                move_task = self.prepare_move_task(recv_tasks[i].navigation_task.location, recv_tasks[i].navigation_task.wait_time)
                task_list.append(move_task)
        else:
            pick_task = PickTask()
            place_task = PlaceTask()
            task_dict = []
            # prepare tasks at source service area 

            for i in range(len(recv_tasks)):
                if not service_area_in_list(task_list, recv_tasks.transportation_task.source.type.data):
                    task_list.append(self.prepare_move_task(recv_tasks[i].transportation_task.source))
                task_list.append(self.prepare_pick_task(recv_tasks[i], i, PickPlaceDirection.SERVICE_AREA_ROBOT)) 
                task_list.append(self.prepare_place_task(recv_tasks[i], i, PickPlaceDirection.SERVICE_AREA_ROBOT))                task_list.append(move_task)
    `       
            # prepare tasks at destination service area 
            for i in range(len(recv_tasks)):
                if not service_area_in_list(task_list, recv_tasks[i].transportation_task.destination.type.data):
                    task_list.append(self.prepare_move_task(recv_tasks[i].transportation_task.destination))
                task_list.append(self.prepare_pick_task(recv_tasks[i], i, PickPlaceDirection.ROBOT_SERVICE_AREA)) 
                task_list.append(self.prepare_place_task(recv_tasks[i], i, PickPlaceDirection.ROBOT_SERVICE_AREA))                task_list.append(move_task)
    `       
            # prepare tasks in the format: Source ( 1 move, max 3 pick-place), Destination ( 1 move, max 3 pick-place)
            #loop
            #if nav task, add
            #remove from task_list
            #if pick or place, count < 3
            #remove from task_list

            # re-organize tasks`
            temp_task_list = []
            for i in range(len(task_list)):
                # get navigation task
                if (type(task_list[i]) == NavigationTask)
                    temp_task_list.append(task_list[i])
                # get all pick and place task at service area
                # get pick task
                # get place task
                
            task_list = temp_task_list
        return task_list
        
    def service_area_in_list(self, task_list, service_area) {
        for i in range(len(task_list)):
            if task_list[i].location.type == service_area:
                return True
        return False
    }

    def prepare_move_task(self, location, wait_time = None):
        move_task = NavigationTask()
        pose      = None
        
        move.location.type        = location.type.data
        move.location.instance_id = location.instance_id.data
        move.location.description = location.description.data
        pose    = self.geo.get_waypoint_pose(move.location.type, move.location.instance_id)
        if(wait_time)
            pose.orientation  = self.geo.get_orientation(self.task_msg.tasks[i].navigation_task.orientation.data)
            move.time = wait_time

        move.pose.position.x    = pose.position.x
        move.pose.position.y    = pose.position.y
        move.pose.position.z    = pose.position.z
        move.pose.orientation.x = pose.orientation.x
        move.pose.orientation.y = pose.orientation.y
        move.pose.orientation.z = pose.orientation.z
        move.pose.orientation.w = pose.orientation.w
        
        return move_task

    def prepare_pick_task(self, task, group_id, direction):
        pick_task = PickTask()
        pick_task.object.type        = task.transportation_task.object.type.data
        pick_task.object.type_id     = task.transportation_task.object.type_id.data
        pick_task.object.instance_id = task.transportation_task.object.instance_id.data
        pick_task.object.description = task.transportation_task.object.description.data
        pick_task.object.group_id    = i
        pick_task.direction          = direction
        return pick_task

    def prepare_place_task(self, task, group_id, direction):
        place_task = PlaceTask()
        place_task.object.type        = task.transportation_task.object.type.data
        place_task.object.type_id     = task.transportation_task.object.type_id.data
        place_task.object.instance_id = task.transportation_task.object.instance_id.data
        place_task.object.description = task.transportation_task.object.description.data
        place_task.object.group_id    = i
        place_task.direction          = direction
        if (direction == PickPlaceDirection.ROBOT_SERVICE_AREA):
            if (task.transportation_task.container.type.data != 0 and task.transportation_task.container.type_id.data != 0)
                place_task.holder.type        = task.transportation_task.container.type.data
                place_task.holder.type_id     = task.transportation_task.container.type_id.data
                place_task.holder.instance_id = task.transportation_task.container.instance_id.data
                place_task.holder.description = task.transportation_task.container.description.data
        else:
            place_task.holder.type    = 100
            place_task.holder.type_id = self.holder_count
            if(self.holder_count == 3):
                self.holder_count = 1
            else:
                self.holder_count += 1

        return place_task
    

    def publish_tasks(self):
        tasks_received = False
        
        (phase_type, state_type) = self.get_benchmark_state()

        # if((state_type == BenchmarkStateProp.RUNNING) and (phase_type == BenchmarkPhaseProp.PREPARATION)):
        #     while(self.test_completed):
        #         self.test_completed.pop(0)
                
        if((state_type != BenchmarkStateProp.RUNNING) and (phase_type != BenchmarkPhaseProp.EXECUTION)):
            return

        if (self.atwork_receiver.tasks_received):
            tasks_received = True
            self.tasks = self.atwork_receiver.get_tasks()
            self.atwork_receiver.reset_tasks_received()

        if (self.gui_receiver.tasks_received):
            tasks_received = True
            self.tasks = self.gui_receiver.get_tasks()
            self.gui_receiver.reset_tasks_received()

        if (tasks_received): 
            task_list = self.get_optimized_tasks()
            msg = String()
            msg.data = dump(task_list)
            self.task_pub.publish(msg)
        else:
            if __debug__:
                rospy.loginfo("Waiting for Tasks")
        
        
