#! /usr/bin/env python
from suii_task_manager.protocol.enum_task_action import TaskActionType
from suii_task_manager.task_with_action import TaskWithAction 
from suii_task_manager.protocol.enum_task_action import TaskActionType
from suii_task_manager.task import Task
from suii_task_manager.task_list import TaskList
from suii_task_manager.task_protocol import TaskProtocol

class TaskManager:
    MAX_HOLDING_CAPACITY = 3

    def __init__(self, holding_capacity=MAX_HOLDING_CAPACITY):
        self.protocol = TaskProtocol()
        self.holding_capacity = holding_capacity
        
    def format_drive(self, dest, result):
        twa = TaskWithAction()
        twa.set_destination(dest)
        twa.set_action(TaskActionType.DRIVE)
        result.append(twa)

    def format_pick_task(self, task, result):
        twa = TaskWithAction()
        twa.copy_from_task(task)
        twa.set_action(TaskActionType.PICK)
        result.append(twa)

    def format_pick (self, task_list, result):
        for task in task_list.task_list:
            self.format_pick_task(task, result)

    def format_place_task (self, task, result):
        twa = TaskWithAction()
        twa.copy_from_task(task)
        twa.set_action(TaskActionType.PLACE)
        print("Placing: " + twa)
        result.append(twa)

    def format_place (self, task_list, result):
        for task in task_list.task_list:
            self.format_place_task(task, result)
        
    def format_set_of_tasks (self, task_list, result):
        unique_task_list = task_list.get_unique_source()

        if (len(task_list) > 0):
            print("\nPicking objects from " + str(len(unique_task_list)) + " source(s)\n")
            
            for key, value in unique_task_list:
                print("\nPicking objects from " + str(value) + "\n")
                self.format_drive(key, result) # drive to first location

                pick_up = task_list.get_tasks_by_source(key)    # get all the tasks with that source

                # Pick up selectively
                for task in pick_up.task_list:
                    if task.is_dest_same_as_src():
                        print("Object has the same destination as source!\n")
                        self.format_pick_task(task, result)
                        self.format_place_task(task, result)
                        task_list.remove_task(task)
                    else:
                        self.format_pick_task(task, result)
    
        unique_dest_list = task_list.get_unique_destination()

        if len(unique_dest_list) > 0:
            print("\nPlacing objects at " + str(len(unique_dest_list)) + " destination(s)\n")
            for key, value in unique_dest_list:
                print("\nPlacing objects at " + value + "...\n")
                self.format_drive(key, result)                    # drive to first location
                drop_off_list = task_list.get_tasks_by_dest(key)   # get all the tasks with that dest
                self.format_place(drop_off_list, result)  


    def optimize_list(self, task_list, result):
        holding_list = TaskList(self.holding_capacity)
        format_now   = False
        print("Sorting task list by source and destination...\n")
        task_list.sort_by_src_and_dest()
        task_list.print_task_list()
        
        while not task_list.is_empty():
            if not holding_list.is_full():
                task  = Task()
                index = 0
                if not task_list.get_next_obj_to_pick(task, index): # picked everything
                    break 

                if holding_list.is_empty():
                    holding_list.add_task(task) 
                    task_list.task_list[index].picked = True
                else: 
                    if (task.source == holding_list.task_list[0].source):
                        holding_list.add_task(task) 
                        task_list.task_list[index].picked = True
                    else:
                        format_now = True
            else: 
                format_now = True


            if (format_now):
                self.format_set_of_tasks(holding_list, result)

                for task in holding_list.task_list:
                    task_list.remove_task(task)
                holding_list.clear_task()
                format_now = False

        if not holding_list.is_empty():
            self.format_set_of_tasks(holding_list, result)

        for task in holding_list.task_list:
            task_list.remove_task(task)
        holding_list.clear_task()

        return True