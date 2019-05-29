#! /usr/bin/env python
from suii_task_manager.task_with_action import TaskWithAction 
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
        twa.set_action(TaskProtocol.look_up_value(TaskProtocol.task_action_dict, "DRIVE"))
        result.append(twa)
        return result

    def format_pick_task(self, task, result):
        twa = TaskWithAction()
        twa.copy_from_task(task)
        twa.set_action(TaskProtocol.look_up_value(TaskProtocol.task_action_dict, "PICK"))
        result.append(twa)
        return result

    def format_pick (self, task_list, result):
        for task in task_list.task_list:
            self.format_pick_task(task, result)
        return result

    def format_place_task (self, task, result):
        twa = TaskWithAction()
        twa.copy_from_task(task)
        twa.set_action(TaskProtocol.look_up_value(TaskProtocol.task_action_dict, "PLACE"))
        result.append(twa)
        return result

    def format_place (self, task_list, result):
        for task in task_list.task_list:
            self.format_place_task(task, result)
        return result
        
    def format_set_of_tasks (self, task_list, result):
        unique_sources = task_list.get_unique_source()

        if (len(unique_sources) > 0):
            for key, value in unique_sources.items():
                self.format_drive(key, result) # drive to first location
                pick_up = task_list.get_tasks_by_source(key)    # get all the tasks with that source

                # Pick up selectively
                for task in pick_up.task_list:
                    if task.is_dest_same_as_src():
                        self.format_pick_task(task, result)
                        self.format_place_task(task, result)
                        task_list.remove_task(task)
                    else:
                        self.format_pick_task(task, result)
    
        unique_destinations = task_list.get_unique_destination()

        if len(unique_destinations) > 0:
            for key, value in unique_destinations.items():
                self.format_drive(key, result)                    # drive to first location
                drop_off_list = task_list.get_tasks_by_destination(key)   # get all the tasks with that dest
                self.format_place(drop_off_list, result) 
        return result

    def optimize_list(self, task_list, result):
        holding_list = TaskList(self.holding_capacity)
        format_now   = False
        task_list.sort_by_src_and_dest()
        # task_list.print_task_list()
        
        while not task_list.is_empty():
            if not holding_list.is_full():
                (status, index, task) =  task_list.get_next_obj_to_pick()
                # task  = Task()
                # index = 0
                if not status: # picked everything
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

        return result