#! /usr/bin/env python
 
from functools import cmp_to_key
from task import Task

class TaskList:
    DEFAULT_CAPACITY   = 20 

    def __init__(self):
        self.capacity   = self.DEFAULT_CAPACITY
        self.task_list  = []
    
    def get_task_index(self, task_type, source, destination, obj, container):
        task = Task()
        task.set_type(task_type)
        task.set_source(source)
        task.set_destination(destination)
        task.set_object(obj)
        task.set_container(container)
        
        for i in range(0, len(self.task_list)):
            if (self.task_list[i].is_equal_to(task)):
                return i
                
        return -1

    def is_empty(self):
        return len(self.task_list) == 0

    def is_full(self):
        return len(self.task_list) == self.capacity

    def clear_task(self):
        self.task_list = []

    def add_task (self, task):
        if not self.is_full():
            # Thanh question: can we append task directly w/o creating tmp? (memory issues?)
            tmp_task = Task()
            tmp_task.set_type(task.type)
            tmp_task.set_source(task.source)
            tmp_task.set_destination(task.destination)
            tmp_task.set_object(task.object)
            tmp_task.set_container(task.container)
            self.task_list.append(tmp_task)
            return True
        return False

    def remove_task(self, task):
        index = self.get_task_index(task.type, task.source, task.destination, task.object, task.container)
        if (index == -1):
            return False
        del self.task_list[index]
        return True

    def get_next_obj_to_pick(self):
        task = Task()
        for i in range(0, len(self.task_list)):
            if not self.task_list[i].picked:
                task.set_type(self.task_list[i].type)
                task.set_source(self.task_list[i].source)
                task.set_destination(self.task_list[i].destination)
                task.set_object(self.task_list[i].object)
                task.set_container(self.task_list[i].container)
                return True, i, task
                
        return False

    def get_tasks_by_source(self, source):
        task_list  = TaskList()
        for task in self.task_list:
            if (task.source == source):
                task_list.add_task(task)
        return task_list

    def get_tasks_by_destination(self, destination):
        task_list = TaskList()
        for task in self.task_list:
            if (task.destination == destination):
                task_list.add_task(task)
        return task_list

    def get_unique_destination(self):
        unique_dest_list = dict()
        for task in self.task_list:
            unique_dest_list[task.destination] = task.destination_str
        return unique_dest_list

    def get_unique_source(self):
        unique_src_list = dict()
        for task in self.task_list:
            unique_src_list[task.source] = task.source_str
        return unique_src_list

    def sort_by_src_and_dest(self):
        self.task_list.sort(key=cmp_to_key(self.cmp_items))

    def cmp_items(self, t1, t2):
        return (t1.source < t2.source) or (t1.source == t2.source and (t1.destination < t2.destination))
        
    def print_task_list(self):
        for task in self.task_list:
            print(task)
        # print([item.name for item in items]) ????