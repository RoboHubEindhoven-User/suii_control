#! /usr/bin/env python
 
from functools import cmp_to_key
from task import Task

class TaskList(object):
    DEFAULT_CAPACITY   = 20 

    def __init__(self, capacity = -1):
        self.capacity   = self.DEFAULT_CAPACITY if (capacity == -1) else capacity
        self.task_list  = []

    def is_empty(self):
        return len(self.task_list) == 0

    def is_full(self):
        return len(self.task_list) == self.capacity

    def clear_task(self):
        del self.task_list[:]

    def add_task (self, task):
        if not self.is_full():
            tmp_task = task.make_duplicate()
            self.task_list.append(tmp_task)
            return True
        return False

    def remove_task(self, task):
        for item in self.task_list:
            if item == task:
                self.task_list.remove(item)
                return True 
        return False 

    def remove_task_in_list(self, task_list):
        for item in task_list:
            self.remove_task(item)

    def get_next_obj_to_pick(self):
        for i in range(0, len(self.task_list)):
            if self.task_list[i].can_be_picked():
                task = self.task_list[i].make_duplicate()
                return True, i, task     
        return False, -1, None

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
        # Destination Dict {ID: String}
        unique_dest_list = dict()
        for task in self.task_list:
            if (task.destination == -1):
                continue
            unique_dest_list[task.destination] = task.destination_str
        return unique_dest_list

    def get_unique_source(self):
        unique_src_list = dict()
        for task in self.task_list:
            if (task.source == -1):
                continue
            unique_src_list[task.source] = task.source_str
        return unique_src_list

    def sort_by_src_and_dest(self):
        self.task_list.sort(key=cmp_to_key(self.cmp_items))

    def cmp_items(self, t1, t2):
        if ((t1.source < t2.source) or ((t1.source == t2.source) and (t1.destination < t2.destination))):
            return -1
        elif ((t1.source == t2.source) and (t1.destination == t2.destination)):
            return 0
        else:
            return 1

    def __str__(self):
        retstr = ''
        for task in self.task_list:
            retstr += repr(task) + '\n'
        return retstr
        
    def make_duplicate(self):
        dup = TaskList()
        for item in self.task_list:
            dup.add_task(item)
        return dup