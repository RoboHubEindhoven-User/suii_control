#! /usr/bin/env python

from task_protocol import TaskProtocol
from task import Task
from task_with_action import TaskWithAction
from task_list import TaskList

print("PROTOCOL TEST")
print("Action")
TaskProtocol.print_protocol(TaskProtocol.task_action_dict)
print("Type")
TaskProtocol.print_protocol(TaskProtocol.task_type_dict)
print("Location")
TaskProtocol.print_protocol(TaskProtocol.location_dict)
print("Container")
TaskProtocol.print_protocol(TaskProtocol.container_dict)
print("Object")
TaskProtocol.print_protocol(TaskProtocol.object_dict)

print("")
lookupstr = "TEST_PROTOCOL"
TaskProtocol.add_key_value(TaskProtocol.location_dict, 15, lookupstr)
print("Look up ", lookupstr, " gets: ", TaskProtocol.look_up_value(TaskProtocol.location_dict, lookupstr))

t = Task()
print(t)
t.set_type(1)
t.set_source(1)
t.set_destination(3)
t.set_object(2)
t.set_container(2)
print("Task")
print(t)

twa = TaskWithAction()
twa.copy_from_task(t)
twa.set_action(1)
twa.set_destination(twa.source)
print("Same as source?: ", twa.is_dest_same_as_src())
print("Same task? ", twa.is_equal_to(t))
print("Task with action")
print(twa)
twa.set_destination(t.destination)
print("Same task? ", twa.is_equal_to(t))

print("")

tl = TaskList()
tl.capacity = 3
print("Tl's cap: ", tl.capacity)
print("Tl's empty? ", tl.is_empty())

print("Add tasks")
tl.add_task(t)
t2 = Task()
t2.set_type(2)
t2.set_source(3)
t2.set_destination(3)
t2.set_object(4)
t2.set_container(2)
tl.add_task(t2)

t3 = Task()
t3.set_type(2)
t3.set_source(1)
t3.set_destination(4)
t3.set_object(4)
t3.set_container(2)
tl.add_task(t3)

print("tl's full? ", tl.is_full())
tl.print_task_list()

print("remove")
tl.remove_task(t3)
tl.print_task_list()

print("add task back again")
tl.add_task(t3)
tl.print_task_list()

print("calling sort")
tl.sort_by_src_and_dest()
tl.print_task_list()

print("Calling clear")
tl.clear_task()
tl.print_task_list()
print("-")
