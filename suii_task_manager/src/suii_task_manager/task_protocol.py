#! /usr/bin/env python
from protocol.msg import BaseMsg
from protocol.msg_with_instance import MsgWithInstance
from protocol.enum_location_identifier import LocationIdentifierType
from protocol.enum_object import ObjectType
from protocol.enum_task_action import TaskActionType
from protocol.enum_task_type import TaskType

class TaskProtocol(object):
    ERROR_STRING = "NOT_FOUND"

    task_type_dict = BaseMsg()
    task_type_dict = task_type_dict.build_dict(TaskType)

    task_action_dict = BaseMsg()
    task_action_dict = task_action_dict.build_dict(TaskActionType)

    location_dict = MsgWithInstance(max_instance_id=3)
    location_dict = location_dict.build_dict(LocationIdentifierType)

    object_dict = BaseMsg()
    object_dict = object_dict.build_dict(ObjectType)

    container_dict = BaseMsg()
    container_dict = container_dict.build_dict(ObjectType)

    # Look up key in dictionary
    @staticmethod
    def look_up_key (dictionary, id):
        if (id in dictionary):
            return dictionary[id]
        return TaskProtocol.ERROR_STRING
        
    # Look up value in dictionary
    @staticmethod
    def look_up_value (dictionary, strval):
        for key, value in dictionary.items():
            if (value == strval):
                return key
        return -1
    
    # Add unique (key,value) to dictionary
    @staticmethod
    def add_key_value (dictionary, id, strval):
        if id in dictionary or TaskProtocol.look_up_value (dictionary, strval) != -1:
            return -1
        dictionary[id] = strval

    # Print out the given dictionary
    @staticmethod
    def print_protocol (dictionary):
        for key, value in dictionary.items():
            print("Key: '%s'; Value: '%s'" % (key, value))