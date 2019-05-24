#! /usr/bin/env python
from msg_with_instance import MsgWithInstance
from msg import BaseMsg
from enum_object_identifier import ObjectIdentifierType
from enum_location_identifier import LocationIdentifierType

class TaskProtocol(object):
    ERROR_STRING = "NOT_FOUND"

    task_type_dict = {}
    task_type_dict[1] = "TRANSPORTATION"
    task_type_dict[2] = "NAVIGATION"

    task_action_dict = {}
    task_action_dict[1] = "PICK"
    task_action_dict[2] = "PICK_FROM_ROBOT"
    task_action_dict[3] = "PLACE"
    task_action_dict[4] = "PLACE_FROM_ROBOT"
    task_action_dict[5] = "DRIVE"

    location_dict = MsgWithInstance(max_instance_id=3)
    location_dict = location_dict.build_dict(LocationIdentifierType)

    object_dict = BaseMsg()
    object_dict = object_dict.build_dict(ObjectIdentifierType)

    container_dict = {}
    container_dict[1] = "CONTAINER_B"
    container_dict[2] = "CONTAINER_R"

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