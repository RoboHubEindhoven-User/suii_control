#! /usr/bin/env python
from protocol.dict_builder import DictBuilder
from protocol.enum_location_identifier import LocationIdentifierType
from protocol.enum_object_identifier import ObjectIdentifierType
from protocol.enum_task_action import TaskActionType
from protocol.enum_task_type import TaskType

## ===== TaskProtocol ===== ##
# The protocol to look up IDs and strings
class TaskProtocol(object):
    ERROR_STRING = "NOT_FOUND"

    # Build dictionaries
    task_type_dict = DictBuilder.build_dict(TaskType)
    task_action_dict = DictBuilder.build_dict(TaskActionType)
    object_dict = DictBuilder.build_dict(ObjectIdentifierType)
    container_dict = DictBuilder.build_dict(ObjectIdentifierType)

    # Make, for each location, 500 instance IDs, except the ones with ID's given in the list
    location_dict = DictBuilder.build_dict_with_instance_id(LocationIdentifierType, max_instance_id=500, 
        exception_lists=[int(LocationIdentifierType.EX), int(LocationIdentifierType.ROBOT), int(LocationIdentifierType.CB), int(LocationIdentifierType.PP)])

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

# print('WP1: ', TaskProtocol.look_up_value(TaskProtocol.location_dict, 'Way Point 1'))
# print('WP2: ', TaskProtocol.look_up_value(TaskProtocol.location_dict, 'Way Point 2'))