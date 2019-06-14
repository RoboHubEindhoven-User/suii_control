#! /usr/bin/env python

## ===== DictBuilder ===== ##
# Build protocol dictionaries for TaskProtocol
class DictBuilder:
    # Build a dictionary that resembles the enum class
    @staticmethod
    def build_dict(enum_class):
        output_dict = {}
        for i in range(1, len(enum_class) + 1):
            # for some reason, python doesn't traverse enum in order
            # force ordered traversal:
            for item in enum_class:
                if (int(item) == i):
                    if hasattr(item, 'fullname'):
                        output_dict[i] = item.fullname
                    else:
                        output_dict[i] = item.name
            i = i + 1
        return output_dict

    # Build a dictionary of the enum_class with instance_ids
    # The instance ids are from 1 to [max_instance_id]
    # exception_lists = list of ID's that we won't generate instance ids for
    @staticmethod
    def build_dict_with_instance_id(enum_class, max_instance_id=3, exception_lists=[]):
        output_dict = {}
        index = 1

        # Build the exceptions first
        for item in enum_class:
            if (int(item) in exception_lists):
                if hasattr(item, 'fullname'):
                    output_dict[index] = item.fullname
                else:
                    output_dict[index] = item.name
                index = index + 1
        
        # Build instances
        # Forced order traversal: they're being built in the order they appear
        for i in range (1, len(enum_class) + 1):
            for item in enum_class:
                # Skip exceptions
                if int(item) in exception_lists:
                    continue
                # Add instances to dict
                if (int(item) == i):
                    for j in range(1, max_instance_id + 1):
                        if hasattr(item, 'fullname'):
                            output_dict[index] = item.fullname + " " + str(j)
                        else:
                            output_dict[index] = item.name + " " + str(j)
                        index = index + 1
                    break # Finish with this item
            i = i + 1
        return output_dict
