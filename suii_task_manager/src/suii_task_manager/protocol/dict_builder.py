#! /usr/bin/env python
 
class BaseMsg(object):
    def __init__(self, max_instance_id=10):
        self.type = None            # An enum

    def __repr__(self):
        return self.type.name

    def build_dict(self, enum_class):
        i = 1
        output_dict = {}
        for item in enum_class:
            tmp = BaseMsg()
            tmp.type = item 
            output_dict[i] = repr(tmp)
            i = i + 1
        return output_dict

class DictBuilder:
    @staticmethod
    def build_dict(enum_class):
        i = 1
        output_dict = {}
        for item in enum_class:
            if hasattr(item, 'fullname'):
                output_dict[i] = item.fullname
            else:
                output_dict[i] = item.name
            i = i + 1
        return output_dict

    @staticmethod
    def build_dict_with_instance_id(enum_class, max_instance_id=3):
        i = 1
        output_dict = {}
        for item in enum_class:
            for j in range(1, max_instance_id + 1):
                if hasattr(item, 'fullname'):
                    output_dict[i] = item.fullname + " " + str(j)
                else:
                    output_dict[i] = item.name + " " + str(j)
                i = i + 1
        return output_dict
