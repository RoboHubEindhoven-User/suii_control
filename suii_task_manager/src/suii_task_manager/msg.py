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