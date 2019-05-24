#! /usr/bin/env python
from msg import BaseMsg

class MsgWithInstance(BaseMsg):
    def __init__(self, max_instance_id=10):
        BaseMsg.__init__(self)
        self.instance_id = -1       # Instance ID
        self.MAX_INSTANCE_ID = max_instance_id

    def __repr__(self):
        return "%s %d" % (self.type.name, self.instance_id)

    def build_dict(self, enum_class):
        i = 1
        output_dict = {}
        for item in enum_class:
            for j in range(1, self.MAX_INSTANCE_ID + 1):
                tmp = MsgWithInstance()
                tmp.type = item 
                tmp.instance_id = j
                output_dict[i] = repr(tmp)
                i = i + 1
        return output_dict