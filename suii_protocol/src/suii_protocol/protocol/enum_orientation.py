#! /usr/bin/env python
from enum import Enum

class OrientationIdentifierType(Enum):
     NORTH = 1, 'North'
     EAST = 2, 'East'
     SOUTH = 3, 'South'
     WEST = 4,'West'
     
     def __new__(cls, value, name):
        member = object.__new__(cls)
        member._value_ = value
        member.fullname = name
        return member

     def __int__(self):
        return self.value