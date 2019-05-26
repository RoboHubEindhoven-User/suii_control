#! /usr/bin/env python
from enum import Enum

class LocationIdentifierType(Enum):
   SH=1, 'Shelf'
   WS=2, 'Workstation'
   CB=3, 'Conveyor Belt'
   WP=4, 'Way Point'
   PP=5, 'Precision Place'
   ROBOT=6, 'Robot'

   def __new__(cls, value, name):
        member = object.__new__(cls)
        member._value_ = value
        member.fullname = name
        return member