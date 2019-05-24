#! /usr/bin/env python
from enum import Enum

class TaskActionType(Enum):
   PICK = 1,
   PICK_FROM_ROBOT = 2,
   PLACE = 3,
   PLACE_FROM_ROBOT = 4,
   DRIVE = 5