from enum import Enum

# direction to pick and place object
class PickPlaceDirection(Enum):
    SERVICE_AREA_ROBOT = 1
    ROBOT_SERVICE_AREA = 2

# Tray holder id on robot to hold objects
class HolderTray(Enum):
    HOLDER_1 = 1
    HOLDER_2 = 2
    HOLDER_3 = 3