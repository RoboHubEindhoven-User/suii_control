from enum import Enum

class StateName(Enum):
    TSL = "test_select"
    BNT = "basic_navigation_test"
    BMT = "basic_manipulation_test"
    BTT = "basic_transportation_test"
    PPT = "precision_placement_test"
    RTT = "rotating_table_test"
    EMR = "emergency"