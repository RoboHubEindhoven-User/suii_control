#! /usr/bin/env python
from enum import Enum

class StateName(Enum):
    IDLE_STATE              = "idle_state"
    CONVERT_DATA_STATE      = "convert_data_state"
    TASK_SELECT             = "task_select_state"
    TASK_EXECUTION          = "task_execution_state"
    EMERGENCY               = "emergency_state"
    ERROR                   = "error_state"

class TransitionName(Enum):
    IDLE_STATE              = "to_idle_state"
    CONVERT_DATA_STATE      = "to_convert_data_state"
    TASK_SELECT             = "to_task_select_state"
    TASK_EXECUTION          = "to_task_execution_state"
    EMERGENCY               = "to_emergency_state"
    ERROR                   = "to_error_state"