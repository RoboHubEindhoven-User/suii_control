from enum import Enum

class BenchmarkStateProp(Enum):
    # States of benchmarks
    RUNNING   = 1
    PAUSED    = 2
    FINISHED  = 3
    STOPPED   = 4

class BenchmarkPhaseProp(Enum):
    # Phases of benchmarks
    EXECUTION   = 0
    CALIBRATION = 1
    PREPARATION = 2