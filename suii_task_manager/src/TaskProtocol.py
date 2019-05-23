class TaskProtocol(object):
    initialized = False
    ERROR_STRING = "NOT_FOUND"
    taskTypeDict = {}
    taskActionDict = {}
    locationDict = {}
    objectDict = {}
    containerDict = {}

    # Check for init
    @staticmethod
    def checkInit():
        if (TaskProtocol.taskTypeDict == {} and 
            TaskProtocol.taskActionDict == {} and 
            TaskProtocol.locationDict == {} and 
            TaskProtocol.objectDict == {} and 
            TaskProtocol.containerDict == {}):
            TaskProtocol.initialize()

    # Look up key in dictionary
    @staticmethod
    def initialize():
        # Populate dictionary
        TaskProtocol.addKeyValue(TaskProtocol.taskTypeDict, 1, "TRANSPORTATION")
        TaskProtocol.addKeyValue(TaskProtocol.taskTypeDict, 2, "NAVIGATION")
        # Task action
        TaskProtocol.addKeyValue(TaskProtocol.taskActionDict, 1, "PICK")
        TaskProtocol.addKeyValue(TaskProtocol.taskActionDict, 2, "PICK_FROM_ROBOT")
        TaskProtocol.addKeyValue(TaskProtocol.taskActionDict, 3, "PLACE")
        TaskProtocol.addKeyValue(TaskProtocol.taskActionDict, 4, "PLACE_FROM_ROBOT")
        TaskProtocol.addKeyValue(TaskProtocol.taskActionDict, 5, "DRIVE")

        # Location
        TaskProtocol.addKeyValue(TaskProtocol.locationDict, 1, "Workstation 1")
        TaskProtocol.addKeyValue(TaskProtocol.locationDict, 2, "Workstation 2")
        TaskProtocol.addKeyValue(TaskProtocol.locationDict, 3, "Workstation 3")
        TaskProtocol.addKeyValue(TaskProtocol.locationDict, 4, "Workstation 4")
        TaskProtocol.addKeyValue(TaskProtocol.locationDict, 5, "Workstation 5")
        TaskProtocol.addKeyValue(TaskProtocol.locationDict, 6, "Workstation 6")

        # Objects
        TaskProtocol.addKeyValue(TaskProtocol.objectDict, 1, "Large Black Alu. Profile")
        TaskProtocol.addKeyValue(TaskProtocol.objectDict, 2, "Large Nut")
        TaskProtocol.addKeyValue(TaskProtocol.objectDict, 3, "Small Nut")
        TaskProtocol.addKeyValue(TaskProtocol.objectDict, 4, "Motor")
        TaskProtocol.addKeyValue(TaskProtocol.objectDict, 5, "Plastic Tube")

        # Containers
        TaskProtocol.addKeyValue(TaskProtocol.containerDict, 1, "CONTAINER_B")
        TaskProtocol.addKeyValue(TaskProtocol.containerDict, 2, "CONTAINER_R")

    # Look up key in dictionary
    @staticmethod
    def lookUpKey(dictionary, id):
        TaskProtocol.checkInit()
        if (id in dictionary):
            return dictionary[id]
        return TaskProtocol.ERROR_STRING
        
    # Look up value in dictionary
    @staticmethod
    def lookUpValue(dictionary, strval):
        for key, value in dictionary.items():
            if (value == strval):
                return key
        return -1
    
    # Add unique (key,value) to dictionary
    @staticmethod
    def addKeyValue(dictionary, id, strval):
        if id in dictionary or TaskProtocol.lookUpValue(dictionary, strval) != -1:
            return -1
        dictionary[id] = strval

    # Print out the given dictionary
    @staticmethod
    def printProtocol(dictionary):
        for key, value in dictionary.items():
            print("Key: '%s'; Value: '%s'" % (key, value))