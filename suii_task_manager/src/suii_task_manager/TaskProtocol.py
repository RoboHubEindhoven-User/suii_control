class TaskProtocol(object):
    ERROR_STRING = "NOT_FOUND"

    taskTypeDict = {}
    taskTypeDict[1] = "TRANSPORTATION"
    taskTypeDict[2] = "NAVIGATION"

    taskActionDict = {}
    taskActionDict[1] = "PICK"
    taskActionDict[2] = "PICK_FROM_ROBOT"
    taskActionDict[3] = "PLACE"
    taskActionDict[4] = "PLACE_FROM_ROBOT"
    taskActionDict[5] = "DRIVE"

    locationDict = {}
    locationDict[1] = "Workstation 1"
    locationDict[2] = "Workstation 2"
    locationDict[3] = "Workstation 3"
    locationDict[4] = "Workstation 4"
    locationDict[5] = "Workstation 5"
    locationDict[6] = "Workstation 6"

    objectDict = {}
    objectDict[1] = "Large Black Alu. Profile"
    objectDict[2] = "Large Nut"
    objectDict[3] = "Small Nut"
    objectDict[4] = "Motor"
    objectDict[5] = "Plastic Tube"

    containerDict = {}
    containerDict[1] = "CONTAINER_B"
    containerDict[2] = "CONTAINER_R"

    # Look up key in dictionary
    @staticmethod
    def lookUpKey(dictionary, id):
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

