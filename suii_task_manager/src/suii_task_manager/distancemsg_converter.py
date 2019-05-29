from suii_task_manager.task_protocol import TaskProtocol
from suii_task_manager.protocol.enum_location_identifier import LocationIdentifierType

class DistanceMsgConverter:
    @staticmethod
    def represents_int(s):
        try: 
            int(s)
            return True
        except ValueError:
            return False
        
    @staticmethod
    def string_to_distance_msg(data):
        fragments = data.split(' ')
        fullname = ''
        type_id = -1
        instance_id = 0

        for item in fragments:
            if (DistanceMsgConverter.represents_int(item)):
                instance_id = int(item)
            else:
                fullname += item + ' '

        fullname = fullname.strip()
        
        for loc in LocationIdentifierType:
            if (loc.fullname == fullname):
                type_id = int(loc)
        
        if (type_id == -1):
            return -1, -1 # Error
        return type_id, instance_id
