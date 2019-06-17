from yaml_objects import *
import yaml 
import string

## ===== YAMLReader ===== ##
# Input: Path
# Output: A parsed YAML Object
class YAMLReader():
    @staticmethod
    def load(path):
        yaml_file = None
        with open(path, 'r') as stream:
            try:
                yaml_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return None 
        return yaml_file 

## ===== YAMLHandler ===== ##
# Get specific data from YAML
class YAMLHandler():
    def __init__(self, path=""):
        self.path = path
        self.data = None 
    
    def load(self):
        if (self.path != ""):
            self.data = YAMLReader.load(self.path)
        return self.data != None

    def get_table_height_for (self, destination_str):
        # Data is a dictionary of waypoints
        # With baby dictionaries inside
        for waypoint_key, waypoint_value in self.data.items():
            if (waypoint_value['location']['description'] == destination_str):
                return waypoint_value['service_area']['height']
        return -1 

    def get_pose_for(self, destination_str):
        # Data is a dictionary of waypoints
        # With baby dictionaries inside
        for waypoint_key, waypoint_value in self.data.items():
            if (waypoint_value['location']['description'] == destination_str):
                orientation = [waypoint_value['pose']['orientation']['w'], 
                            waypoint_value['pose']['orientation']['x'],
                            waypoint_value['pose']['orientation']['y'],
                            waypoint_value['pose']['orientation']['z']]

                position = [waypoint_value['pose']['position']['x'], 
                            waypoint_value['pose']['position']['y'],
                            waypoint_value['pose']['position']['z']]
                return orientation, position
        return None, None
        # for task in self.data:
        #     if (type(task) == NavigationTask):
        #         lower_string = string.lower(task.destination.description)
        #         if (lower_string == string.lower(destination_str)):
        #             # w, x, y, z
        #             orientation = [task.destination.orientation.w, task.destination.orientation.x, 
        #                         task.destination.orientation.y, task.destination.orientation.z]
        #             # x, y, z
        #             position = [task.destination.position.x, task.destination.position.y, task.destination.position.z]
        #             return orientation, position
        # return None, None