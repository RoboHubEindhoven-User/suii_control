from yaml_objects import *
import yaml 
import string

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

class YAMLHandler():
    def __init__(self, path=""):
        self.path = path
        self.data = None 
    
    def load(self):
        if (self.path != ""):
            self.data = YAMLReader.load(self.path)
        return self.data != None
    
    def get_pose_for(self, destination_str):
        for task in self.data:
            if (type(task) == NavigationTask):
                lower_string = string.lower(task.destination.description)
                if (lower_string == string.lower(destination_str)):
                    # w, x, y, z
                    orientation = [task.destination.orientation.w, task.destination.orientation.x, 
                                task.destination.orientation.y, task.destination.orientation.z]
                    # x, y, z
                    position = [task.destination.position.x, task.destination.position.y, task.destination.position.z]
                    return orientation, position
        return None, None

