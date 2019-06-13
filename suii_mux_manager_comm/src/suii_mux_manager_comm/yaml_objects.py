import yaml 

### ==== GEOMETRY DATA ==== ###
class Pose2D(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!ServiceArea'
    
    def __init__(self, x, y, angle):
        self.x     = x
        self.y     = y
        self.angle = angle
    
    def __repr__(self):
        return yaml.dump(self)

class Position(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!Position'
    
    def __init__(self, x, y, z):
        self.x     = x
        self.y     = y
        self.z     = z
    def __repr__(self):
        return yaml.dump(self)

class Orientation(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!Orientation'
    
    def __init__(self, x, y, z, w):
        self.x     = x
        self.y     = y
        self.z     = z
        self.w     = w
    def __repr__(self):
        return yaml.dump(self)

### ==== OBJECT DATA ==== ###
class ObjectData(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!ObjectData'

    def __init__(self, obj_type , type_id , instance_id , description, dist_x = 0, dist_y = 0, height = 0, width = 0, angle = 0, rotation_direction = 0):
        self.distance_to_center_x = dist_x
        self.distance_to_center_y = dist_y
        self.height = height
        self.width = width
        self.angle = angle
        self.rotation_direction = rotation_direction
        self.description = description
        self.type = obj_type
        self.type_id = type_id
        self.instance_id = instance_id
    def __repr__(self):
        return yaml.dump(self)

### ==== SERVICE AREA ==== ###
class ServiceArea(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!ServiceArea'
    
    def __init__(self, area_type, area_name, instance_id, description, orientation_id = None, orientation_name = None, position = None, orientation = None):
        self.area_type          = area_type
        self.area_name          = area_name
        self.instance_id        = instance_id
        self.description        = description
        self.orientation_id     = orientation_id
        self.orientation_name   = orientation_name
        self.position           = position
        self.orientation        = orientation
    def __repr__(self):
        return yaml.dump(self)


### ==== TASKS ==== ###
class TaskSpecification(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!TaskSpecification'

    def __init__(self, tool_object, container, source, destination): 
        if (isinstance(tool_object, ObjectData) and isinstance(container, ObjectData) 
        and isinstance(source, ServiceArea) and isinstance(destination, ServiceArea)):
            self.tool_object = tool_object 
            self.container   = container 
            self.source      = source 
            self.destination = destination
    def __repr__(self):
        return yaml.dump(self)


class NavigationTask(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!NavigationTask'

    def __init__(self, destination, wait_time): 
        if isinstance(destination, ServiceArea):
            self.destination = destination
            self.wait_time   = wait_time
    def __repr__(self):
        return yaml.dump(self)


class Time(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!Time'

    def __init__(self, secs , nsecs):
        self.secs  = secs
        self.nsecs = nsecs
    def __repr__(self):
        return yaml.dump(self)