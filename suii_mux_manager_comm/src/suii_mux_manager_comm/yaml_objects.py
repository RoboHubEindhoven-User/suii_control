from yaml import YAMLObject, SafeLoader
import yaml

### ==== GEOMETRY DATA ==== ###
class Pose(YAMLObject):
    yaml_loader = SafeLoader
    yaml_tag = u'!Pose'

    def __init__(self, position = None, orientation = None):
        self.position    = position
        self.orientation = orientation
    def __repr__(self):
        return yaml.dump(self)
        
class Point(YAMLObject):
    yaml_loader = SafeLoader
    yaml_tag = u'!Point'

    def __init__(self, x = None, y = None, z = None):
        self.x = x
        self.y = y
        self.z = z
    def __repr__(self):
        return yaml.dump(self)
    
class Quaternion(YAMLObject):
    yaml_loader = SafeLoader
    yaml_tag = u'!Quaternion'

    def __init__(self, x = None, y = None, z = None, w = None):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    def __repr__(self):
        return yaml.dump(self)

### ==== ROBOT ARM ==== ###
class JointPose(YAMLObject):
    yaml_loader = SafeLoader
    yaml_tag = u'!JointPose'
    
    def __init__(self, id = 0, description = "", joint_1 = 0, joint_2 = 0, joint_3 = 0, joint_4 = 0, joint_5 = 0, joint_6 = 0):
        self.id          = id
        self.description = description
        self.joint_1     = joint_1
        self.joint_2     = joint_2
        self.joint_3     = joint_3
        self.joint_4     = joint_4
        self.joint_5     = joint_5
        self.joint_6     = joint_6
    def __repr__(self):
        return yaml.dump(self)

### ==== OBJECT DATA ==== ###
class ObjectData(YAMLObject):
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
class ServiceArea(YAMLObject):
    yaml_loader = SafeLoader
    yaml_tag = u'!ServiceArea'

    def __init__(self, id = None, length = None, width = None, height = None, diameter = None, tilt_angle = None):
        self.id         = id
        self.length     = length
        self.width      = width
        self.height     = height
        self.diameter   = diameter
        self.tilt_angle = tilt_angle
    def __repr__(self):
        return yaml.dump(self)

class Location:
    def __init__(self, loc_type = None, instance_id = None, description = None):
        self.type        = loc_type
        self.instance_id = instance_id
        self.description = description
    def __repr__(self):
        return yaml.dump(self)

class Waypoint(YAMLObject):
    yaml_loader = SafeLoader
    yaml_tag = u'!Waypoint'

    def __init__(self, location = None, pose = None, service_area = None, should_scan = None):
        self.location     = location
        self.pose         = pose
        self.service_area = service_area
        self.should_scan  = should_scan
    def __repr__(self):
        return yaml.dump(self)


### ==== TASKS ==== ###
# class TaskSpecification(YAMLObject):
#     yaml_loader = yaml.SafeLoader
#     yaml_tag = u'!TaskSpecification'

#     def __init__(self, tool_object, container, source, destination): 
#         if (isinstance(tool_object, ObjectData) and isinstance(container, ObjectData) 
#         and isinstance(source, ServiceArea) and isinstance(destination, ServiceArea)):
#             self.tool_object = tool_object 
#             self.container   = container 
#             self.source      = source 
#             self.destination = destination
#     def __repr__(self):
#         return yaml.dump(self)


# class NavigationTask(YAMLObject):
#     yaml_loader = yaml.SafeLoader
#     yaml_tag = u'!NavigationTask'

#     def __init__(self, destination, wait_time): 
#         if isinstance(destination, ServiceArea):
#             self.destination = destination
#             self.wait_time   = wait_time
#     def __repr__(self):
#         return yaml.dump(self)


# class Time(YAMLObject):
#     yaml_loader = yaml.SafeLoader
#     yaml_tag = u'!Time'

#     def __init__(self, secs , nsecs):
#         self.secs  = secs
#         self.nsecs = nsecs
#     def __repr__(self):
#         return yaml.dump(self)