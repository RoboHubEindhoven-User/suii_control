import yaml

class Pose2D(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!ServiceArea'
    
    def __init__(self, x, y, angle):
        self.x     = x
        self.y     = y
        self.angle = angle

class Position(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!Position'
    
    def __init__(self, x, y, z):
        self.x     = x
        self.y     = y
        self.z     = z

class Orientation(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!Orientation'
    
    def __init__(self, x, y, z, w):
        self.x     = x
        self.y     = y
        self.z     = z
        self.w     = w

class Pose(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!Pose'
    
    def __init__(self, position = None, orientation = None):
        self.position    = position
        self.orientation = orientation

class Coordinate(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!Coordinate'
    
    def __init__(self, direction = None, orientation = None):
        self.direction   = direction
        self.orientation = orientation