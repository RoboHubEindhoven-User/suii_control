import yaml

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
        