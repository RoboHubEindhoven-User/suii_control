import yaml
from suii_control.utils.object_data import ObjectData
from suii_control.utils.service_area import ServiceArea
from suii_control.utils.time_data import Time

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


class NavigationTask(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!NavigationTask'

    def __init__(self, destination, wait_time): 
        if isinstance(destination, ServiceArea):
            self.destination = destination
            self.wait_time   = wait_time