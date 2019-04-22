import yaml

class Time(yaml.YAMLObject):
    yaml_loader = yaml.SafeLoader
    yaml_tag = u'!Time'

    def __init__(self, secs , nsecs):
        self.secs  = secs
        self.nsecs = nsecs