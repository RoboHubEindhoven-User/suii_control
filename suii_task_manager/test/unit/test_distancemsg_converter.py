# !/usr/bin/env python

# To import suii_task_manager
import sys, os
sys.path.append(os.path.abspath(os.path.join('../..', 'src/')))

import unittest
from suii_task_manager.distancemsg_converter import DistanceMsgConverter
from suii_task_manager.protocol.enum_location_identifier import LocationIdentifierType

class DistanceMsgConverterTest(unittest.TestCase):
    def test_string_to_distance_msg(self):
        print ("Testing method: " + str(self._testMethodName))

        test_string = "Shelf 5" 
        print("Test string: '%s'" % test_string)
        type_id, instance_id = DistanceMsgConverter.string_to_distance_msg(test_string)
        
        self.assertEquals(type_id, int(LocationIdentifierType.SH))
        self.assertEquals(instance_id, 5)

        test_string = "Workstation 5" 
        print("Test string: '%s'" % test_string)
        type_id, instance_id = DistanceMsgConverter.string_to_distance_msg(test_string)
        
        self.assertEquals(type_id, int(LocationIdentifierType.WS))
        self.assertEquals(instance_id, 5)

        test_string = "Conveyor Belt 5" 
        print("Test string: '%s'" % test_string)
        type_id, instance_id = DistanceMsgConverter.string_to_distance_msg(test_string)
        
        self.assertEquals(type_id, int(LocationIdentifierType.CB))
        self.assertEquals(instance_id, 5)

        test_string = "Way Point 5" 
        print("Test string: '%s'" % test_string)
        type_id, instance_id = DistanceMsgConverter.string_to_distance_msg(test_string)
        
        self.assertEquals(type_id, int(LocationIdentifierType.WP))
        self.assertEquals(instance_id, 5)

        test_string = "Precision Place 5"
        print("Test string: '%s'" % test_string)
        type_id, instance_id = DistanceMsgConverter.string_to_distance_msg(test_string)

        self.assertEquals(type_id, int(LocationIdentifierType.PP))
        self.assertEquals(instance_id, 5)

        test_string = "Robot 5"
        print("Test string: '%s'" % test_string)
        type_id, instance_id = DistanceMsgConverter.string_to_distance_msg(test_string)

        self.assertEquals(type_id, int(LocationIdentifierType.ROBOT))
        self.assertEquals(instance_id, 5)   

        test_string = "Random 5"
        print("Test string: '%s'" % test_string)
        type_id, instance_id = DistanceMsgConverter.string_to_distance_msg(test_string)

        self.assertEquals(type_id, -1)
        self.assertEquals(instance_id, -1)       
        

if __name__ == '__main__':
    unittest.main()