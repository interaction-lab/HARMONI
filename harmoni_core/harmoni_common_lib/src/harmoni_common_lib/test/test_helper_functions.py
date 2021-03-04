#!/bin/usr/env python3

import unittest

PKG = 'harmoni_common_lib'
# import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import harmoni_common_lib.helper_functions as hf

class TestHelperFunctions(unittest.TestCase):

    def test_get_routers(self):
        routers = hf.get_routers()
        self.assertTrue(len(routers) > 0)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_helper_functions', TestHelperFunctions)
