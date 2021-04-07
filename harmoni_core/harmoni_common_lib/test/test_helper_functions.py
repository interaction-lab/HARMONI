#!/usr/bin/env python3

import unittest, harmoni_common_lib.constants

PKG = 'harmoni_common_lib'
# import roslib; roslib.load_manifest(PKG)  # This line is not needed with Catkin.

import harmoni_common_lib.helper_functions as hf

class TestHelperFunctions(unittest.TestCase):

    def test_get_all_repos(self):
        self.assertTrue(len(hf.get_all_repos()) > 0)

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_helper_functions', TestHelperFunctions)
