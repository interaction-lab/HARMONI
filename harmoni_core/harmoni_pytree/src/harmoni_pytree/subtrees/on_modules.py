#!/usr/bin/env python3
import py_trees
from leaves.camera_service_pytree import CameraServicePytree

def create_root():
    camera = py_trees.behaviours.Count(name="Camera",
                                            fail_until=0,
                                            running_until=1,
                                            success_until=10,
                                            reset=False)
    external_camera = py_trees.behaviours.Count(name="ExternalCamera",
                                                        fail_until=0,
                                                        running_until=1,
                                                        success_until=10,
                                                        reset=False)
    root = py_trees.composites.Parallel("Parallel_On_Modules")
    root.add_children([camera, external_camera])

    return root