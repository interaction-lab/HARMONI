#!/usr/bin/env python3
import py_trees

from harmoni_pytree.leaves.camera_service import CameraServicePytree

def create_root():
    camera = CameraServicePytree(name="Camera")
    #external_camera = CameraServicePytree(name="ExternalCamera")
    #root = py_trees.composites.Parallel("Parallel_On_Modules")
    #root.add_children([camera, external_camera])
    root = camera

    return root