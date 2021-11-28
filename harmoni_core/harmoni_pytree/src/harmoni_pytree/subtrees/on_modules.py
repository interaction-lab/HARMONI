#!/usr/bin/env python3
import py_trees

from harmoni_pytree.leaves.camera_service import CameraServicePytree
from harmoni_pytree.leaves.microphone_service import MicrophoneServicePytree

def create_root():
    #camera = CameraServicePytree(name="Camera")
    microphone=MicrophoneServicePytree("MicrophoneMainActivity")
    #external_camera = CameraServicePytree(name="ExternalCamera")
    root = py_trees.composites.Parallel("Parallel_On_Modules")
    #root.add_children([camera, microphone])
    root.add_child(microphone)

    return root