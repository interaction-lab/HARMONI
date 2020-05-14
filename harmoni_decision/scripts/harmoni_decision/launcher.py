#!/usr/bin/env python3

# Importing the libraries
import roslaunch
import rospy
import subprocess
import xml.etree.cElementTree as ET
import os
import json
import ast
from harmoni_common_lib.constants import HelperFunctions


class Launcher():
    """
    Launcher for launching nodes dynamically
    """
   
    def __init__(self):
        """Init launchers"""
        rospy.loginfo("Init Launcher")
        
        
    def launch_node_ros_api(self, package, executable):
        """ Launcher of nodes """
        node = roslaunch.core.Node(package, executable)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node)
        print (process.is_alive())
        process.stop()
        return

    def launch_router_ros_api(self):
        """Launch routers """
        repo = "harmoni"
        routers = HelperFunctions.get_routers()
        processes = []
        for rout in routers:
            package = rout
            executable = rout+"_router"
            p = self._launch_with_subprocess(repo, package)
            processes.append(p)
        for cp in processes:
            cp.wait()
        return

    def _launch_with_subprocess(self,repo, launch):
        """Launch with subprocess """
        p = subprocess.Popen("roslaunch harmoni_decision "+repo+"_"+launch+"_launch.launch", shell=True)
        return p

    def _get_router_pkg(self):
        """Get routers """
        repo = "harmoni"
        routers = HelperFunctions.get_routers()
        pkg_array = []
        exec_array= []
        for rout in routers:
            pkg_array.append(rout)
            exec_array.append(rout+"_router")
        return(repo, pkg_array, exec_array)

    def _get_service_pkg(self, repo):
        """Get routers """
        services = HelperFunctions.get_service_list_of_repo(repo)
        pkg_array = []
        exec_array= []
        for serv in services:
            name = serv.split("_")
            pkg_array.append(name[1])
            exec_array.append(name[1] +"_service")
        return(repo, pkg_array, exec_array)

    def _create_xml_launcher(self,name, repo, package_array, launch_file_array):
        """Create xml launch file """
        root = ET.Element("launch")
        for i in range(0, len(launch_file_array)):
            ET.SubElement(root, "include", file="$(find "+repo+"_"+package_array[i]+")/launch/"+launch_file_array[i]+".launch")
        tree = ET.ElementTree(root)
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("scripts/")
        tree.write(path[0] + "launch/"+ repo +"_"+ name+"_launch.launch")
        rospy.loginfo("Creating the launch file")
        return

    def launch_router(self):
        """Launch routers """
        name = "router"
        [repo, pkg, exb] = self._get_router_pkg()
        self._create_xml_launcher(name,repo, pkg, exb)
        self._launch_with_subprocess(repo, name)
        return


    def launch_services(self, repo):
        """Launch services """
        name = "service"
        [repo, pkg, exb] = self._get_service_pkg(repo)
        self._create_xml_launcher(name,repo, pkg, exb)
        self._launch_with_subprocess(repo, name)
        return


def main():
    try: 
        interface_name = "launcher"
        rospy.init_node(interface_name + "_node")
        rospy.loginfo("Set up the %s" %interface_name)
        l = Launcher()
        l.launch_router()
        """
        repos_list = HelperFunctions.get_all_repos()
        for repo in repos_list:
            l.launch_services(repo)
        """
        l.launch_services("harmoni")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
