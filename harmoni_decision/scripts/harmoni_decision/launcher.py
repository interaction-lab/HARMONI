#!/usr/bin/env python3

# Importing the libraries
import roslaunch
import rospy
import subprocess
import xml.etree.cElementTree as ET
import os
import json
import ast
from harmoni_common_lib.constants import HelperFunctions, RouterDetector


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
        p = subprocess.Popen("roslaunch harmoni_decision "+repo+"_"+launch+".launch", shell=True)
        return p

    def _check_if_detector(self, service_name):
        """Check if detector. It returns true if it is a detector """
        list_detectors = [enum.name for enum in list(RouterDetector)]
        for d in list_detectors:
            if service_name == d:
                return True
        rospy.loginfo("%s not a detector" %service_name)
        return False 


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
        """Get services """
        services = HelperFunctions.get_service_list_of_repo(repo)
        pkg_array = []
        exec_array= []
        for serv in services:
            name = serv.split("_")
            n= ""
            for i in range(0,len(name)):
                if i!=0 and i!=(len(name)-1):
                    n+= name[i]+"_"
                elif i!=0 and i==(len(name)-1):
                    n+= name[i]
            if not self._check_if_detector(n):
                pkg_array.append(n)
                exec_array.append(n +"_service")
        return(repo, pkg_array, exec_array)

    def _create_xml_launcher(self,name, repo, package_array, launch_file_array):
        """Create xml launch file """
        root = ET.Element("launch")
        for i in range(0, len(launch_file_array)):
            ET.SubElement(root, "include", file="$(find "+repo+"_"+package_array[i]+")/launch/"+launch_file_array[i]+".launch")
        tree = ET.ElementTree(root)
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("scripts/")
        tree.write(path[0] + "launch/"+ repo +"_"+ name+".launch")
        rospy.loginfo("Creating the launch file")
        return

    def launch_router(self, launch):
        """Launch routers """
        name = "router"
        [repo, pkg, exb] = self._get_router_pkg()
        print(repo)
        self._create_xml_launcher(name,repo, pkg, exb)
        if launch:
            self._launch_with_subprocess(repo, name)
        return


    def launch_services(self, repo, launch):
        """Launch services """
        name = "service"
        [repo, pkg, exb] = self._get_service_pkg(repo)
        print(pkg)
        self._create_xml_launcher(name,repo, pkg, exb)
        if launch:
            self._launch_with_subprocess(repo, name)
        return


def main():
    try: 
        interface_name = "launcher"
        rospy.init_node(interface_name)
        rospy.loginfo("Set up the %s" %interface_name)
        launch = rospy.get_param("~launch")
        router = rospy.get_param("~router")
        service = rospy.get_param("~service")
        services = service.split(",")
        print(services)
        l = Launcher()
        if router:
            l.launch_router(launch)
        for serv in services:
            if serv != "":
                l.launch_services(serv, launch)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
