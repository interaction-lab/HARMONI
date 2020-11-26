#!/usr/bin/env python3

# Importing the libraries
import roslaunch
import rospy
import subprocess
import xml.etree.cElementTree as ET
import os
import json
import ast
from harmoni_common_lib.constants import DetectorNameSpace
import harmoni_common_lib.helper_functions as hf


class Launcher:
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
        print(process.is_alive())
        process.stop()
        return

    def _launch_with_subprocess(self, repo, launch):
        """Launch with subprocess """
        p = subprocess.Popen(
            "roslaunch harmoni_decision " + repo + "_" + launch + ".launch", shell=True
        )
        return p

    def _check_if_detector(self, service_name):
        """Check if detector. It returns true if it is a detector """
        list_detectors = [enum.name for enum in list(DetectorNameSpace)]
        for d in list_detectors:
            if service_name == d:
                rospy.loginfo("checking: %s is a detector" % service_name)
                return True
        rospy.loginfo("checking: %s is not a detector" % service_name)
        return False


    def _get_service_pkg(self, repo):
        """Get services """
        [repo_services, services] = hf.get_service_list_of_repo(
            repo
        )  # read from configuration.yaml
        pkg_array = []
        exec_array = []
        for serv in repo_services:
            name = serv.split("_")
            n = ""
            for i in range(0, len(name)):
                if i != 0 and i != (len(name) - 1):
                    n += name[i] + "_"
                elif i != 0 and i == (len(name) - 1):
                    n += name[i]
            if not self._check_if_detector(n):
                pkg_array.append(n)
                exec_array.append(n + "_service")
        return (repo, pkg_array, exec_array)

    def _create_xml_launcher(self, name, repo, package_array, launch_file_array):
        """Create xml launch file """
        root = ET.Element("launch")
        for i in range(0, len(launch_file_array)):
            service_id_list = hf.get_child_list(package_array[i], resources=False)
            for _service_id in service_id_list:
                service_id = hf.get_child_id(_service_id)
                include = ET.SubElement(
                    root,
                    "include",
                    file=f"$(find harmoni_{package_array[i]})/launch/{launch_file_array[i]}.launch",
                )
                args = ET.SubElement(include, "arg", name="test_id", value=service_id)
        tree = ET.ElementTree(root)
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("scripts/")
        file_name = repo + "_" + name + ".launch"
        tree.write(path[0] + "launch/" + file_name)
        rospy.loginfo(f"Created the launch file {file_name}")
        return

    def _create_xml_launcher_pattern(self, name, repo, package_array, launch_file_array):
        """Create xml launch file """
        root = ET.Element("launch")
        for i in range(0, len(launch_file_array)):
            include = ET.SubElement(
                root,
                "include",
                file=f"$(find harmoni_{name})/launch/sequence_pattern.launch",
            )
            args = ET.SubElement(include, "arg", name="pattern_name", value=package_array[i])
        tree = ET.ElementTree(root)
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("scripts/")
        file_name = "harmoni_" + name + ".launch"
        tree.write(path[0] + "launch/" + file_name)
        rospy.loginfo(f"Created the launch file {file_name}")
        return

    def launch_services(self, repo, launch):
        """Launch services """
        name = "service"
        [repo, pkg, exb] = self._get_service_pkg(repo)
        rospy.loginfo(f"Launching services: {pkg} from repo {repo}")
        self._create_xml_launcher(name, repo, pkg, exb)
        if launch:
            self._launch_with_subprocess(repo, name)
        rospy.loginfo(f"Done spawning services: {pkg} from repo {repo}")
        return

    def launch_pattern(self, repo, launch):
        """Launch services """
        name = repo
        [repo, pkg, exb] = self._get_service_pkg(repo)
        rospy.loginfo(f"Launching services: {pkg} from repo {repo}")
        self._create_xml_launcher_pattern(name, repo, pkg, exb)
        if launch:
            self._launch_with_subprocess("harmoni", name)
        rospy.loginfo(f"Done spawning services: {pkg} from repo {repo}")
        return


def main():
    try:
        interface_name = "launcher"
        rospy.init_node(interface_name)
        rospy.loginfo("Set up the %s" % interface_name)
        launch_params = rospy.get_param("~launch")
        service_params = rospy.get_param("~service")
        repos = service_params.split(",")
        rospy.loginfo(f"Repos to include: {repos}")
        launch = Launcher()
        for repo in repos:
            if repo != "" and repo!="pattern":
                launch.launch_services(repo, launch_params)
            elif repo=="pattern":
                launch.launch_pattern(repo, launch_params)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
