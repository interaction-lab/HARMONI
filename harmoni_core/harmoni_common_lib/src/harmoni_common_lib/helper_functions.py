#!/usr/bin/env python3

# Importing the libraries
import yaml
import rospy
import os
from harmoni_common_lib.constants import (
    DetectorNameSpace,
    SensorNameSpace,
    Resources,
)


PATH_CONFIG = "HARMONI/harmoni_core/harmoni_decision/config/configuration.yaml"


def get_child(child_name):
    """Get children name without ids from config file"""
    abs_path = os.path.abspath(__file__)
    path = abs_path.split("HARMONI/")
    with open(path[0] + PATH_CONFIG) as file:
        repos = yaml.load(file, Loader=yaml.FullLoader)
    for repo in repos:
        if child_name in repos[repo]:
            child_name = child_name
    return child_name


def get_child_list(child_name, resources=True):
    """Get children from config file"""
    ids_list = []
    existed = False
    resources_name = [
        {"name": enum.name, "value": enum.value} for enum in list(Resources)
    ]
    resource_array = ""
    for resource in resources_name:
        if child_name == resource["name"]:
            resource_array = resource["value"]
    abs_path = os.path.abspath(__file__)
    path = abs_path.split("HARMONI/")
    with open(path[0] + PATH_CONFIG) as file:
        repos = yaml.load(file, Loader=yaml.FullLoader)
    for repo in repos:
        if child_name in repos[repo]:
            repo_child_name = repo + "_" + child_name
            existed = True
    if existed:
        child_repo = repo_child_name.split("_")[0]
        for id_child in repos[child_repo][child_name]:
            if resource_array != "" and resources:
                for r in resource_array:
                    ids_list.append(child_name + "_" + r + "_" + id_child)
            else:
                ids_list.append(child_name + "_" + id_child)
    return ids_list


def get_service_list_of_repo(repository):
    """Get children from config file of a specific repo"""
    abs_path = os.path.abspath(__file__)
    path = abs_path.split("HARMONI/")
    service_list = []
    repo_service_list = []
    with open(path[0] + PATH_CONFIG) as file:
        repos = yaml.load(file, Loader=yaml.FullLoader)
    for repo in repos:
        if repo == repository:
            for child in repos[repo]:
                repo_child_name = repo + "_" + child
                child_array = repo_child_name.split("_")
                child_name = ""
                for i in range(0, len(child_array)):
                    if i == 0:
                        child_repo = child_array[0]
                    else:
                        child_name += "_" + child_array[i]
                repo_service_list.append(child_repo + child_name)
                service_list.append(child_name.split("_")[1])
    return (repo_service_list, service_list)


def get_child_id(service_name):
    """Get id of the child from service name"""
    service = service_name.split("_")
    id = service[-1]
    return id


def get_service_name(repo_service_id):
    """Get name of the child from service"""
    service = repo_service_id.split("_")
    name = ""
    for i in range(0, len(service)):
        if not i == 0 and not i == len(service) - 1:
            name += service[i]
    print(name)
    return name


def get_all_repos():
    abs_path = os.path.abspath(__file__)
    path = abs_path.split("HARMONI/")
    repo_list = []
    with open(path[0] + PATH_CONFIG) as file:
        repos = yaml.load(file, Loader=yaml.FullLoader)
    for repo in repos:
        repo_list.append(repo)
    return repo_list


def get_service_server_instance_id(service_name, input_id):
    """Get the service server name """
    name = ""
    if _check_if_resources(service_name):
        service_server = get_child(service_name)  # child
    else:
        list_service_names = get_child_list(service_name)
        for service in list_service_names:
            service_id = get_child_id(service)
            if service_id == input_id:
                service_server = service  # child_id
    return service_server


def check_if_detector(service_name):
    """Check if detector. It returns true if it is a detector """
    list_detectors = [enum.name for enum in list(DetectorNameSpace)]
    for d in list_detectors:
        if service_name == d:
            return True
    return False


def check_if_sensor(service_name):
    """Check if sensor. It returns true if it is a sensor """
    list_sensors = [enum.name for enum in list(SensorNameSpace)]
    for d in list_sensors:
        if service_name == d:
            return True
    return False


def _check_if_resources(service):
    """Check if the service contains many resources """
    has_resources = False
    if service == "face":
        has_resources = True
    return has_resources
