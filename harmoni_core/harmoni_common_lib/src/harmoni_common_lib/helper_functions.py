#!/usr/bin/env python3

# Importing the libraries
import yaml
import os
from harmoni_common_lib.constants import Router


class HelperFunctions:
    def get_routers():
        router_names = [enum.value for enum in list(Router)]
        return router_names

    def get_child_list(child_name):
        """Get children from config file"""
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("HARMONI/")
        with open(
            path[0] + "HARMONI/harmoni_decision/config/configuration.yaml"
        ) as file:
            repos = yaml.load(file, Loader=yaml.FullLoader)
        for repo in repos:
            if child_name in repos[repo]:
                repo_child_name = repo + "_" + child_name
        child_repo = repo_child_name.split("_")[0]
        ids_list = []
        for id_child in repos[child_repo][child_name]:
            ids_list.append(child_repo + "_" + child_name + "_" + id_child)
        return ids_list

    def get_service_list_of_repo(repository):
        """Get children from config file of a specific repo"""
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("HARMONI/")
        service_list = []
        with open(
            path[0] + "HARMONI/harmoni_decision/config/configuration.yaml"
        ) as file:
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
                    service_list.append(child_repo + child_name)
        return service_list

    def get_child_id(service_name):
        """Get id of the child from service name"""
        service = service_name.split("_")
        id = service[-1]
        return id

    def get_all_repos():
        abs_path = os.path.abspath(__file__)
        path = abs_path.split("HARMONI/")
        repo_list = []
        with open(
            path[0] + "HARMONI/harmoni_decision/config/configuration.yaml"
        ) as file:
            repos = yaml.load(file, Loader=yaml.FullLoader)
        for repo in repos:
            repo_list.append(repo)
        return repo_list
