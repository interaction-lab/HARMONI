#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager

# Specific Imports
import argparse
import os
import rasa
import threading
from harmoni_common_lib.constants import DialogueNameSpace
from harmoni_bot.rasa_client import RasaClient


class RasaService(HarmoniServiceManager):
    """This is a class representation of a harmoni_dialogue service
        (HarmoniServiceManager). It is essentially an extended combination of the
        :class:`harmoni_common_lib.service_server.HarmoniServiceServer` and
        :class:`harmoni_common_lib.service_manager.HarmoniServiceManager` classes

        :param name: Name of the current service
        :type name: str
        :param param: input parameters of the configuration.yaml file
        :type param: from yaml
        """

    def __init__(self, name, param):
        """Constructor method: Initialization of variables and lex parameters + setting up"""
        super().__init__(name)
        """ Initialization of variables and parameters """
        self.host = param["host"]
        self.port = param["port"]

        self.rasa_client = RasaClient(
            self.host,
            self.port
        )

        self.state = State.INIT
        return

    def request(self, input_text):
        """[summary]

        Args:
            input_text (str): User request (or input text) for triggering Rasa Intent

        Returns:
            object: It contains information about the response received (bool) and response message (str)
                response: bool
                message: str
        """
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST

        try:
            rasa_response = self.rasa_client.get_rasa_response(input_text)
            rospy.loginfo(f"The Rasa response is {rasa_response}")
            self.state = State.SUCCESS
        except Exception as e:
            rospy.loginfo(f"Exception occurred: {e}")
            self.state = State.FAILED
            rasa_response = ""
        self.response_received = True
        self.result_msg = rasa_response
        return {"response": self.state, "message": rasa_response}


def main():
    """[summary]
    Main function for starting HarmoniRasa service
    """
    service_name = DialogueNameSpace.bot.name
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name, log_level=rospy.INFO)
        params = rospy.get_param("rasa" + "/" + instance_id + "_param/")
        s = RasaService(service_id, params)
        service_server = HarmoniServiceServer(service_id, s)
        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
