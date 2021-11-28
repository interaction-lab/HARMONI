#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf

# Specific Imports
from harmoni_common_lib.constants import DialogueNameSpace
import boto3
import json

class AWSLexService(HarmoniServiceManager):
    """This is a class representation of a harmoni_dialogue service
    (HarmoniServiceManager). It is essentially an extended combination of the
    :class:`harmoni_common_lib.service_server.HarmoniServiceServer` and :class:`harmoni_common_lib.service_manager.HarmoniServiceManager` classes

    :param name: Name of the current service
    :type name: str
    :param param: input parameters of the configuration.yaml file
    :type param: from yaml
    """

    def __init__(self, name, param):
        """Constructor method: Initialization of variables and lex parameters + setting up"""
        super().__init__(name)
        """ Initialization of variables and lex parameters """
        self.user_id = param["user_id"]
        self.bot_name = param["bot_name"]
        self.bot_alias = param["bot_alias"]
        self.region_name = param["region_name"]
        self.intentName = None
        self.state = State.INIT
        return

    def setup_aws_lex(self):
        """[summary] Setup the lex request, connecting to AWS services"""
        rospy.loginfo("Connecting to Lex")
        self.lex_client = boto3.client("lex-runtime", region_name=self.region_name)
        rospy.loginfo("Connected")
        return

    def request(self, input_text):
        """[summary]

        Args:
            input_text (str): User request (or input text) for triggering Lex Intent

        Returns:
            object: It containes information about the response received (bool) and response message (str)
                response: bool
                message: str
        """
        rospy.loginfo("Start the %s request" % self.name)
        self.state = State.REQUEST
        textdata = input_text
        result = {"response": False, "message": None}
        try:
            lex_response = self.lex_client.post_content(
                botName=self.bot_name,
                botAlias=self.bot_alias,
                userId=self.user_id,
                contentType="text/plain; charset=utf-8",
                accept="text/plain; charset=utf-8",
                inputStream=textdata,
            )
            rospy.loginfo(f"The lex response is {lex_response}")
            #if lex_response["ResponseMetadata"]["HTTPStatusCode"] == 200:
            
            if "intentName" in lex_response:
                if lex_response["dialogState"] == "Fulfilled":
                    rospy.loginfo(
                        "The dialogue is fulfilled, end the conversation."
                    )
            self.response_received = True
            #FIXME
            #vorrei scrivere questo
            lex_response.pop("audioStream")
            if lex_response.get("intentName") is not None:
                self.intentName = lex_response["intentName"]
            else:
                lex_response["intentName"] = self.intentName
            self.result_msg = str(lex_response)
            #ma scrivo questo
            #self.result_msg = str(lex_response["message"]+"-"+lex_response["dialogState"]+"-"+lex_response["intentName"])
            self.state = State.SUCCESS
            #else:
            #    self.start = State.FAILED
            #    rospy.loginfo("Service call failed")
            #    self.response_received = True
            #    self.result_msg = ""
        except rospy.ServiceException:
            self.state = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_received = True
            self.result_msg = ""
        return {"response": self.state, "message": self.result_msg}

def main():
    """[summary]
    Main function for starting HarmoniLex service
    """
    service_name = DialogueNameSpace.bot.name
    instance_id = rospy.get_param("instance_id")  # "default"
    service_id = f"{service_name}_{instance_id}"
    try:
        rospy.init_node(service_name, log_level=rospy.DEBUG)
        params = rospy.get_param(service_name + "/" + instance_id + "_param/")
        s = AWSLexService(service_id, params)
        s.setup_aws_lex()
        service_server = HarmoniServiceServer(service_id, s)

        print(service_name)
        print("**********************************************************************************************")
        print(service_id)

        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()