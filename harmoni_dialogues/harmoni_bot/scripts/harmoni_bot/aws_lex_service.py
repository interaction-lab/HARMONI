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
        """Constructor method: Initialization of variables and lex parameters + setting up
        """
        super().__init__(name)
        """ Initialization of variables and lex parameters """
        self.user_id = param["user_id"]
        self.bot_name = param["bot_name"]
        self.bot_alias = param["bot_alias"]
        self.region_name = param["region_name"]
        self.setup_aws_lex()
        self.state = State.INIT
        return

    def setup_aws_lex(self):
        """[summary] Setup the lex request, connecting to AWS services
        """
        self.lex_client = boto3.client("lex-runtime", region_name=self.region_name)
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
        result = {"response": False, "message":None}
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
            if lex_response["ResponseMetadata"]["HTTPStatusCode"]==200:
                self.state = State.SUCCESS
                if "intentName" in lex_response:
                    if lex_response["dialogState"] == "Fulfilled":
                        rospy.loginfo("The dialogue is fulfilled, end the conversation.")
                rospy.loginfo("The response is %s" % (lex_response["message"]))
                self.response_received = True
                self.result_msg = lex_response["message"]
            else: 
                self.start = State.FAILED
                rospy.loginfo("Service call failed")
                self.response_received = True
                self.result_msg = ""
        except rospy.ServiceException:
            self.start = State.FAILED
            rospy.loginfo("Service call failed")
            self.response_received = True
            self.result_msg = ""
        return {"response": self.response_received, "message":self.result_msg}


def main():
    """[summary]
    Main function for starting HarmoniLex service
    """
    service_name = DialogueNameSpace.bot.name
    name = rospy.get_param("/name_" + service_name + "/")
    test_id = rospy.get_param("/test_id_" + service_name + "/")
    try:
        rospy.init_node(service_name)
        param = rospy.get_param(name + "/" + test_id + "_param/")
        service = hf.set_service_server(service_name, test_id)
        s = AWSLexService(service, param)
        service_server = HarmoniServiceServer(name=service, service_manager=s)
        service_server.start_sending_feedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
