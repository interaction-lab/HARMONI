#!/usr/bin/env python3

# Common Imports
import rospy
from harmoni_common_lib.constants import State
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
import harmoni_common_lib.helper_functions as hf
from harmoni_bot.aws_lex_service import AWSLexService

# Specific Imports
from harmoni_common_lib.constants import DialogueNameSpace


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
