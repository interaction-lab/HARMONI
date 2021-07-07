#!/usr/bin/env python3

# Common Imports
import rospy
import roslib

from harmoni_common_lib.constants import State
from actionlib_msgs.msg import GoalStatus
from harmoni_common_lib.service_server import HarmoniServiceServer
from harmoni_common_lib.service_manager import HarmoniServiceManager
from harmoni_common_lib.action_client import HarmoniActionClient
import harmoni_common_lib.helper_functions as hf
from gesture_service import GestureService
# Specific Imports
from harmoni_common_lib.constants import ActuatorNameSpace, ActionType
from botocore.exceptions import BotoCoreError, ClientError
from contextlib import closing
from collections import deque 
import soundfile as sf
import numpy as np
import boto3
import re
import json
import ast
import sys

#py_tree
import py_trees
import time

import py_trees.console

class GestureServicePytree(py_trees.behaviour.Behaviour):

    """
    mode è il boolean che controlla la modalità di funzionamento:
    true: opzione 1 (utilizzo come una classe python)
    false: opzione 2 (utilizzo mediate action_goal)
    """

    def __init__(self, name = "AWSTtsServicePytree"):
        
        """
        Qui abbiamo pensato di chiamare soltanto 
        il costruttore del behaviour tree 
        """
        self.name = name
        self.mode = False
        self.gesture_service = None
        self.result_data = None
        self.service_client_gesture = None
        self.client_result = None

        self.blackboards = []
        self.blackboard_gesture = self.attach_blackboard_client(name=self.name, namespace="harmoni_gesture")
        self.blackboard_gesture.register_key("result_data", access=py_trees.common.Access.READ)
        self.blackboard_gesture.register_key("result_message", access=py_trees.common.Access.READ)

        super(GestureServicePytree, self).__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))

    def setup(self,**additional_parameters):
        """
        Qui chiamiamo l'inizializzazione del servizio AWSTtsService, 
        motivo per cui abbiamo aggiunto param al metodo che 
        pensiamo debbano essere passati dal chiamante e non possono essere
        creati all'interno del metodo stesso.  
        """
        for parameter in additional_parameters:
            print(parameter, additional_parameters[parameter])  
            if(parameter =="GestureServicePytree_mode"):
                self.mode = additional_parameters[parameter]        

        service_name = ActuatorNameSpace.gesture.name   
        instance_id = rospy.get_param("/instance_id")

        params = rospy.get_param(service_name + "/" + instance_id + "_param/")

        self.gesture_service = GestureService(self.name,params)
        #comment the following line if you are not doing main()
        rospy.init_node(service_name, log_level=rospy.INFO)
        
        if(not self.mode):
            self.service_client_gesture = HarmoniActionClient(self.name)
            self.client_result = deque()
            self.service_client_gesture.setup_client("gesture_default", 
                                                self._result_callback,
                                                self._feedback_callback)
            self.logger.debug("Behavior interface action clients have been set up!")
        
        self.logger.debug("%s.setup()" % (self.__class__.__name__))

    def initialise(self):
        """
        
        """    
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self):
        """
        
        """
        if(self.mode):
            if self.blackboard_gesture.result_message == "SUCCESS":
                self.result_data = self.gesture_service.do(self.blackboard_gesture.result_data)
                new_status = py_trees.common.Status.SUCCESS
            else:
                new_status = self.blackboard_gesture.result_message
        else:
            if self.blackboard_gesture.result_message == "SUCCESS":
                if self.service_client_gesture.get_state() == GoalStatus.LOST:
                    self.logger.debug(f"Sending goal to {self.gesture_service}")
                    # Dove posso prendere details["action_goal"]?
                    self.service_client_gesture.send_goal(
                        action_goal = ActionType["DO"].value,
                        optional_data = self.blackboard_gesture.result_data["message"],
                        wait=False,
                    )
                    self.logger.debug(f"Goal sent to {self.gesture_service}")
                    new_status = py_trees.common.Status.RUNNING
                else:
                    if len(self.client_result) > 0:
                        #se siamo qui vuol dire che il risultato c'è e quindi 
                        #possiamo terminare la foglia
                        self.result_data = self.client_result.popleft()["data"]
                        self.logger.debug(f"Results of gesture module: {self.result_data}")
                        new_status = py_trees.common.Status.SUCCESS
                    else:
                        #incerti di questa riga
                        if(self.gesture_service.state == State.FAILED):
                            new_status = py_trees.common.Status.FAILURE
                        else:
                            new_status = py_trees.common.Status.RUNNING
            else:
                new_status = self.blackboard_gesture.result_message
            self.logger.debug("%s.update()[%s]--->[%s]" % (self.__class__.__name__, self.status, new_status))
        return new_status

        

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        if(new_status == py_trees.common.Status.INVALID):
            #esegui codice per interrupt 
            #self.blackboard_tts.result_message = "INVALID"
            #TODO 
            if(self.mode):
                pass
            else:
                pass
        else:
            #esegui codice per terminare (SUCCESS || FAILURE)
            self.client_result = deque()

        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))

    def _result_callback(self, result):
        """ Recieve and store result with timestamp """
        self.logger.debug("The result of the request has been received")
        self.logger.debug(
            f"The result callback message from {result['service']} was {len(result['message'])} long"
        )
        self.client_result.append(
            {"data": result["message"]}
        )
        # TODO add handling of errors and continue=False
        return

    def _feedback_callback(self, feedback):
        """ Feedback is currently just logged """
        self.logger.debug("The feedback recieved is %s." % feedback)
        # Check if the state is end, stop the behavior pattern
        # if feedback["state"] == State.END:
        #    self.end_pattern = True
        return

def main():
    #command_line_argument_parser().parse_args()

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    blackboardProva = py_trees.blackboard.Client(name="blackboardProva", namespace="harmoni_gesture")
    blackboardProva.register_key("result_data", access=py_trees.common.Access.WRITE)
    blackboardProva.register_key("result_message", access=py_trees.common.Access.WRITE)

    blackboardProva.result_message = "SUCCESS"
    blackboardProva.result_data = "{'gesture':'QT/sad', 'timing': 2}"

    print(blackboardProva)

    gesturePyTree = GestureServicePytree("GestureServiceTest")

    additional_parameters = dict([
        ("GestureServicePytree_mode",False)])

    gesturePyTree.setup(**additional_parameters)
    try:
        for unused_i in range(0, 7):
            gesturePyTree.tick_once()
            time.sleep(0.5)
            print(blackboardProva)
        print("\n")
    except KeyboardInterrupt:
        print("Exception occurred")
        pass
    

if __name__ == "__main__":
    main()
