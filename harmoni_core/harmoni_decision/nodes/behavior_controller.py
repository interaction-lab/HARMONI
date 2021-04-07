#!/usr/bin/env python3

# Importing the libraries
import rospy
import roslib
from harmoni_common_lib.action_client import HarmoniActionClient
from harmoni_common_lib import constants

import inspect


class BehaviorController:
    """Instantiates behaviors and receives commands/data for them.

    This class is a singleton ROS node and should only be instantiated once.
    """

    def __init__(self, auto_start=True, startup_patterns: list = None):
        self.ready = False
        self.router_names = [enum.value for enum in list(constants.Router)]
        self.active_patterns = []
        self.router_clients = {}
        self.topic_data = {}  # accessed by behavior patterns

        self._subscribers = {}
        self._routers_connected = False
        self._topics_connected = False

        if auto_start:
            self.connect_to_routers()
            if startup_patterns:
                self.setup_behavior_patterns(startup_patterns)

    def _update_ready(self):
        if self._routers_connected and self._topics_connected:
            self.ready = True
        else:
            self.ready = False

    def connect_to_routers(self):
        """Setup behavior clients and subscribers """
        for route in self.router_names:
            self.router_clients[route] = HarmoniActionClient()
        for route, client in self.router_clients.items():
            client.setup_client(route)
        rospy.loginfo("Behavior controller has connected to HARMONI routers.")
        self._routers_connected = True
        self._update_ready()
        return

    def setup_behavior_patterns(self, patterns: list):
        """Setup behavior patterns needed for this session.

        Args:
            patterns(list<string>): All the behavior patterns we may use this session
        """
        ROUTE = 0
        count = 0
        for pattern in patterns:
            for topic_info in pattern.TOPICS:
                if not (topic_info[ROUTE] in self.topic_data):
                    func = None
                    # create a new callback function for each topic
                    cb_name = str(count) + "_callback"
                    callback_def = inspect.cleandoc(
                        """
                        def {}(cls, data):
                            topic = {}
                            cls.topic_data[topic] = data
                        func = {}
                    """.format(
                            cb_name, topic_info[ROUTE], cb_name
                        )
                    )
                    exec(callback_def)
                    setattr(self, cb_name, classmethod(func))

                    self._subscribers[topic_info[ROUTE]] = rospy.Subscriber(
                        *topic_info, cb_name
                    )
                    count += 1
        self._topics_connected = (
            True  # TODO add error handling to provide a robust false case
        )
        self._update_ready()
        return


if __name__ == "__main__":
    try:
        rospy.init_node("harmoni/behavior_node")
        bc = BehaviorController()
        rospy.loginfo("behavior_node started.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
