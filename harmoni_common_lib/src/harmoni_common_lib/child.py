#!/usr/bin/env python

# Importing the libraries
import rospy
import roslib
from action_server import HarmoniActionServer


class HardwareControlServer(HarmoniActionServer, object):
    """
    A hardware control provider receives some data which it will formulate
    into an action for the hardware
    """

    def __init__(self, name, service_manager):
        """
        Initialize hardware control, logical defaults, and publishers/subscribers
        Can optionally test connctivity and availability of the hardware
        """

        self.name = name
        self.service_manager = service_manager

        success = self.service_manager.test()
        if success:
            rospy.loginfo("%s has been successfully set up" %name)
        else:
            rospy.logwarn("%s has not been started" %name)
        self.setup_server(name, self.execute_goal_received_callback)

    def update_feedback(self):
        """Update the feedback message """
        rospy.loginfo("Start updating the feedback")
        while not rospy.is_shutdown():
            if self.service_manager.state != 3:
                if self.service_manager.state != 0:
                    self.send_feedback(self.service_manager.state)
                rospy.Rate(.2)
            else:
                break
        return

    def execute_goal_received_callback(self, goal):
        """
        Handle function execute in the goal reiceved callback
        """
        success = True
        self.service_manager.do(goal.optional_data)
        while not self.service_manager.actuation_completed:
            if self.get_preemption_state():
                success = False
                rospy.Rate(10)

        if success:
            if self.service_manager.state == 2: # Success
                rospy.loginfo("Result success")
                self.send_result(
                    do_action=True,
                    message=self.service_manager.result_msg)
                self.service_manager.reset_init()
            elif self.service_manager.state == 3: # Failure
                rospy.loginfo("Result failure")
                self.send_result(
                    do_action=False,
                    message=self.service_manager.result_msg)
                self.service_manager.reset_init()
        return


class WebServiceServer(HarmoniActionServer, object):
    """
    An external service provider receives some data which it will formulate
    into an API request of some cloud provider
    """

    def __init__(self, name, service_manager):
        """
        Initialize web client, logical defaults, and publishers/subscribers
        Can optionally test connctivity with web services
        """
        self.name = name
        self.service_manager = service_manager

        success = self.service_manager.test()
        if success:
            rospy.loginfo("%s has been successfully set up" %name)
        else:
            rospy.logwarn("%s has not been started"%name)
        self.setup_server(name, self.execute_goal_received_callback)

    def update_feedback(self):
        """Update the feedback message """
        rospy.loginfo("Start updating the feedback")
        while not rospy.is_shutdown():
            if self.service_manager.state != 3:
                if self.service_manager.state != 0:
                    self.send_feedback(self.service_manager.state)
                rospy.Rate(.2)
            else:
                break
        return

    def execute_goal_received_callback(self, goal):
        """
        Currently not supporting sending data to external service except through optional_data
        """
        self.service_manager.request(goal.optional_data)  # state is in response_recieved, result in return_msg
        success = True
        while not self.service_manager.response_received:
            if self.get_preemption_state():
                rospy.loginfo("Check prempt")
                success = False
                rospy.Rate(10)
        if success:
            rospy.loginfo("Send result")
            rospy.loginfo("The state is %i" %self.service_manager.state)
            if self.service_manager.state == 2: # Success
                self.send_result(
                    do_action=True,
                    message=self.service_manager.result_msg)
                self.service_manager.reset_init()
            elif self.service_manager.state == 3: # Failure
                self.send_result(
                    do_action=False,
                    message=self.service_manager.result_msg)
                self.service_manager.reset_init()

        return


class InternalServiceServer(HarmoniActionServer, object):
    """
    An Internal Service controls the behavior of a class that processes some
    data from a topic(s) and publishes it to a topic
    """

    def __init__(self, name, service_manager):
        """
        Initialize, control flow of information through processing class
        """
        self.name = name
        self.service_manager = service_manager

        success = self.service_manager.test()
        if success:
            rospy.loginfo("{name} has been successfully set up")
        else:
            rospy.logwarn("{name} has not been started")
        self.setup_server(name, self.execute_goal_received_callback)
        while not rospy.is_shutdown():
            self.send_feedback(self.service_manager.state)
            rospy.Rate(.2)

    def update_feedback(self):
        """Update the feedback message """
        rospy.loginfo("Start updating the feedback")
        while not rospy.is_shutdown():
            if self.service_manager.state != 3:
                if self.service_manager.state != 0:
                    self.send_feedback(self.service_manager.state)
                rospy.Rate(.2)
            else:
                break
        return

    def execute_goal_received_callback(self, goal):
        """Control flow through internal processing class"""
        # TODO better case management
        if goal.action == "start_{self.name}":
            if goal.optional_data != "":
                self.service_manager.start(int(goal.optional_data))
            else:
                self.service_manager.start()
        elif goal.action == "pause_{self.name}":
            self.service_manager.stop()
        elif goal.action == "stop_{self.name}":
            self.service_manager.stop()
            self.service_manager.reset_init
        return


class HarwareReadingServer(HarmoniActionServer, object):
    """
    An hardware reading class controls the behavior of a class that processes some
    data from a sensor and publishes it to a topic
    """

    def __init__(self, name, service_manager):
        """
        Initialize, control flow of information through sensor class
        """
        self.name = name
        rospy.loginfo("The service name is %s" % self.name)
        self.service_manager = service_manager

        success = self.service_manager.test()
        if success:
            rospy.loginfo("%s has been successfully set up" % self.name)
        else:
            rospy.logwarn("%s has not been started" % self.name)
        self.setup_server(name, self.execute_goal_received_callback)
 
    def update_feedback(self):
        """Update the feedback message """
        rospy.loginfo("Start updating the feedback")
        while not rospy.is_shutdown():
            if self.service_manager.state != 3:
                if self.service_manager.state != 0:
                    self.send_feedback(self.service_manager.state)
                rospy.Rate(.2)
            else:
                break
        return

    def execute_goal_received_callback(self, goal):
        """Control flow through internal processing class"""
        # TODO better case management here
        if goal.action == "start_%s" % self.name:
            if goal.optional_data != "":
                self.service_manager.start(int(goal.optional_data))
            else:
                self.service_manager.start()
        elif goal.action == "pause_%s" % self.name:
            self.service_manager.pause()
        elif goal.action == "stop_%s" % self.name:
            self.service_manager.stop()
            self.service_manager.reset_init
        return
