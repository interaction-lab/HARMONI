#!/usr/bin/env python                                                           
import rospy
import subprocess
from std_srvs.srv import Trigger, TriggerResponse
import os

def callback_srv(data):
    os.system("pkill luakit")
    process = subprocess.Popen("luakit --display=:0 -U http://192.168.100.2:8081/KiwiLite.html", shell=True)
    # process = subprocess.Popen("echo hi", shell=True)

    resp = TriggerResponse()
    resp.success = True
    resp.message = "Browser started with PID: %d" % process.pid
    return resp

if __name__ == "__main__":
    rospy.init_node("pi_browser_starter_server")
    srv = rospy.Service('start_pi_browser', Trigger, callback_srv)
    print("ready")
    rospy.spin()
