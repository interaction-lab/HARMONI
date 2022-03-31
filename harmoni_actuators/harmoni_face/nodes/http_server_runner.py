#!/usr/bin/env python3

"""
This script takes two arguements: a path to where it should open the http-server and the desired port.
If no port is specified, http-server will pick.
If the port selected is in use, you will see a long javascript error.
When the script is terminated, it kills the http-server that it has created

Usage example:

    python http_server_runner.py /path/to/dir 8080

"""

import rospy
import subprocess
import atexit
import os
import sys


def is_int(value):
    try:
        int(value)
        return True
    except ValueError:
        return False


DIR_IDX = 1
PORT_IDX = 2
if __name__ == "__main__":
    rospy.init_node("http_server_runner")
    args = sys.argv
    if os.path.exists(args[DIR_IDX]):
        directory = args[DIR_IDX]
        os.chdir(directory)
    else:
        raise ValueError("Invalid directory input")

    command = ["http-server"]
    if is_int(args[PORT_IDX]):
        port = args[PORT_IDX]
        command += ["-p", port]
        #command += ["-S"]

        #cert_path = args[DIR_IDX] + "/server_cert.pem"
        #key_path = args[DIR_IDX] + "/server_key.pem"
        #command += ["-C", cert_path]
        #command += ["-K", key_path]

    p = subprocess.Popen(command)
    atexit.register(p.terminate)

    while not rospy.is_shutdown():
        rospy.sleep(1)
    # rospy.spin()
