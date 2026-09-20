#!/usr/bin/env python
import rospy
import os
from std_msgs.msg import String

shutdown_triggered = False

def status_callback(msg):
    global shutdown_triggered
    if 'SHUTDOWN' in msg.data and not shutdown_triggered:
        shutdown_triggered = True
        rospy.loginfo("[SHUTDOWN] Stopping NoMachine to prevent DB corruption...")
        os.system('sudo /usr/NX/bin/nxserver --stop')
        rospy.loginfo("[SHUTDOWN] Shutting down Jetson...")
        os.system('sudo /sbin/shutdown -h now')

def main():
    rospy.init_node('shutdown_listener')
    rospy.Subscriber('/carbot/status', String, status_callback)
    rospy.loginfo("[SHUTDOWN] Listener ready")
    rospy.spin()

if __name__ == '__main__':
    main()
