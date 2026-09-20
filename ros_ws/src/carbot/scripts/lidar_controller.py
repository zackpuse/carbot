#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty

lidar_running = False

def stop_lidar():
    try:
        rospy.wait_for_service('/stop_motor', timeout=5)
        rospy.ServiceProxy('/stop_motor', Empty)()
        rospy.loginfo("[LIDAR] Motor STOPPED")
    except Exception as e:
        rospy.logerr("[LIDAR] Stop failed: %s" % str(e))

def start_lidar():
    try:
        rospy.wait_for_service('/start_motor', timeout=5)
        rospy.ServiceProxy('/start_motor', Empty)()
        rospy.loginfo("[LIDAR] Motor STARTED")
    except Exception as e:
        rospy.logerr("[LIDAR] Start failed: %s" % str(e))

def status_callback(msg):
    global lidar_running
    data = msg.data

    if 'ST:R' in data and not lidar_running:
        start_lidar()
        lidar_running = True

    elif ('ST:S' in data or 'ST:E' in data) and lidar_running:
        stop_lidar()
        lidar_running = False

def main():
    rospy.init_node('lidar_controller')
    # Boot - stop motor dulu
    rospy.loginfo("[LIDAR] Boot - stopping motor")
    stop_lidar()
    rospy.Subscriber('/carbot/status', String, status_callback)
    rospy.loginfo("[LIDAR] Ready - waiting for START button")
    rospy.spin()

if __name__ == '__main__':
    main()
