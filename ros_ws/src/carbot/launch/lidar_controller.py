#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty

lidar_running = False

def status_callback(msg):
    global lidar_running
    data = msg.data

    # Extract state
    if 'ST:R' in data and not lidar_running:
        # START button ditekan — start LiDAR motor
        try:
            rospy.wait_for_service('/start_motor', timeout=2)
            start = rospy.ServiceProxy('/start_motor', Empty)
            start()
            lidar_running = True
            rospy.loginfo("[LIDAR] Motor STARTED")
        except Exception as e:
            rospy.logerr("[LIDAR] Failed to start: %s" % str(e))

    elif 'ST:S' in data and lidar_running:
        # STOP button ditekan — stop LiDAR motor
        try:
            rospy.wait_for_service('/stop_motor', timeout=2)
            stop = rospy.ServiceProxy('/stop_motor', Empty)
            stop()
            lidar_running = False
            rospy.loginfo("[LIDAR] Motor STOPPED")
        except Exception as e:
            rospy.logerr("[LIDAR] Failed to stop: %s" % str(e))

def main():
    rospy.init_node('lidar_controller')
    rospy.Subscriber('/carbot/status', String, status_callback)
    rospy.loginfo("[LIDAR] Controller ready — waiting for START button")
    rospy.spin()

if __name__ == '__main__':
    main()
