#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Camera state
cam0_active = False
cam1_active = False
cap0 = None
cap1 = None
camera_should_run = False

def get_pipeline(sensor_id, width=640, height=480):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=%d, height=%d, framerate=30/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
        % (sensor_id, width, height)
    )

def numpy_to_image_msg(frame, frame_id):
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id
    msg.height = frame.shape[0]
    msg.width  = frame.shape[1]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    return msg

def open_cameras():
    global cap0, cap1, cam0_active, cam1_active

    # CAM0 depan - sensor-id=0 (TUKAR dari 1)
    cap0 = cv2.VideoCapture(get_pipeline(0), cv2.CAP_GSTREAMER)
    if cap0.isOpened():
        cam0_active = True
        rospy.loginfo("[CAM] CAM0 depan ready")
    else:
        rospy.logerr("[CAM] CAM0 gagal buka")

    # CAM1 belakang - sensor-id=1 (TUKAR dari 0)
    cap1 = cv2.VideoCapture(get_pipeline(1), cv2.CAP_GSTREAMER)
    if cap1.isOpened():
        cam1_active = True
        rospy.loginfo("[CAM] CAM1 belakang ready")
    else:
        rospy.logerr("[CAM] CAM1 gagal buka")


def close_cameras():
    global cap0, cap1, cam0_active, cam1_active
    if cap0 is not None:
        cap0.release()
        cap0 = None
    if cap1 is not None:
        cap1.release()
        cap1 = None
    cam0_active = False
    cam1_active = False
    rospy.loginfo("[CAM] Semua kamera ditutup")

def status_callback(msg):
    global camera_should_run
    data = msg.data

    if 'ST:R' in data and not camera_should_run:
        camera_should_run = True
        rospy.loginfo("[CAM] START - buka kamera")
        open_cameras()

    elif ('ST:S' in data or 'ST:E' in data) and camera_should_run:
        camera_should_run = False
        rospy.loginfo("[CAM] STOP - tutup kamera")
        close_cameras()

def main():
    rospy.init_node('camera_node')

    pub_cam1 = rospy.Publisher('/camera/front/image_raw', Image, queue_size=1)
    pub_cam0 = rospy.Publisher('/camera/rear/image_raw',  Image, queue_size=1)

    rospy.Subscriber('/carbot/status', String, status_callback)

    rospy.loginfo("[CAM] Camera node ready - tunggu START button")

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if cam0_active and cap0 is not None:
            ret0, frame0 = cap0.read()
            if ret0:
                pub_cam1.publish(numpy_to_image_msg(frame0, "camera_front"))

        if cam1_active and cap1 is not None:
            ret1, frame1 = cap1.read()
            if ret1:
                pub_cam0.publish(numpy_to_image_msg(frame1, "camera_rear"))

        rate.sleep()

    close_cameras()

if __name__ == '__main__':
    main()
