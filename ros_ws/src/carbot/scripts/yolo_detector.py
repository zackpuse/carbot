#!/usr/bin/env python
import rospy
import cv2
import torch
import numpy as np
import sys
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Setup path YOLOv5
YOLO_PATH = "/ssd/models/yolov5"
sys.path.insert(0, YOLO_PATH)

# Suppress matplotlib backend warning
import matplotlib
matplotlib.use('Agg')

model = None
pub_result = None
pub_annotated = None

def image_msg_to_numpy(msg):
    # Convert ROS Image message to numpy array tanpa cv_bridge
    dtype = np.uint8
    img = np.frombuffer(msg.data, dtype=dtype)
    img = img.reshape(msg.height, msg.width, 3)
    return img  # BGR format

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

def load_model():
    global model
    try:
        model = torch.hub.load(
            YOLO_PATH,
            'custom',
            path='/ssd/models/yolov5/carbot_signage.pt',
            source='local',
            force_reload=False,
            verbose=False
        )
        model.conf = 0.5
        model.iou  = 0.45
        model.to('cuda' if torch.cuda.is_available() else 'cpu')
        rospy.loginfo("[YOLO] Model loaded - CUDA: %s" % str(torch.cuda.is_available()))
    except Exception as e:
        rospy.logerr("[YOLO] Model load failed: %s" % str(e))

def cam0_callback(msg):
    if model is None:
        return
    try:
        # Convert tanpa cv_bridge
        frame = image_msg_to_numpy(msg)

        # BGR to RGB untuk YOLO
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Run inference
        results = model(img_rgb, size=640)
        detections = results.pandas().xyxy[0]

        # Build result string
        result_str = "CAM:front"
        for _, det in detections.iterrows():
            label = det['name']
            conf  = det['confidence']
            result_str += " %s:%.2f" % (label, conf)

            # Draw bounding box
            x1 = int(det['xmin'])
            y1 = int(det['ymin'])
            x2 = int(det['xmax'])
            y2 = int(det['ymax'])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame,
                        "%s %.2f" % (label, conf),
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

        # Publish result
        result_msg = String()
        result_msg.data = result_str
        pub_result.publish(result_msg)

        # Publish annotated image
        pub_annotated.publish(numpy_to_image_msg(frame, "camera_front"))

    except Exception as e:
        rospy.logerr("[YOLO] Error: %s" % str(e))

def main():
    global pub_result, pub_annotated

    rospy.init_node('yolo_detector')

    pub_result    = rospy.Publisher('/yolo/detections',      String, queue_size=1)
    pub_annotated = rospy.Publisher('/yolo/annotated_image', Image,  queue_size=1)

    # Load model sebelum subscribe
    rospy.loginfo("[YOLO] Loading model...")
    load_model()

    if model is None:
        rospy.logerr("[YOLO] Model gagal load - node berhenti")
        return

    rospy.Subscriber('/camera/front/image_raw', Image, cam0_callback, queue_size=1)
    rospy.loginfo("[YOLO] Detector ready - listening /camera/front/image_raw")
    rospy.spin()

if __name__ == '__main__':
    main()
