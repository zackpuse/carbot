#!/usr/bin/env python
import cv2
import numpy as np
import time

MODEL = "/ssd/models/yolov5/carbot_signage.onnx"

# Class names ikut susunan sebenar dalam data.yaml (nc=19)
CLASS_NAMES = [
    'BoomGate_Closed',
    'BoomGate_Open',
    'Downhill',
    'Enter_Tunnel_sign',
    'Exit_Tunnel_sign',
    'Linechanging_sign',
    'Parking_sign',
    'Roundabout_sign',
    'Slippery_sign',
    'SpeedBump',
    'Speedbump_sign',
    'TrafficLight',
    'TrafficLight_sign',
    'Yellow_light',
    'green_light',
    'parallel_parking_sign',
    'perpendicular_parking_sign',
    'red_light',
    'side_parking'
]

CONF_THRESHOLD = 0.5
NMS_THRESHOLD  = 0.45
INPUT_SIZE     = 640

def get_pipeline(sensor_id, width=640, height=480):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=%d, height=%d, framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=1"
        % (sensor_id, width, height)
    )

def postprocess(output, frame_w, frame_h, num_classes):
    # output shape: (1, N, 5+num_classes)
    predictions = output[0]

    boxes       = []
    confidences = []
    class_ids   = []

    x_scale = frame_w / INPUT_SIZE
    y_scale = frame_h / INPUT_SIZE

    for pred in predictions:
        obj_conf = pred[4]
        if obj_conf < CONF_THRESHOLD:
            continue

        class_scores = pred[5:5+num_classes]
        class_id     = np.argmax(class_scores)
        class_conf   = class_scores[class_id]

        final_conf = obj_conf * class_conf
        if final_conf < CONF_THRESHOLD:
            continue

        cx, cy, w, h = pred[0], pred[1], pred[2], pred[3]

        x1 = int((cx - w/2) * x_scale)
        y1 = int((cy - h/2) * y_scale)
        bw = int(w * x_scale)
        bh = int(h * y_scale)

        boxes.append([x1, y1, bw, bh])
        confidences.append(float(final_conf))
        class_ids.append(class_id)

    if len(boxes) == 0:
        return []

    indices = cv2.dnn.NMSBoxes(boxes, confidences, CONF_THRESHOLD, NMS_THRESHOLD)

    results = []
    if len(indices) > 0:
        for i in indices.flatten():
            results.append({
                'box': boxes[i],
                'conf': confidences[i],
                'class_id': class_ids[i],
                'label': CLASS_NAMES[class_ids[i]] if class_ids[i] < len(CLASS_NAMES) else str(class_ids[i])
            })
    return results

def main():
    cap = cv2.VideoCapture(get_pipeline(0), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("ERROR: Cannot open CAM0")
        return

    print("CAM0 opened")
    print("Loading ONNX model...")

    net = cv2.dnn.readNetFromONNX(MODEL)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

    print("ONNX model loaded")
    print("Press Q to quit")

    num_classes = len(CLASS_NAMES)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        h, w = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (INPUT_SIZE, INPUT_SIZE),
                                       swapRB=True, crop=False)
        net.setInput(blob)

        start = time.time()
        output = net.forward()
        inference_time = (time.time() - start) * 1000

        detections = postprocess(output, w, h, num_classes)

        for det in detections:
            x, y, bw, bh = det['box']
            label = "%s %.2f" % (det['label'], det['conf'])
            cv2.rectangle(frame, (x, y), (x+bw, y+bh), (0, 255, 0), 2)
            cv2.putText(frame, label, (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.putText(frame, "Inference: %.1f ms | Det: %d" % (inference_time, len(detections)),
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        if detections:
            print("Detected:", [d['label'] for d in detections])

        cv2.imshow("CarBot CAM0 - ONNX", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
