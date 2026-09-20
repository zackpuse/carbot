#!/usr/bin/env python
# Lane Following v2 - FIX: centering bias
# Perubahan dari v1:
#   1. left_pos/right_pos guna BOTTOM WINDOWS sahaja (elak perspektif bias)
#   2. Histogram check - kalau kosong jangan bagi base yang salah
#   3. Tambah center_offset_px untuk kompensasi kamera tak centred
#   4. lp_alpha default naikkan ke 0.50 (lebih responsif)
import rospy
import cv2
import numpy as np
import json
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from carbot.cfg import LaneFollowConfig

CONFIG_FILE = "/ssd/config/lane_follow_config.json"

DEFAULT_CFG = {
    'h_min': 0,   'h_max': 180,
    's_min': 0,   's_max': 60,
    'v_min': 180, 'v_max': 255,
    'roi_top_ratio': 0.55,
    'num_windows': 9,
    'window_margin': 60,
    'min_pixels': 40,
    'kp': 0.008, 'ki': 0.0001, 'kd': 0.004,
    'max_steer': 0.6,
    'base_speed': 0.20,
    'min_speed': 0.10,
    'steer_deadzone': 5,
    'lp_alpha': 0.50,           # FIX: 0.30 -> 0.50 (lebih responsif)
    'lane_width_ratio': 0.50,
    'center_offset_px': 0.0,    # FIX: shift frame_center (+ = kanan, - = kiri)
    'bottom_windows': 3,        # FIX: berapa windows bawah untuk kira position
    'save_config': False,
}

cfg = dict(DEFAULT_CFG)

def load_config_from_file():
    global cfg
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                saved = json.load(f)
            cfg.update(saved)
            rospy.loginfo("[LANE_V2] Config dimuat dari %s" % CONFIG_FILE)
            return True
        except Exception as e:
            rospy.logwarn("[LANE_V2] Gagal load config: %s" % str(e))
    return False

def save_config_to_file():
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        save_data = {k: v for k, v in cfg.items() if k != 'save_config'}
        with open(CONFIG_FILE, 'w') as f:
            json.dump(save_data, f, indent=2)
        rospy.loginfo("[LANE_V2] Config disimpan")
    except Exception as e:
        rospy.logerr("[LANE_V2] Gagal save: %s" % str(e))

srv = None

def reconfigure_callback(config, level):
    global cfg
    cfg.update(config)
    if cfg.get('save_config', False):
        save_config_to_file()
        config['save_config'] = False
        cfg['save_config'] = False
    rospy.loginfo("[LANE_V2] Config: KP=%.4f alpha=%.2f offset_px=%.1f bottom_win=%d" %
                  (cfg['kp'], cfg['lp_alpha'], cfg['center_offset_px'], cfg['bottom_windows']))
    return config

CARBOT_RUNNING = False
TUNNEL_ACTIVE  = False

pid_error_prev = 0.0
pid_integral   = 0.0
pid_last_time  = None
steer_prev     = 0.0

pub_cmd     = None
pub_debug   = None
pub_mask    = None
pub_overlay = None

def reset_pid():
    global pid_error_prev, pid_integral, pid_last_time, steer_prev
    pid_error_prev = 0.0
    pid_integral   = 0.0
    pid_last_time  = None
    steer_prev     = 0.0

def pid_steer(error):
    global pid_error_prev, pid_integral, pid_last_time
    now = rospy.Time.now().to_sec()
    if pid_last_time is None:
        pid_last_time = now
        dt = 0.03
    else:
        dt = now - pid_last_time
        if dt <= 0.001:
            dt = 0.03
        pid_last_time = now
    pid_integral += error * dt
    pid_integral  = max(-500, min(500, pid_integral))
    derivative     = (error - pid_error_prev) / dt
    pid_error_prev = error
    max_s = cfg['max_steer']
    output = cfg['kp'] * error + cfg['ki'] * pid_integral + cfg['kd'] * derivative
    return max(-max_s, min(max_s, output))

def image_msg_to_numpy(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8)
    return img.reshape(msg.height, msg.width, 3)

def numpy_to_image_msg(frame, encoding="bgr8"):
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height = frame.shape[0]
    msg.width  = frame.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = frame.shape[1] if encoding == "mono8" else frame.shape[1] * 3
    msg.data = frame.tobytes()
    return msg

def get_binary_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([cfg['h_min'], cfg['s_min'], cfg['v_min']])
    upper = np.array([cfg['h_max'], cfg['s_max'], cfg['v_max']])
    return cv2.inRange(hsv, lower, upper)

def get_roi(mask):
    h, w = mask.shape
    top = int(h * cfg['roi_top_ratio'])
    return mask[top:h, :], top

def sliding_window_search(binary_roi):
    h, w = binary_roi.shape
    num_windows   = cfg['num_windows']
    window_margin = cfg['window_margin']
    min_pixels    = cfg['min_pixels']
    bottom_n      = max(1, int(cfg['bottom_windows']))

    # FIX 1: Semak histogram kosong sebelum argmax
    # np.argmax pada array kosong (semua 0) return 0 — ini bagi base yang salah
    histogram = np.sum(binary_roi[h//2:, :], axis=0)
    midpoint  = w // 2

    left_hist  = histogram[:midpoint]
    right_hist = histogram[midpoint:]

    left_base  = int(np.argmax(left_hist))             if left_hist.max()  > 0 else None
    right_base = int(np.argmax(right_hist)) + midpoint if right_hist.max() > 0 else None

    if left_base is None and right_base is None:
        return None, None

    window_height = h // num_windows

    nonzero   = binary_roi.nonzero()
    nonzero_y = np.array(nonzero[0])
    nonzero_x = np.array(nonzero[1])

    left_current  = left_base  if left_base  is not None else midpoint // 2
    right_current = right_base if right_base is not None else midpoint + midpoint // 2

    # FIX 2: Simpan mean_x tiap window berasingan (bukan kumpul semua pixel)
    left_windows  = []   # index 0 = window paling bawah (dekat kamera)
    right_windows = []

    for window in range(num_windows):
        win_y_low  = h - (window + 1) * window_height
        win_y_high = h - window * window_height

        good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                     (nonzero_x >= left_current  - window_margin) &
                     (nonzero_x <  left_current  + window_margin)).nonzero()[0]
        good_right = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                      (nonzero_x >= right_current - window_margin) &
                      (nonzero_x <  right_current + window_margin)).nonzero()[0]

        if len(good_left) > min_pixels:
            mx = int(np.mean(nonzero_x[good_left]))
            left_current = mx
            left_windows.append(mx)

        if len(good_right) > min_pixels:
            mx = int(np.mean(nonzero_x[good_right]))
            right_current = mx
            right_windows.append(mx)

    # FIX 2: Guna BOTTOM N windows sahaja
    # Window index 0 = paling bawah = paling dekat kamera = perspektif distortion minimum
    left_pos  = int(np.mean(left_windows[:bottom_n]))  if len(left_windows)  > 0 else None
    right_pos = int(np.mean(right_windows[:bottom_n])) if len(right_windows) > 0 else None

    return left_pos, right_pos

def navigate(frame):
    global steer_prev

    h, w = frame.shape[:2]
    mask = get_binary_mask(frame)
    roi, roi_top = get_roi(mask)

    left_pos, right_pos = sliding_window_search(roi)

    center_of_lane = None
    error_px       = 0.0
    mode = "NO_LINE"

    if left_pos is not None and right_pos is not None:
        center_of_lane = (left_pos + right_pos) / 2.0
        mode = "DUAL_LINE"
    elif left_pos is not None:
        lane_width_px  = w * cfg['lane_width_ratio']
        center_of_lane = left_pos + lane_width_px / 2.0
        mode = "LEFT_ONLY"
    elif right_pos is not None:
        lane_width_px  = w * cfg['lane_width_ratio']
        center_of_lane = right_pos - lane_width_px / 2.0
        mode = "RIGHT_ONLY"

    twist = Twist()
    max_s     = cfg['max_steer']
    steer_out = 0.0
    speed_out = cfg['min_speed']

    if center_of_lane is None:
        twist.linear.x  = cfg['min_speed']
        twist.angular.z = steer_prev * 0.5
        pub_cmd.publish(twist)
        steer_out = twist.angular.z
        speed_out = twist.linear.x
    else:
        # FIX 3: center_offset_px — tuning via dynamic_reconfigure
        # Kalau car selalu ke KANAN: naikkan offset_px (positive) supaya
        # reference center bergerak ke kanan, car akan adjust ke kiri
        frame_center = w / 2.0 + cfg['center_offset_px']
        error_px     = center_of_lane - frame_center

        if abs(error_px) < cfg['steer_deadzone']:
            raw_steer = 0.0
        else:
            raw_steer = -pid_steer(error_px)

        alpha      = cfg['lp_alpha']
        steer      = alpha * raw_steer + (1 - alpha) * steer_prev
        steer_prev = steer

        speed = cfg['base_speed'] * (1.0 - abs(steer) / max_s)
        speed = max(cfg['min_speed'], speed)

        twist.linear.x  = speed
        twist.angular.z = -steer   # FIX: sign terbalik pada robot ini
        pub_cmd.publish(twist)
        steer_out = steer
        speed_out = speed

    publish_debug(mode, left_pos, right_pos, steer_out, speed_out, error_px)

    if pub_mask is not None:
        try:
            pub_mask.publish(numpy_to_image_msg(mask, encoding="mono8"))
        except Exception:
            pass

    if pub_overlay is not None:
        try:
            overlay     = frame.copy()
            h_ov, w_ov = overlay.shape[:2]
            draw_y      = roi_top + 50

            if left_pos is not None:
                cv2.circle(overlay, (left_pos, draw_y), 10, (255, 80, 0), -1)
                cv2.putText(overlay, "L:%d" % left_pos,
                            (left_pos - 20, draw_y - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 80, 0), 2)
            if right_pos is not None:
                cv2.circle(overlay, (right_pos, draw_y), 10, (0, 80, 255), -1)
                cv2.putText(overlay, "R:%d" % right_pos,
                            (right_pos - 20, draw_y - 18),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 80, 255), 2)
            if center_of_lane is not None:
                cv2.circle(overlay, (int(center_of_lane), draw_y), 10, (0, 255, 0), -1)

            # Garisan cyan = frame_center (dengan offset)
            fc = int(w_ov / 2.0 + cfg['center_offset_px'])
            cv2.line(overlay, (fc, 0), (fc, h_ov), (0, 255, 255), 2)
            cv2.line(overlay, (0, roi_top), (w_ov, roi_top), (255, 255, 0), 1)

            cv2.putText(overlay,
                        "[v2] %s err:%.0fpx steer:%+.2f" % (mode, error_px, steer_out),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(overlay,
                        "offset:%.0fpx alpha:%.2f bot_win:%d" % (
                            cfg['center_offset_px'], cfg['lp_alpha'], cfg['bottom_windows']),
                        (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1)

            pub_overlay.publish(numpy_to_image_msg(overlay, encoding="bgr8"))
        except Exception:
            pass

def publish_debug(mode, left_pos, right_pos, steer, speed, error_px):
    if pub_debug is None:
        return
    msg = String()
    msg.data = "[v2] mode:%s L:%s R:%s err:%.0fpx steer:%+.3f spd:%.2f" % (
        mode,
        str(left_pos)  if left_pos  is not None else "-",
        str(right_pos) if right_pos is not None else "-",
        error_px, steer, speed
    )
    pub_debug.publish(msg)

def stop_carbot():
    reset_pid()
    twist = Twist()
    twist.linear.x  = 0.0
    twist.angular.z = 0.0
    pub_cmd.publish(twist)

def status_callback(msg):
    global CARBOT_RUNNING, TUNNEL_ACTIVE
    data = msg.data
    if 'ST:R' in data:
        CARBOT_RUNNING = True
    elif 'ST:S' in data or 'ST:E' in data:
        CARBOT_RUNNING = False
        stop_carbot()
    TUNNEL_ACTIVE = ('TUN:1' in data)

def image_callback(msg):
    if not CARBOT_RUNNING or TUNNEL_ACTIVE:
        return
    try:
        frame = image_msg_to_numpy(msg)
        navigate(frame)
    except Exception as e:
        rospy.logerr("[LANE_V2] Error: %s" % str(e))

def main():
    global pub_cmd, pub_debug, pub_mask, pub_overlay, srv
    rospy.init_node('lane_follow_v2')
    load_config_from_file()
    srv = Server(LaneFollowConfig, reconfigure_callback)
    srv.update_configuration(cfg)
    pub_cmd     = rospy.Publisher('/cmd_vel',                    Twist,  queue_size=1)
    pub_debug   = rospy.Publisher('/lane_follow/debug',           String, queue_size=1)
    pub_mask    = rospy.Publisher('/lane_follow/mask_image',      Image,  queue_size=1)
    pub_overlay = rospy.Publisher('/lane_follow/overlay_image',   Image,  queue_size=1)
    rospy.Subscriber('/carbot/status',          String, status_callback)
    rospy.Subscriber('/camera/front/image_raw', Image,  image_callback, queue_size=1)
    rospy.loginfo("[LANE_V2] Lane Follow v2 ready")
    rospy.loginfo("[LANE_V2] bottom_windows=%d  offset_px=%.1f  alpha=%.2f" %
                  (cfg['bottom_windows'], cfg['center_offset_px'], cfg['lp_alpha']))
    rospy.spin()

if __name__ == '__main__':
    main()
