#!/usr/bin/env python
# Lane Following - garisan putih kiri/kanan sebagai reference
# HSV tuning via dynamic_reconfigure + persist ke fail JSON
# + Publish mask/overlay image untuk visual debug (tanpa perlu cv2.imshow)
import rospy
import cv2
import numpy as np
import json
import os
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from carbot.cfg import LaneFollowConfig

# ==================== PERSIST FILE ====================
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
    'lp_alpha': 0.30,
    'lane_width_ratio': 0.50,
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
            rospy.loginfo("[LANE] Config dimuat dari %s" % CONFIG_FILE)
            return True
        except Exception as e:
            rospy.logwarn("[LANE] Gagal load config fail: %s - guna default" % str(e))
    else:
        rospy.loginfo("[LANE] Tiada config fail sebelum ini - guna default")
    return False

def save_config_to_file():
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        save_data = {k: v for k, v in cfg.items() if k != 'save_config'}
        with open(CONFIG_FILE, 'w') as f:
            json.dump(save_data, f, indent=2)
        rospy.loginfo("[LANE] Config disimpan ke %s" % CONFIG_FILE)
        return True
    except Exception as e:
        rospy.logerr("[LANE] Gagal save config: %s" % str(e))
        return False

# ==================== DYNAMIC RECONFIGURE ====================
srv = None

def reconfigure_callback(config, level):
    global cfg
    cfg.update(config)

    if cfg.get('save_config', False):
        save_config_to_file()
        config['save_config'] = False
        cfg['save_config'] = False

    rospy.loginfo("[LANE] Config updated: H[%d-%d] S[%d-%d] V[%d-%d] KP=%.4f speed=%.2f" %
                  (cfg['h_min'], cfg['h_max'], cfg['s_min'], cfg['s_max'],
                   cfg['v_min'], cfg['v_max'], cfg['kp'], cfg['base_speed']))
    return config

# ==================== STATE ====================
CARBOT_RUNNING        = False
TUNNEL_ACTIVE         = False
CURRENT_MISSION_PHASE = ""  # Dari /mission/phase - lane_follow diam bila bukan fasa memandu

# Fasa-fasa di mana lane_follow DIBENARKAN publish /cmd_vel.
# Di luar senarai ini (WAIT_*, PARK_*, MISSION_DONE), lane_follow DIAM.
LANE_ACTIVE_PHASES = {
    "LANE_TO_CHANGE", "LANE_CHANGED", "LANE_TO_ROUND",
    "ROUNDABOUT",     "LANE_TO_TUNNEL", "LANE_TO_GATE2",
    "HILL",           "LANE_TO_LIGHT",  "LANE_TO_CHANGE2",
    "LANE_TO_ROUND2", "ROUNDABOUT2",
    "LANE_TO_PARALLEL", "LANE_TO_PERP",
}

# Ultrasonic safety stop - berhenti bila terlalu dekat objek/palang
# (data dari Arduino via /carbot/status, format: "... US:14.5 ...")
US_STOP_CM  = 15.0   # cm - berhenti bila US < nilai ini
current_us  = 999.0  # cm - nilai lalai: jauh (tiada halangan)

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
    output = max(-max_s, min(max_s, output))
    return output

# ==================== IMAGE CONVERSION ====================
def image_msg_to_numpy(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8)
    img = img.reshape(msg.height, msg.width, 3)
    return img

def numpy_to_image_msg(frame, encoding="bgr8"):
    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height = frame.shape[0]
    msg.width  = frame.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    if encoding == "mono8":
        msg.step = frame.shape[1]
    else:
        msg.step = frame.shape[1] * 3
    msg.data = frame.tobytes()
    return msg

# ==================== LANE DETECTION ====================
def get_binary_mask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([cfg['h_min'], cfg['s_min'], cfg['v_min']])
    upper = np.array([cfg['h_max'], cfg['s_max'], cfg['v_max']])
    mask = cv2.inRange(hsv, lower, upper)
    return mask

def get_roi(mask):
    h, w = mask.shape
    top = int(h * cfg['roi_top_ratio'])
    roi = mask[top:h, :]
    return roi, top

def sliding_window_search(binary_roi):
    h, w = binary_roi.shape
    num_windows   = cfg['num_windows']
    window_margin = cfg['window_margin']
    min_pixels    = cfg['min_pixels']

    histogram = np.sum(binary_roi[h//2:, :], axis=0)
    midpoint  = w // 2

    left_base  = np.argmax(histogram[:midpoint])
    right_base = np.argmax(histogram[midpoint:]) + midpoint

    window_height = h // num_windows

    nonzero   = binary_roi.nonzero()
    nonzero_y = np.array(nonzero[0])
    nonzero_x = np.array(nonzero[1])

    left_current  = left_base
    right_current = right_base

    left_pixel_x  = []
    right_pixel_x = []

    for window in range(num_windows):
        win_y_low  = h - (window + 1) * window_height
        win_y_high = h - window * window_height

        win_xl_low  = left_current  - window_margin
        win_xl_high = left_current  + window_margin
        win_xr_low  = right_current - window_margin
        win_xr_high = right_current + window_margin

        good_left = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                     (nonzero_x >= win_xl_low) & (nonzero_x < win_xl_high)).nonzero()[0]
        good_right = ((nonzero_y >= win_y_low) & (nonzero_y < win_y_high) &
                      (nonzero_x >= win_xr_low) & (nonzero_x < win_xr_high)).nonzero()[0]

        if len(good_left) > min_pixels:
            left_current = int(np.mean(nonzero_x[good_left]))
            left_pixel_x.extend(nonzero_x[good_left])

        if len(good_right) > min_pixels:
            right_current = int(np.mean(nonzero_x[good_right]))
            right_pixel_x.extend(nonzero_x[good_right])

    left_pos  = int(np.mean(left_pixel_x))  if len(left_pixel_x)  > 0 else None
    right_pos = int(np.mean(right_pixel_x)) if len(right_pixel_x) > 0 else None

    return left_pos, right_pos

# ==================== NAVIGATION ====================
def navigate(frame):
    global steer_prev

    h, w = frame.shape[:2]
    mask = get_binary_mask(frame)
    roi, roi_top = get_roi(mask)

    left_pos, right_pos = sliding_window_search(roi)

    center_of_lane = None
    mode = "NO_LINE"

    if left_pos is not None and right_pos is not None:
        center_of_lane = (left_pos + right_pos) / 2.0
        mode = "DUAL_LINE"
    elif left_pos is not None:
        lane_width_px = w * cfg['lane_width_ratio']
        center_of_lane = left_pos + lane_width_px / 2.0
        mode = "LEFT_ONLY"
    elif right_pos is not None:
        lane_width_px = w * cfg['lane_width_ratio']
        center_of_lane = right_pos - lane_width_px / 2.0
        mode = "RIGHT_ONLY"

    twist = Twist()
    max_s = cfg['max_steer']
    steer_out = 0.0
    speed_out = cfg['min_speed']

    if center_of_lane is None:
        twist.linear.x  = cfg['min_speed']
        twist.angular.z = steer_prev * 0.5
        pub_cmd.publish(twist)
        steer_out = twist.angular.z
        speed_out = twist.linear.x
    else:
        frame_center = w / 2.0
        error_px = center_of_lane - frame_center

        if abs(error_px) < cfg['steer_deadzone']:
            raw_steer = 0.0
        else:
            raw_steer = -pid_steer(error_px)

        alpha = cfg['lp_alpha']
        steer = alpha * raw_steer + (1 - alpha) * steer_prev
        steer_prev = steer

        speed = cfg['base_speed'] * (1.0 - abs(steer) / max_s)
        speed = max(cfg['min_speed'], speed)

        twist.linear.x  = speed
        twist.angular.z = steer
        pub_cmd.publish(twist)
        steer_out = steer
        speed_out = speed

    publish_debug(mode, left_pos, right_pos, steer_out, speed_out)

    # ---- Publish mask & overlay untuk visual debug (rqt_image_view / web_video_server) ----
    if pub_mask is not None:
        try:
            pub_mask.publish(numpy_to_image_msg(mask, encoding="mono8"))
        except Exception:
            pass

    if pub_overlay is not None:
        try:
            overlay = frame.copy()
            if left_pos is not None:
                cv2.circle(overlay, (left_pos, roi_top + 50), 8, (255, 0, 0), -1)
            if right_pos is not None:
                cv2.circle(overlay, (right_pos, roi_top + 50), 8, (0, 0, 255), -1)
            if center_of_lane is not None:
                cv2.circle(overlay, (int(center_of_lane), roi_top + 50), 8, (0, 255, 0), -1)
            cv2.line(overlay, (int(w/2), 0), (int(w/2), h), (0, 255, 255), 1)
            cv2.line(overlay, (0, roi_top), (w, roi_top), (255, 255, 0), 1)
            cv2.putText(overlay, "%s steer:%.2f spd:%.2f" % (mode, steer_out, speed_out),
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            pub_overlay.publish(numpy_to_image_msg(overlay, encoding="bgr8"))
        except Exception:
            pass

def publish_debug(mode, left_pos, right_pos, steer, speed):
    if pub_debug is None:
        return
    msg = String()
    msg.data = "mode:%s L:%s R:%s steer:%+.3f spd:%.2f" % (
        mode,
        str(left_pos) if left_pos is not None else "-",
        str(right_pos) if right_pos is not None else "-",
        steer, speed
    )
    pub_debug.publish(msg)

def stop_carbot():
    reset_pid()
    twist = Twist()
    twist.linear.x  = 0.0
    twist.angular.z = 0.0
    pub_cmd.publish(twist)

# ==================== CALLBACKS ====================
def status_callback(msg):
    global CARBOT_RUNNING, current_us
    data = msg.data

    if 'ST:R' in data:
        CARBOT_RUNNING = True
    elif 'ST:S' in data or 'ST:E' in data:
        CARBOT_RUNNING = False
        stop_carbot()

    # Parse bacaan ultrasonik dari Arduino (format: "US:14.5")
    for part in data.split():
        if part.startswith('US:'):
            try:
                current_us = float(part.split(':', 1)[1])
            except ValueError:
                pass

def tunnel_active_callback(msg):
    """Subscribe /tunnel_nav/active — stop lane_follow bila tunnel_nav ambil alih."""
    global TUNNEL_ACTIVE
    TUNNEL_ACTIVE = msg.data
    if TUNNEL_ACTIVE:
        reset_pid()   # clear PID supaya steer bersih bila masuk balik selepas tunnel
        rospy.loginfo("[LANE] Tunnel aktif — lane_follow yield cmd_vel")
    else:
        rospy.loginfo("[LANE] Tunnel selesai — lane_follow ambil semula cmd_vel")

def mission_phase_callback(msg):
    """Subscribe /mission/phase — lane_follow hanya aktif pada fasa memandu."""
    global CURRENT_MISSION_PHASE
    prev = CURRENT_MISSION_PHASE
    CURRENT_MISSION_PHASE = msg.data
    if prev != CURRENT_MISSION_PHASE:
        in_active = CURRENT_MISSION_PHASE in LANE_ACTIVE_PHASES
        rospy.loginfo("[LANE] Fasa: %s -> %s | lane_follow %s" % (
            prev, CURRENT_MISSION_PHASE, "AKTIF" if in_active else "DIAM"))
        if not in_active:
            reset_pid()  # bersihkan PID supaya tidak ada steer terkumpul bila mula balik

def image_callback(msg):
    if not CARBOT_RUNNING or TUNNEL_ACTIVE:
        return
    # Diam bila berada di fasa WAIT_*, PARK_*, atau MISSION_DONE
    # (elak berebut /cmd_vel dengan parking_nav atau melanggar palang)
    if CURRENT_MISSION_PHASE and CURRENT_MISSION_PHASE not in LANE_ACTIVE_PHASES:
        return
    # Ultrasonic safety stop - berhenti dan tahan bila terlalu dekat halangan
    # (terutama BoomGate yang masih tertutup semasa LANE_TO_GATE2)
    if current_us < US_STOP_CM:
        rospy.logwarn_throttle(2.0, "[LANE] US STOP: %.1fcm < %.0fcm - menunggu laluan terbuka" %
                               (current_us, US_STOP_CM))
        stop_carbot()
        return
    try:
        frame = image_msg_to_numpy(msg)
        navigate(frame)
    except Exception as e:
        rospy.logerr("[LANE] Error: %s" % str(e))

# ==================== MAIN ====================
def main():
    global pub_cmd, pub_debug, pub_mask, pub_overlay, srv

    rospy.init_node('lane_follow')

    load_config_from_file()

    srv = Server(LaneFollowConfig, reconfigure_callback)
    srv.update_configuration(cfg)

    pub_cmd     = rospy.Publisher('/cmd_vel',                 Twist,  queue_size=1)
    pub_debug   = rospy.Publisher('/lane_follow/debug',        String, queue_size=1)
    pub_mask    = rospy.Publisher('/lane_follow/mask_image',   Image,  queue_size=1)
    pub_overlay = rospy.Publisher('/lane_follow/overlay_image',Image,  queue_size=1)

    rospy.Subscriber('/carbot/status',          String, status_callback)
    rospy.Subscriber('/tunnel_nav/active',       Bool,   tunnel_active_callback)
    rospy.Subscriber('/mission/phase',           String, mission_phase_callback)
    rospy.Subscriber('/camera/front/image_raw', Image,  image_callback, queue_size=1)

    rospy.loginfo("[LANE] Lane Following ready")
    rospy.loginfo("[LANE] Config file: %s" % CONFIG_FILE)
    rospy.loginfo("[LANE] Visual debug: /lane_follow/mask_image dan /lane_follow/overlay_image")
    rospy.loginfo("[LANE] Skip aktif bila /tunnel_nav/active=True (tunnel_nav ambil alih)")
    rospy.loginfo("[LANE] Ultrasonic stop: berhenti bila US < %.0fcm" % US_STOP_CM)
    rospy.spin()

if __name__ == '__main__':
    main()