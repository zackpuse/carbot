#!/usr/bin/env python
# Tunnel Navigation - 100% LiDAR based
# Entry: dual-wall detected, width 50-70cm
# Selekoh (L-shape): satu dinding sahaja - kekal dalam tunnel, guna anggaran
#                     lebar tunnel untuk kekal center
# Exit: KEDUA-DUA dinding tiada dalam radius 40cm

import rospy
import math
from collections import deque
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ==================== CONFIG ====================
CARBOT_RUNNING = False
TUNNEL_ACTIVE  = False

# Wall classification zones (degrees, 0=depan)
LEFT_ZONE_MIN  = 20
LEFT_ZONE_MAX  = 140
RIGHT_ZONE_MIN = -140
RIGHT_ZONE_MAX = -20

# Wall detection range
WALL_MAX_RANGE = 0.70   # jangan kira dinding lebih dari ini (elak salah kesan bilik luas)
WALL_MIN_RANGE = 0.04

# Tunnel width (entry validation - dual wall)
TUNNEL_MIN_WIDTH = 0.40
TUNNEL_MAX_WIDTH = 0.70
TUNNEL_WIDTH_ESTIMATE = 0.55   # anggaran untuk kira center semasa single-wall (selekoh)

# Exit condition - KEDUA dinding kena > EXIT_RADIUS
EXIT_RADIUS = 0.40

# Hysteresis frames
ENTRY_HYSTERESIS_FRAMES = 2
EXIT_HYSTERESIS_FRAMES  = 4   # lebih lama drpd entry - elak exit palsu semasa selekoh sekejap

# PID center gains (dual-wall mode)
KP = 2.8
KI = 0.05
KD = 0.8

# PID single-wall mode (selekoh) - guna gain berasingan, biasanya lebih agresif
KP_SINGLE = 2.8
KD_SINGLE = 0.6

# Wall bias (terlalu dekat dinding - safety push)
KP_WALL = 8.0
MIN_WALL_DIST = 0.22

# Steering
MAX_STEER      = 1.0
STEER_DEADZONE = 0.05
STEER_SIGN     = 1.0   # tukar ke -1.0 kalau arah terbalik

# Front obstacle / stop
FRONT_STOP_DIST = 0.40

# Speed
TUNNEL_SPEED = 0.15
MIN_SPEED    = 0.06

# Filter
FILTER_SIZE = 3
LP_ALPHA    = 0.85

# ==================== FILTER ====================
filter_left  = deque(maxlen=FILTER_SIZE)
filter_right = deque(maxlen=FILTER_SIZE)
filter_front = deque(maxlen=FILTER_SIZE)

def safe_median(values):
    valid = [v for v in values if not math.isinf(v) and not math.isnan(v) and v > WALL_MIN_RANGE]
    if not valid:
        return 999.0
    valid.sort()
    n = len(valid)
    mid = n // 2
    return valid[mid] if n % 2 != 0 else (valid[mid-1] + valid[mid]) / 2.0

def reset_filters():
    filter_left.clear()
    filter_right.clear()
    filter_front.clear()

# ==================== STATE ====================
entry_count = 0
exit_count  = 0

pid_error_prev = 0.0
pid_integral   = 0.0
pid_last_time  = None
steer_prev     = 0.0

def reset_pid():
    global pid_error_prev, pid_integral, pid_last_time, steer_prev
    pid_error_prev = 0.0
    pid_integral   = 0.0
    pid_last_time  = None
    steer_prev     = 0.0

last_left  = 999.0
last_right = 999.0
last_front = 999.0

pub_cmd   = None
pub_debug = None

# ==================== PID ====================
def pid_step(error, kp, kd):
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
    pid_integral  = max(-0.5, min(0.5, pid_integral))

    derivative     = (error - pid_error_prev) / dt
    pid_error_prev = error

    output = kp * error + KI * pid_integral + kd * derivative
    output = max(-MAX_STEER, min(MAX_STEER, output))
    return output

# ==================== CALLBACKS ====================
def status_callback(msg):
    global CARBOT_RUNNING
    data = msg.data
    if 'ST:R' in data:
        CARBOT_RUNNING = True
    elif 'ST:S' in data or 'ST:E' in data:
        CARBOT_RUNNING = False
        stop_carbot()

def scan_callback(msg):
    global last_left, last_right, last_front
    global entry_count, exit_count, TUNNEL_ACTIVE

    left_vals  = []
    right_vals = []
    front_vals = []

    angle_min = msg.angle_min
    angle_inc = msg.angle_increment

    for i, r in enumerate(msg.ranges):
        if math.isinf(r) or math.isnan(r) or r < WALL_MIN_RANGE:
            continue

        angle_deg = math.degrees(angle_min + i * angle_inc)
        while angle_deg > 180:
            angle_deg -= 360
        while angle_deg < -180:
            angle_deg += 360

        if LEFT_ZONE_MIN <= angle_deg <= LEFT_ZONE_MAX and r <= WALL_MAX_RANGE:
            left_vals.append(r)
        elif RIGHT_ZONE_MIN <= angle_deg <= RIGHT_ZONE_MAX and r <= WALL_MAX_RANGE:
            right_vals.append(r)
        elif -15 <= angle_deg <= 15:
            front_vals.append(r)

    if left_vals:
        filter_left.append(safe_median(left_vals))
        last_left = safe_median(list(filter_left))
    else:
        last_left = 999.0

    if right_vals:
        filter_right.append(safe_median(right_vals))
        last_right = safe_median(list(filter_right))
    else:
        last_right = 999.0

    if front_vals:
        filter_front.append(safe_median(front_vals))
        last_front = safe_median(list(filter_front))
    else:
        last_front = 999.0

    # ---- ENTRY LOGIC: dual-wall, width dalam julat ----
    dual_wall = (len(left_vals) > 0 and len(right_vals) > 0)
    tunnel_width = last_left + last_right
    width_valid  = (TUNNEL_MIN_WIDTH <= tunnel_width <= TUNNEL_MAX_WIDTH)

    if dual_wall and width_valid:
        entry_count += 1
    else:
        entry_count = 0

    # ---- EXIT LOGIC: KEDUA dinding > EXIT_RADIUS ----
    both_open = (last_left > EXIT_RADIUS) and (last_right > EXIT_RADIUS)

    if both_open:
        exit_count += 1
    else:
        exit_count = 0

    if not TUNNEL_ACTIVE and entry_count >= ENTRY_HYSTERESIS_FRAMES:
        TUNNEL_ACTIVE = True
        reset_pid()
        rospy.logwarn("[TUNNEL] >>> MASUK TUNNEL (width=%.2fm) <<<" % tunnel_width)

    elif TUNNEL_ACTIVE and exit_count >= EXIT_HYSTERESIS_FRAMES:
        TUNNEL_ACTIVE = False
        reset_pid()
        reset_filters()
        rospy.loginfo("[TUNNEL] >>> KELUAR TUNNEL (kedua dinding >%.2fm) <<<" % EXIT_RADIUS)

    if TUNNEL_ACTIVE and CARBOT_RUNNING:
        navigate_tunnel()

# ==================== NAVIGATION ====================
def navigate_tunnel():
    global pid_integral, steer_prev
    twist = Twist()

    front_blocked = last_front < FRONT_STOP_DIST
    left_seen  = last_left  <= EXIT_RADIUS
    right_seen = last_right <= EXIT_RADIUS

    # ---- STOP: front blocked DAN tiada ruang sisi untuk elak ----
    if front_blocked and last_left < MIN_WALL_DIST and last_right < MIN_WALL_DIST:
        rospy.logerr("[TUNNEL] Tersekat! F:%.2f L:%.2f R:%.2f" % (last_front, last_left, last_right))
        stop_carbot()
        publish_debug("STOP", 0.0, 0.0)
        return

    # ---- MODE 1: DUAL WALL (lurus) - center antara dua dinding ----
    if left_seen and right_seen:
        center_error = last_left - last_right   # positif = terlalu kanan, steer kiri

        wall_bias = 0.0
        mode = "CENTER"
        if last_left < MIN_WALL_DIST:
            wall_bias -= KP_WALL * (MIN_WALL_DIST - last_left)
            mode = "WALL_BIAS_L"
        elif last_right < MIN_WALL_DIST:
            wall_bias += KP_WALL * (MIN_WALL_DIST - last_right)
            mode = "WALL_BIAS_R"

        if abs(center_error) < STEER_DEADZONE:
            pid_integral = 0.0
            pid_steer_val = 0.0
        else:
            pid_steer_val = pid_step(center_error, KP, KD)

        raw_steer = max(-MAX_STEER, min(MAX_STEER, pid_steer_val + wall_bias))

    # ---- MODE 2: SINGLE WALL KIRI SAHAJA (selekoh L-shape, kanan terbuka) ----
    elif left_seen and not right_seen:
        mode = "SELEKOH_L_WALL"
        target_from_left = TUNNEL_WIDTH_ESTIMATE / 2.0
        error = last_left - target_from_left
        # error positif = terlalu jauh dari kiri (perlu dekat balik / steer kiri)
        # error negatif = terlalu dekat kiri (steer kanan)
        raw_steer = pid_step(error, KP_SINGLE, KD_SINGLE)

    # ---- MODE 3: SINGLE WALL KANAN SAHAJA (selekoh sebaliknya) ----
    elif right_seen and not left_seen:
        mode = "SELEKOH_R_WALL"
        target_from_right = TUNNEL_WIDTH_ESTIMATE / 2.0
        error = target_from_right - last_right
        raw_steer = pid_step(error, KP_SINGLE, KD_SINGLE)

    # ---- MODE 4: TIADA DINDING (sepatutnya sudah exit, tapi safety fallback) ----
    else:
        mode = "NO_WALL_FALLBACK"
        raw_steer = steer_prev * 0.5   # decay perlahan-lahan

    steer = LP_ALPHA * raw_steer + (1 - LP_ALPHA) * steer_prev
    steer_prev = steer

    speed = TUNNEL_SPEED * (1.0 - abs(steer) / MAX_STEER)
    speed = max(MIN_SPEED, speed)

    twist.linear.x  = speed
    twist.angular.z = steer * STEER_SIGN
    pub_cmd.publish(twist)

    publish_debug(mode, steer, speed)

def publish_debug(mode, steer, speed):
    if pub_debug is None:
        return
    msg = String()
    msg.data = "mode:%s L:%.3f R:%.3f F:%.3f steer:%+.3f spd:%.2f TUN:%d" % (
        mode, last_left, last_right, last_front, steer, speed, int(TUNNEL_ACTIVE)
    )
    pub_debug.publish(msg)

def stop_carbot():
    reset_pid()
    twist = Twist()
    twist.linear.x  = 0.0
    twist.angular.z = 0.0
    pub_cmd.publish(twist)

# ==================== MAIN ====================
def main():
    global pub_cmd, pub_debug
    rospy.init_node('tunnel_nav')

    pub_cmd   = rospy.Publisher('/cmd_vel',          Twist,  queue_size=1)
    pub_debug = rospy.Publisher('/tunnel_nav/debug', String, queue_size=1)

    rospy.Subscriber('/carbot/status', String,    status_callback)
    rospy.Subscriber('/scan',          LaserScan, scan_callback)

    rospy.loginfo("[TUNNEL] Tunnel Nav ready (LiDAR only, no TUN:1 dependency)")
    rospy.loginfo("[TUNNEL] Entry: dual-wall %.2f-%.2fm | Exit: both walls >%.2fm" %
                  (TUNNEL_MIN_WIDTH, TUNNEL_MAX_WIDTH, EXIT_RADIUS))
    rospy.loginfo("[TUNNEL] Selekoh mode: guna lebar anggaran %.2fm untuk kekal center" %
                  TUNNEL_WIDTH_ESTIMATE)
    rospy.spin()

if __name__ == '__main__':
    main()
