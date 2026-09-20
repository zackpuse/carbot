#!/usr/bin/env python
# Tunnel Steering Test Tool - motor tidak bergerak, steering sahaja
# Guna trend detection untuk curve - elak overshoot arah salah
import rospy
import math
from collections import deque
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ==================== CONFIG ====================
CARBOT_RUNNING = False
TUNNEL_ACTIVE  = False

LEFT_ZONE_MIN  = 15
LEFT_ZONE_MAX  = 120
RIGHT_ZONE_MIN = -120
RIGHT_ZONE_MAX = -15

WALL_MAX_RANGE = 0.60
WALL_MIN_RANGE = 0.05

HYSTERESIS_FRAMES = 3

KP = 1.2
KI = 0.01
KD = 0.8
KP_WALL = 6.0

MAX_STEER      = 1.0
STEER_DEADZONE = 0.02

MIN_WALL_DIST   = 0.20
FRONT_STOP_DIST = 0.40

FILTER_SIZE = 5
LP_ALPHA        = 0.3   # untuk center steering (responsive)
LP_ALPHA_CURVE  = 0.15  # untuk curve steering (lebih smooth, elak overshoot)

# Trend detection - berapa frame untuk kira arah trend
TREND_WINDOW = 5

MOTOR_ENABLED = False

# ==================== FILTER ====================
filter_left  = deque(maxlen=FILTER_SIZE)
filter_right = deque(maxlen=FILTER_SIZE)
filter_front = deque(maxlen=FILTER_SIZE)

# Trend history - untuk detect arah gap mengecil/membesar
left_history  = deque(maxlen=TREND_WINDOW)
right_history = deque(maxlen=TREND_WINDOW)

def median_filter(dq, new_val):
    if math.isinf(new_val) or math.isnan(new_val):
        return None
    if new_val < 0.05 or new_val > 2.0:
        return None
    dq.append(new_val)
    if len(dq) == 0:
        return None
    sorted_vals = sorted(dq)
    n = len(sorted_vals)
    mid = n // 2
    if n % 2 == 0:
        return (sorted_vals[mid-1] + sorted_vals[mid]) / 2.0
    return float(sorted_vals[mid])

def reset_filters():
    filter_left.clear()
    filter_right.clear()
    filter_front.clear()
    left_history.clear()
    right_history.clear()

def get_trend(history):
    # Positive = jarak MENGECIL (dinding menghampiri, ruang berkurang)
    # Negative = jarak MEMBESAR (ruang bertambah)
    if len(history) < 3:
        return 0.0
    return history[0] - history[-1]

# ==================== STATE ====================
dual_wall_count    = 0
missing_wall_count = 0

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

def pid_center(error):
    global pid_error_prev, pid_integral, pid_last_time
    now = rospy.Time.now().to_sec()
    if pid_last_time is None:
        pid_last_time = now
        dt = 0.02
    else:
        dt = now - pid_last_time
        if dt <= 0.001:
            dt = 0.02
        pid_last_time = now

    pid_integral += error * dt
    pid_integral  = max(-0.5, min(0.5, pid_integral))
    derivative     = (error - pid_error_prev) / dt
    pid_error_prev = error

    output = KP * error + KI * pid_integral + KD * derivative
    output = max(-MAX_STEER, min(MAX_STEER, output))
    return output

last_left  = 999.0
last_right = 999.0
last_front = 999.0

pub_cmd = None
frame_count = 0

def status_callback(msg):
    global CARBOT_RUNNING
    data = msg.data
    if 'ST:R' in data:
        CARBOT_RUNNING = True
    elif 'ST:S' in data or 'ST:E' in data:
        CARBOT_RUNNING = False

def scan_callback(msg):
    global last_left, last_right, last_front
    global dual_wall_count, missing_wall_count, TUNNEL_ACTIVE
    global frame_count

    n = len(msg.ranges)
    if n == 0:
        return

    angle_min = msg.angle_min
    angle_inc = msg.angle_increment

    left_vals  = []
    right_vals = []
    front_vals = []

    for i, r in enumerate(msg.ranges):
        if math.isinf(r) or math.isnan(r) or r < WALL_MIN_RANGE:
            continue

        angle_rad = angle_min + i * angle_inc
        angle_deg = math.degrees(angle_rad)

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
        v = median_filter(filter_left, min(left_vals))
        if v is not None:
            last_left = v
    else:
        last_left = 999.0

    if right_vals:
        v = median_filter(filter_right, min(right_vals))
        if v is not None:
            last_right = v
    else:
        last_right = 999.0

    if front_vals:
        v = median_filter(filter_front, min(front_vals))
        if v is not None:
            last_front = v
    else:
        last_front = 999.0

    # Update trend history - guna nilai fizikal (bukan clamp)
    left_history.append(last_left)
    right_history.append(last_right)

    dual_wall_detected = (len(left_vals) > 0 and len(right_vals) > 0)

    if dual_wall_detected:
        dual_wall_count    += 1
        missing_wall_count  = 0
    else:
        missing_wall_count  += 1
        dual_wall_count      = 0

    if not TUNNEL_ACTIVE and dual_wall_count >= HYSTERESIS_FRAMES:
        TUNNEL_ACTIVE = True
        reset_pid()
        rospy.loginfo("[TEST] ENTER TUNNEL")
    elif TUNNEL_ACTIVE and missing_wall_count >= HYSTERESIS_FRAMES:
        TUNNEL_ACTIVE = False
        reset_pid()
        reset_filters()
        rospy.loginfo("[TEST] EXIT TUNNEL")

    compute_and_log_steer(len(left_vals), len(right_vals))

def compute_and_log_steer(n_left, n_right):
    global pid_integral, steer_prev, frame_count
    frame_count += 1

    front_blocked = last_front < FRONT_STOP_DIST
    left_blocked  = last_left  < MIN_WALL_DIST
    right_blocked = last_right < MIN_WALL_DIST

    mode = "CENTER"
    steer = 0.0
    left_trend  = 0.0
    right_trend = 0.0

    if front_blocked and left_blocked and right_blocked:
        mode  = "STOP"
        steer = 0.0
        steer_prev = 0.0

    elif front_blocked:
        mode = "CURVE"

        left_trend  = get_trend(left_history)   # positif = kiri mengecil
        right_trend = get_trend(right_history)  # positif = kanan mengecil

        # Steer ke arah yang GAP-nya membesar (trend negatif / kurang mengecil)
        # right_trend > left_trend bermakna kanan mengecil LEBIH LAJU dari kiri
        # -> ruang kanan semakin sempit -> avoid kanan -> steer KIRI
        if right_trend > left_trend:
            target_steer = MAX_STEER    # kanan makin sempit - steer KIRI (avoid)
            mode = "CURVE_L"
        elif left_trend > right_trend:
            target_steer = -MAX_STEER   # kiri makin sempit - steer KANAN (avoid)
            mode = "CURVE_R"
        else:
            # Trend sama / tidak cukup data - guna snapshot semasa sebagai fallback
            if last_right > last_left:
                target_steer = -MAX_STEER
                mode = "CURVE_R_FB"
            else:
                target_steer = MAX_STEER
                mode = "CURVE_L_FB"

        # Blend perlahan dengan steer_prev - elak overshoot/salah arah snap
        steer = LP_ALPHA_CURVE * target_steer + (1 - LP_ALPHA_CURVE) * steer_prev
        steer_prev = steer

    else:
        left  = last_left  if last_left  < 2.0 else 0.60
        right = last_right if last_right < 2.0 else 0.60
        center_error = left - right

        wall_bias = 0.0
        if left < MIN_WALL_DIST * 2:
            wall_bias = -KP_WALL * (MIN_WALL_DIST * 2 - left)
            mode = "WALL_BIAS_L"
        elif right < MIN_WALL_DIST * 2:
            wall_bias = KP_WALL * (MIN_WALL_DIST * 2 - right)
            mode = "WALL_BIAS_R"

        if abs(center_error) < STEER_DEADZONE:
            pid_steer    = 0.0
            pid_integral = 0.0
        else:
            pid_steer = pid_center(center_error)

        raw_steer = pid_steer + wall_bias
        raw_steer = max(-MAX_STEER, min(MAX_STEER, raw_steer))
        steer = LP_ALPHA * raw_steer + (1 - LP_ALPHA) * steer_prev
        steer_prev = steer

    if frame_count % 5 == 0:
        rospy.loginfo(
            "[STEER] mode=%-12s TUN=%s n_L=%d n_R=%d | L:%.3f R:%.3f F:%.3f | Ltr:%+.3f Rtr:%+.3f | steer:%+.3f" %
            (mode, str(TUNNEL_ACTIVE), n_left, n_right, last_left, last_right, last_front,
             left_trend, right_trend, steer)
        )

    test_twist = Twist()
    test_twist.linear.x  = 0.0
    test_twist.angular.z = steer
    pub_cmd.publish(test_twist)

def main():
    global pub_cmd
    rospy.init_node('tunnel_steer_test')

    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rospy.Subscriber('/carbot/status', String,    status_callback)
    rospy.Subscriber('/scan',          LaserScan, scan_callback)

    rospy.loginfo("[TEST] Tunnel Steering Test v2 - Trend Detection")
    rospy.loginfo("[TEST] MOTOR STATIONARY - servo test sahaja")
    rospy.loginfo("[TEST] Tekan START button pada Mega dulu")
    rospy.spin()

if __name__ == '__main__':
    main()
