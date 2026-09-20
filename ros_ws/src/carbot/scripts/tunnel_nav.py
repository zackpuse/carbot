#!/usr/bin/env python
# Tunnel Navigation - 100% LiDAR based
# Entry: dual-wall detected, width 50-70cm
# Selekoh (L-shape): satu dinding sahaja - kekal dalam tunnel, guna anggaran
#                     lebar tunnel untuk kekal center
# Exit: KEDUA-DUA dinding tiada dalam radius 40cm

import rospy
import math
from collections import deque
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# ==================== CONFIG ====================
CARBOT_RUNNING = False
TUNNEL_ACTIVE  = False

# Wall classification zones (degrees, 0=depan)
# FIX: zones now share exact boundaries with FRONT so every angle -180..180
# gets classified into exactly one bucket. Previously LEFT/RIGHT started at
# +-20 while FRONT stopped at +-15, leaving a 15-20 deg blind gap on each
# side where a wall reading was silently dropped (not front, not left/right).
LEFT_ZONE_MIN  = 20
LEFT_ZONE_MAX  = 140
RIGHT_ZONE_MIN = -140
RIGHT_ZONE_MAX = -20
FRONT_ZONE_MIN = -20
FRONT_ZONE_MAX = 20

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
KP = 5.0          # dinaikkan: tangkap drift awal sebelum masuk WALL_BIAS zone
KI = 0.03         # dikurangkan: kurang integral windup semasa oscillate
KD = 0.7          # dinaikkan: damping kuat elak overshoot antara dinding

# PID single-wall mode (selekoh) - guna gain berasingan, biasanya lebih agresif
KP_SINGLE = 2.8
KD_SINGLE = 0.3

# Corner turn bias (selekoh/L-shape) - proactively turn toward the open side
# BEFORE hitting the front wall, instead of waiting for center_error/wall
# offset to notice. This fixes "steering go straight" into the front wall
# when the single visible wall reading doesn't change much near a corner.
#
# IMPORTANT (Ackermann + slow servo): the chassis can't pivot in place and
# the steering servo is slow to reach full deflection. Confirmed via field
# test: correct turn direction, but only starts turning ~1cm from the wall
# - by then there's no runway left for a slow servo + fixed turn radius to
# actually change heading in time. FRONT_WARN_DIST must trigger the turn
# with enough distance/time left for the servo to reach full lock AND for
# the car to arc through it. Start high and tune down carefully - too low
# and you're back to late-turn-into-wall; too high and it starts turning
# before it should mid-straight.
FRONT_WARN_DIST  = 0.90   # was 0.55 - too late for a slow Ackermann servo. TUNE ON BENCH
CORNER_TURN_GAIN = 1.2    # higher = sharper corner turn - TUNE ON BENCH

# While front_urgency is high, floor speed even lower than normal MIN_SPEED
# so the car covers less distance per second of servo travel time - more
# effective arc achieved per meter, since a slow servo can't be sped up but
# the car CAN be slowed down while it catches up.
CORNER_MIN_SPEED = 0.08   # dinaikkan sepadan dengan speed keseluruhan - TUNE ON BENCH

# Wall bias (terlalu dekat dinding - safety push)
# BODY_EDGE_OFFSET_*: raw LiDAR range reading when the car's physical body
# edge (not the sensor) is exactly 2cm from the wall - MEASURED via
# calibrate_body_offset.py, one object placed at exactly 0.02m from each
# side of the body. Body is NOT symmetric (LiDAR isn't centered), so each
# direction has its own offset - do not average them into one constant.
#   Left  measured: raw 0.134m @ 2cm -> offset 0.114m
#   Right measured: raw 0.159m @ 2cm -> offset 0.139m
#   Front measured: raw 0.213m @ 2cm -> offset 0.193m
BODY_EDGE_OFFSET_L = 0.114
BODY_EDGE_OFFSET_R = 0.139
BODY_EDGE_OFFSET_F = 0.193

SAFE_CLEARANCE   = 0.12   # desired real clearance (car edge to wall) before steering correction kicks in
CONTACT_MARGIN   = 0.03   # real clearance below which we treat it as "already touching" - hard stop, no more steering
                          # naik dari 0.02 - lagi laju = lagi jauh momentum coast lepas stop, perlukan buffer lagi besar

KP_WALL = 1.8     # dikurangkan drastik: elak kick kuat yg buat overshoot
MIN_WALL_DIST_L = BODY_EDGE_OFFSET_L + SAFE_CLEARANCE   # raw-range trigger for steer-away bias, left side
MIN_WALL_DIST_R = BODY_EDGE_OFFSET_R + SAFE_CLEARANCE   # raw-range trigger for steer-away bias, right side
CONTACT_DIST_L  = BODY_EDGE_OFFSET_L + CONTACT_MARGIN   # raw-range = left side essentially touching - hard stop
CONTACT_DIST_R  = BODY_EDGE_OFFSET_R + CONTACT_MARGIN   # raw-range = right side essentially touching - hard stop

# Steering
MAX_STEER      = 1.0
STEER_DEADZONE = 0.015  # dikurangkan: tangkap drift 1.5cm, bukan 5cm
STEER_SIGN     = 1.0   # tukar ke -1.0 kalau arah terbalik

# Front obstacle / stop - recalibrated using BODY_EDGE_OFFSET_F the same way
FRONT_SAFE_CLEARANCE = 0.20   # real clearance in front before treating it as "blocked" (was flat 0.40 raw)
FRONT_STOP_DIST = BODY_EDGE_OFFSET_F + FRONT_SAFE_CLEARANCE   # raw-range threshold, ~0.393m

# Speed
TUNNEL_SPEED = 0.20    # dinaikkan dari 0.15 - TUNE ON BENCH, verify hard-stop masih cukup awal pada speed ni
MIN_SPEED    = 0.12    # dinaikkan dari 0.09 sepadan dengan TUNNEL_SPEED - TUNE ON BENCH

# Filter
FILTER_SIZE = 1   # diturunkan: hapus sensor lag (~300ms -> ~100ms)
LP_ALPHA    = 0.70  # diturunkan: steer filter lebih responsive

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
current_mode   = None   # track mode semasa untuk reset PID bila tukar mod

def reset_pid():
    global pid_error_prev, pid_integral, pid_last_time, steer_prev, current_mode
    pid_error_prev = 0.0
    pid_integral   = 0.0
    pid_last_time  = None
    steer_prev     = 0.0
    current_mode   = None

last_left  = 999.0
last_right = 999.0
last_front = 999.0

pub_cmd    = None
pub_debug  = None
pub_active = None   # Bool: True bila tunnel aktif, supaya lane_follow boleh yield

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

    # Reset integral bila error cross zero (elak windup lawan arah baru)
    if pid_error_prev * error < 0:
        pid_integral = 0.0

    pid_integral += error * dt
    pid_integral  = max(-0.3, min(0.3, pid_integral))  # had lebih ketat

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

        # FIX: zones are contiguous now (LEFT/RIGHT start exactly where FRONT
        # ends), so no angle range is silently skipped. A wall appearing in
        # the old 15-20 deg gap used to vanish from all three buckets.
        if LEFT_ZONE_MIN <= angle_deg <= LEFT_ZONE_MAX and r <= WALL_MAX_RANGE:
            left_vals.append(r)
        elif RIGHT_ZONE_MIN <= angle_deg <= RIGHT_ZONE_MAX and r <= WALL_MAX_RANGE:
            right_vals.append(r)
        elif FRONT_ZONE_MIN <= angle_deg <= FRONT_ZONE_MAX:
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
        if pub_active is not None:
            pub_active.publish(Bool(data=True))
        rospy.logwarn("[TUNNEL] >>> MASUK TUNNEL (width=%.2fm) <<<" % tunnel_width)

    elif TUNNEL_ACTIVE and exit_count >= EXIT_HYSTERESIS_FRAMES:
        TUNNEL_ACTIVE = False
        reset_pid()
        reset_filters()
        if pub_active is not None:
            pub_active.publish(Bool(data=False))
        rospy.loginfo("[TUNNEL] >>> KELUAR TUNNEL (kedua dinding >%.2fm) <<<" % EXIT_RADIUS)

    if TUNNEL_ACTIVE and CARBOT_RUNNING:
        navigate_tunnel()

# ==================== NAVIGATION ====================
def navigate_tunnel():
    global pid_integral, steer_prev, current_mode
    twist = Twist()

    front_blocked = last_front < FRONT_STOP_DIST
    left_seen  = last_left  <= EXIT_RADIUS
    right_seen = last_right <= EXIT_RADIUS

    # ---- HARD STOP: either side already at/past contact - independent of
    # the other side and independent of front. This is the fix for the
    # actual crash: previously STOP only fired when BOTH sides were close,
    # so a left-side contact with an open right side (exactly this corner
    # scenario) never stopped the car - it kept computing steering PID
    # against a wall it was already touching. No amount of steering undoes
    # contact that has already happened; the only correct action here is
    # stop immediately. ----
    if last_left <= CONTACT_DIST_L or last_right <= CONTACT_DIST_R:
        side = "L" if last_left <= CONTACT_DIST_L else "R"
        rospy.logerr("[TUNNEL] KONTAK dinding %s! L:%.2f R:%.2f F:%.2f - STOP" %
                     (side, last_left, last_right, last_front))
        stop_carbot()
        publish_debug("CONTACT_STOP", 0.0, 0.0)
        return

    # ---- STOP: front blocked DAN tiada ruang sisi untuk elak ----
    if front_blocked and last_left < MIN_WALL_DIST_L and last_right < MIN_WALL_DIST_R:
        rospy.logerr("[TUNNEL] Tersekat! F:%.2f L:%.2f R:%.2f" % (last_front, last_left, last_right))
        stop_carbot()
        publish_debug("STOP", 0.0, 0.0)
        return

    # ---- MODE 1: DUAL WALL (lurus) - center antara dua dinding ----
    if left_seen and right_seen:
        center_error = last_left - last_right   # positif = terlalu kanan, steer kiri

        wall_bias = 0.0
        mode = "CENTER"
        if last_left < MIN_WALL_DIST_L:
            wall_bias -= KP_WALL * (MIN_WALL_DIST_L - last_left)
            mode = "WALL_BIAS_L"
        elif last_right < MIN_WALL_DIST_R:
            wall_bias += KP_WALL * (MIN_WALL_DIST_R - last_right)
            mode = "WALL_BIAS_R"

        # Reset PID bila bertukar dari mode lain (elak derivative spike)
        if current_mode not in ("CENTER", "WALL_BIAS_L", "WALL_BIAS_R"):
            reset_pid()

        if abs(center_error) < STEER_DEADZONE:
            pid_integral = 0.0
            pid_steer_val = 0.0
        else:
            pid_steer_val = pid_step(center_error, KP, KD)

        raw_steer = max(-MAX_STEER, min(MAX_STEER, pid_steer_val + wall_bias))

    # ---- MODE 2: SINGLE WALL KIRI SAHAJA (selekoh L-shape, kanan terbuka) ----
    elif left_seen and not right_seen:
        mode = "SELEKOH_L_WALL"
        # Reset PID bila bertukar dari mode lain (elak derivative spike)
        if current_mode != mode:
            reset_pid()
        target_from_left = TUNNEL_WIDTH_ESTIMATE / 2.0
        error = last_left - target_from_left
        # error positif = terlalu jauh dari kiri (perlu dekat balik / steer kiri)
        # error negatif = terlalu dekat kiri (steer kanan)
        if abs(error) < STEER_DEADZONE:
            pid_integral = 0.0
            raw_steer = 0.0
        else:
            raw_steer = pid_step(error, KP_SINGLE, KD_SINGLE)

    # ---- MODE 3: SINGLE WALL KANAN SAHAJA (selekoh sebaliknya) ----
    elif right_seen and not left_seen:
        mode = "SELEKOH_R_WALL"
        # Reset PID bila bertukar dari mode lain (elak derivative spike)
        if current_mode != mode:
            reset_pid()
        target_from_right = TUNNEL_WIDTH_ESTIMATE / 2.0
        error = target_from_right - last_right
        if abs(error) < STEER_DEADZONE:
            pid_integral = 0.0
            raw_steer = 0.0
        else:
            raw_steer = pid_step(error, KP_SINGLE, KD_SINGLE)

    # ---- MODE 4: TIADA DINDING (sepatutnya sudah exit, tapi safety fallback) ----
    else:
        mode = "NO_WALL_FALLBACK"
        raw_steer = steer_prev * 0.5   # decay perlahan-lahan

    # ---- CORNER TURN BIAS ----
    # FIX: previously, selekoh mode only reacted to the single-wall offset,
    # which can stay near-zero right up to an L-shape corner (the corner
    # wall is ahead-left/ahead-right, not beside the robot), so raw_steer
    # stayed ~0 and the robot drove straight into the front wall. This adds
    # a proactive turn toward the OPEN side as the front distance shrinks,
    # independent of the single-wall offset error above.
    front_urgency = 0.0
    if last_front < FRONT_WARN_DIST and mode in ("SELEKOH_L_WALL", "SELEKOH_R_WALL"):
        front_urgency = (FRONT_WARN_DIST - last_front) / FRONT_WARN_DIST  # 0..1
        if mode == "SELEKOH_L_WALL":
            # wall on left, open on right -> turn right
            raw_steer -= CORNER_TURN_GAIN * front_urgency
        else:
            # wall on right, open on left -> turn left
            raw_steer += CORNER_TURN_GAIN * front_urgency
        raw_steer = max(-MAX_STEER, min(MAX_STEER, raw_steer))

    current_mode = mode   # simpan mode semasa untuk detect pertukaran mod seterusnya

    # Slow Ackermann servo: the LP_ALPHA smoothing below adds its own lag on
    # top of the physical servo lag. When front_urgency is high, ramp the
    # effective alpha up toward 1.0 (raw_steer passes through almost
    # unfiltered) so the full turn command reaches the servo immediately
    # instead of easing in over several frames.
    effective_alpha = max(LP_ALPHA, front_urgency)
    steer = effective_alpha * raw_steer + (1 - effective_alpha) * steer_prev
    steer_prev = steer

    speed = TUNNEL_SPEED * (1.0 - abs(steer) / MAX_STEER)
    speed_floor = CORNER_MIN_SPEED if front_urgency > 0.0 else MIN_SPEED
    speed = max(speed_floor, speed)

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
    global pub_cmd, pub_debug, pub_active
    rospy.init_node('tunnel_nav')

    pub_cmd    = rospy.Publisher('/cmd_vel',             Twist,  queue_size=1)
    pub_debug  = rospy.Publisher('/tunnel_nav/debug',    String, queue_size=1)
    pub_active = rospy.Publisher('/tunnel_nav/active',   Bool,   queue_size=1, latch=True)

    rospy.Subscriber('/carbot/status', String,    status_callback)
    rospy.Subscriber('/scan',          LaserScan, scan_callback)

    rospy.loginfo("[TUNNEL] Tunnel Nav ready (LiDAR only, no TUN:1 dependency)")
    rospy.loginfo("[TUNNEL] Entry: dual-wall %.2f-%.2fm | Exit: both walls >%.2fm" %
                  (TUNNEL_MIN_WIDTH, TUNNEL_MAX_WIDTH, EXIT_RADIUS))
    rospy.loginfo("[TUNNEL] Selekoh mode: guna lebar anggaran %.2fm untuk kekal center" %
                  TUNNEL_WIDTH_ESTIMATE)
    rospy.loginfo("[TUNNEL] Offset L:%.3f R:%.3f F:%.3f | Steer bias L<%.2f R<%.2f | Hard stop L<=%.2f R<=%.2f | Front stop <%.2f" %
                  (BODY_EDGE_OFFSET_L, BODY_EDGE_OFFSET_R, BODY_EDGE_OFFSET_F,
                   MIN_WALL_DIST_L, MIN_WALL_DIST_R, CONTACT_DIST_L, CONTACT_DIST_R, FRONT_STOP_DIST))
    rospy.loginfo("[TUNNEL] Corner turn bias aktif bila front < %.2fm (corner min speed %.2f)" %
                  (FRONT_WARN_DIST, CORNER_MIN_SPEED))
    rospy.loginfo("[TUNNEL] Publish /tunnel_nav/active (Bool) untuk coordinate dengan node lain")
    rospy.spin()

if __name__ == '__main__':
    main()
