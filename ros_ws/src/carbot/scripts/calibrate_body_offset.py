#!/usr/bin/env python
# Calibrate BODY_EDGE_OFFSET (tunnel_nav.py) menggunakan jarak sebenar.
#
# CARA GUNA:
#   1. Letak objek rata (kayu/board/kotak) tepat pada jarak SEBENAR dari
#      hujung fizikal badan kereta (bukan dari LiDAR) - contoh 0.02m (2cm).
#      Ukur guna pembaris dari titik badan kereta yang paling menonjol
#      (bumper/fender), bukan dari casing LiDAR.
#   2. Run: rosrun <pkg> calibrate_body_offset.py  (atau python3 terus kalau
#      dah source workspace) sambil roscore + LiDAR driver dah jalan.
#   3. Tengok nilai "L raw" / "R raw" / "F raw" bila dah stabil (std kecil).
#   4. BODY_EDGE_OFFSET = raw_value_yang_stabil - jarak_sebenar_yang_diukur
#      Contoh: kalau objek diletak tepat 0.02m dari body, dan L raw stabil
#      bacaan 0.19m -> BODY_EDGE_OFFSET = 0.19 - 0.02 = 0.17
#   5. Ulang untuk kanan (guna R) dan depan (guna F) - offset boleh berbeza
#      ikut arah sebab bentuk fizikal body tak simetri (mounting LiDAR tak
#      di tengah-tengah).
#   6. Buat 2-3 jarak berbeza (contoh 0.02m, 0.05m, 0.10m) untuk pastikan
#      offset konsisten merentasi jarak - kalau offset berubah-ubah ikut
#      jarak, kemungkinan ada sudut/pemasangan sensor yang tak straight,
#      bukan sekadar offset linear mudah.

import rospy
import math
from collections import deque
from sensor_msgs.msg import LaserScan

# ---- Zon sama macam tunnel_nav.py - kekalkan selaras ----
LEFT_ZONE_MIN, LEFT_ZONE_MAX   = 20, 140
RIGHT_ZONE_MIN, RIGHT_ZONE_MAX = -140, -20
FRONT_ZONE_MIN, FRONT_ZONE_MAX = -20, 20
WALL_MAX_RANGE = 0.70
WALL_MIN_RANGE = 0.04

SAMPLE_WINDOW = 20   # bilangan scan untuk kira purata/std bergerak

hist_left  = deque(maxlen=SAMPLE_WINDOW)
hist_right = deque(maxlen=SAMPLE_WINDOW)
hist_front = deque(maxlen=SAMPLE_WINDOW)

def safe_median(values):
    valid = [v for v in values if not math.isinf(v) and not math.isnan(v) and v > WALL_MIN_RANGE]
    if not valid:
        return None
    valid.sort()
    n = len(valid)
    mid = n // 2
    return valid[mid] if n % 2 != 0 else (valid[mid-1] + valid[mid]) / 2.0

def mean_std(values):
    vals = [v for v in values if v is not None]
    if not vals:
        return None, None
    m = sum(vals) / len(vals)
    var = sum((v - m) ** 2 for v in vals) / len(vals)
    return m, math.sqrt(var)

def scan_callback(msg):
    left_vals, right_vals, front_vals = [], [], []
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
        elif FRONT_ZONE_MIN <= angle_deg <= FRONT_ZONE_MAX:
            front_vals.append(r)

    l = safe_median(left_vals)
    r = safe_median(right_vals)
    f = safe_median(front_vals)

    if l is not None:
        hist_left.append(l)
    if r is not None:
        hist_right.append(r)
    if f is not None:
        hist_front.append(f)

    l_mean, l_std = mean_std(list(hist_left))
    r_mean, r_std = mean_std(list(hist_right))
    f_mean, f_std = mean_std(list(hist_front))

    def fmt(mean, std):
        if mean is None:
            return "no data"
        return "%.3fm (std %.3f)" % (mean, std)

    rospy.loginfo("L raw: %-22s R raw: %-22s F raw: %-22s" %
                  (fmt(l_mean, l_std), fmt(r_mean, r_std), fmt(f_mean, f_std)))
    rospy.loginfo_throttle(10,
        "[CALIB] Bila stabil (std kecil), BODY_EDGE_OFFSET = raw - jarak_sebenar_diukur")

def main():
    rospy.init_node('calibrate_body_offset')
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.loginfo("[CALIB] Ready. Letak objek pada jarak sebenar dari body, tunggu std kecil.")
    rospy.loginfo("[CALIB] Zon: LEFT %d-%d deg | RIGHT %d-%d deg | FRONT %d-%d deg" %
                  (LEFT_ZONE_MIN, LEFT_ZONE_MAX, RIGHT_ZONE_MIN, RIGHT_ZONE_MAX,
                   FRONT_ZONE_MIN, FRONT_ZONE_MAX))
    rospy.spin()

if __name__ == '__main__':
    main()
