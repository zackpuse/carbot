#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan

# Tetapan zon (Sama seperti tunnel_nav_dashboard.py)
LEFT_MIN = 15
LEFT_MAX = 120
RIGHT_MIN = -120
RIGHT_MAX = -15
FRONT_MIN = -15
FRONT_MAX = 15

def safe_median(values):
    valid = [v for v in values if not math.isinf(v) and not math.isnan(v) and v > 0.05]
    if not valid:
        return 999.0
    valid.sort()
    n = len(valid)
    mid = n // 2
    return valid[mid] if n % 2 != 0 else (valid[mid-1] + valid[mid]) / 2.0

def scan_callback(msg):
    left_vals = []
    right_vals = []
    front_vals = []

    for i, r in enumerate(msg.ranges):
        if math.isinf(r) or math.isnan(r) or r < 0.05:
            continue
        
        # Baca sudut sebenar (TANPA INVERT)
        angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)

        # Hanya ambil bacaan dinding yang tak terlalu jauh (contoh max 1.5 meter)
        if r <= 1.5:
            if LEFT_MIN <= angle_deg <= LEFT_MAX:
                left_vals.append(r)
            elif RIGHT_MIN <= angle_deg <= RIGHT_MAX:
                right_vals.append(r)
            if FRONT_MIN <= angle_deg <= FRONT_MAX:
                front_vals.append(r)

    # Dapatkan nilai median (purata paling stabil)
    dist_L = safe_median(left_vals)
    dist_R = safe_median(right_vals)
    dist_F = safe_median(front_vals)

    # Format output (hanya print maksimum 2.0 meter untuk bacaan yang kosong)
    L_str = "{:.2f}m".format(dist_L) if dist_L < 999 else "KOSONG"
    R_str = "{:.2f}m".format(dist_R) if dist_R < 999 else "KOSONG"
    F_str = "{:.2f}m".format(dist_F) if dist_F < 999 else "KOSONG"

    rospy.loginfo("KIRI: %-8s | DEPAN: %-8s | KANAN: %-8s", L_str, F_str, R_str)

def main():
    rospy.init_node('lidar_test_distances')
    # Throttled subscribe supaya tak spam terminal teruk sangat (print 2 kali sesaat je)
    # Tapi rospy takde throttle terbina dalam subscriber biasa, jadi kita print semua.
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    
    rospy.loginfo("========================================")
    rospy.loginfo(" MULA MEMBACA JARAK LIDAR SEBENAR CARBOT ")
    rospy.loginfo("========================================")
    
    rospy.spin()

if __name__ == '__main__':
    main()
