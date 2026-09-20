#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

pub_filtered  = None
pub_distances = None

def safe_median(values):
    valid = [v for v in values if not math.isinf(v) and not math.isnan(v) and v > 0.05]
    if not valid:
        return 999.0
    valid.sort()
    n = len(valid)
    mid = n // 2
    return valid[mid] if n % 2 != 0 else (valid[mid-1] + valid[mid]) / 2.0

def scan_callback(msg):
    global pub_filtered, pub_distances

    total = len(msg.ranges)
    mid   = total // 2
    span  = int((math.pi / 2) / msg.angle_increment)

    start = mid - span
    end   = mid + span

    filtered = LaserScan()
    filtered.header          = msg.header
    filtered.angle_min       = -math.pi / 2
    filtered.angle_max       =  math.pi / 2
    filtered.angle_increment = msg.angle_increment
    filtered.time_increment  = msg.time_increment
    filtered.scan_time       = msg.scan_time
    filtered.range_min       = msg.range_min
    filtered.range_max       = msg.range_max
    filtered.ranges          = msg.ranges[start:end]
    filtered.intensities     = msg.intensities[start:end]

    pub_filtered.publish(filtered)

    # Kira F L R dengan median
    ranges = list(filtered.ranges)
    n      = len(ranges)
    w      = max(1, n // 10)  # window 10% dari total

    front = safe_median(ranges[n//2 - w : n//2 + w])
    left  = safe_median(ranges[0:w])
    right = safe_median(ranges[n-w:n])

    dist_msg      = String()
    dist_msg.data = "F:%.3f L:%.3f R:%.3f" % (front, left, right)
    pub_distances.publish(dist_msg)

def main():
    global pub_filtered, pub_distances
    rospy.init_node('lidar_filter')
    pub_filtered  = rospy.Publisher('/scan_filtered',   LaserScan, queue_size=1)
    pub_distances = rospy.Publisher('/lidar/distances', String,    queue_size=1)
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.loginfo("[LIDAR FILTER] Ready - /scan_filtered + /lidar/distances")
    rospy.spin()

if __name__ == '__main__':
    main()
