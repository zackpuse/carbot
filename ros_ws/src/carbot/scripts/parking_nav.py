#!/usr/bin/env python
import rospy
import json
import os
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from carbot.cfg import ParkingNavConfig

# ==================== CONFIG ====================
CONFIG_FILE = "/ssd/config/parking_config.json"

PARALLEL_REVERSE_DIST  = 35.0
PARALLEL_FORWARD_DIST  = 15.0
PARALLEL_STEER_ANGLE   = 0.8

PERP_FORWARD_DIST      = 10.0
PERP_REVERSE_DIST      = 30.0
PERP_STEER_ANGLE       = 0.0

PARKING_SPEED          = 0.15
PARKING_REVERSE_SPEED  = -0.15

# ==================== CONFIG CALLBACK ====================
srv = None

def load_config_from_file():
    global PARALLEL_REVERSE_DIST, PARALLEL_FORWARD_DIST, PARALLEL_STEER_ANGLE
    global PERP_FORWARD_DIST, PERP_REVERSE_DIST, PERP_STEER_ANGLE
    global PARKING_SPEED, PARKING_REVERSE_SPEED
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                saved = json.load(f)
            if 'parallel_reverse_dist' in saved: PARALLEL_REVERSE_DIST = saved['parallel_reverse_dist']
            if 'parallel_forward_dist' in saved: PARALLEL_FORWARD_DIST = saved['parallel_forward_dist']
            if 'parallel_steer_angle' in saved: PARALLEL_STEER_ANGLE = saved['parallel_steer_angle']
            if 'perp_forward_dist' in saved: PERP_FORWARD_DIST = saved['perp_forward_dist']
            if 'perp_reverse_dist' in saved: PERP_REVERSE_DIST = saved['perp_reverse_dist']
            if 'perp_steer_angle' in saved: PERP_STEER_ANGLE = saved['perp_steer_angle']
            if 'parking_speed' in saved: PARKING_SPEED = saved['parking_speed']
            if 'parking_reverse_speed' in saved: PARKING_REVERSE_SPEED = saved['parking_reverse_speed']
            rospy.loginfo("[PARK] Config dimuat dari %s" % CONFIG_FILE)
            return True
        except Exception as e:
            rospy.logwarn("[PARK] Gagal load config fail: %s" % str(e))
    return False

def save_config_to_file(config):
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        save_data = {k: v for k, v in config.items() if k != 'save_config' and k != 'groups'}
        with open(CONFIG_FILE, 'w') as f:
            json.dump(save_data, f, indent=2)
        rospy.loginfo("[PARK] Config disimpan ke %s" % CONFIG_FILE)
    except Exception as e:
        rospy.logwarn("[PARK] Gagal simpan config: %s" % str(e))

def config_callback(config, level):
    global PARALLEL_REVERSE_DIST, PARALLEL_FORWARD_DIST, PARALLEL_STEER_ANGLE
    global PERP_FORWARD_DIST, PERP_REVERSE_DIST, PERP_STEER_ANGLE
    global PARKING_SPEED, PARKING_REVERSE_SPEED

    PARALLEL_REVERSE_DIST = config.parallel_reverse_dist
    PARALLEL_FORWARD_DIST = config.parallel_forward_dist
    PARALLEL_STEER_ANGLE  = config.parallel_steer_angle

    PERP_FORWARD_DIST = config.perp_forward_dist
    PERP_REVERSE_DIST = config.perp_reverse_dist
    PERP_STEER_ANGLE  = config.perp_steer_angle

    PARKING_SPEED         = config.parking_speed
    PARKING_REVERSE_SPEED = config.parking_reverse_speed

    if config.save_config:
        save_config_to_file(config)
        config.save_config = False

    return config

# ==================== STATE ====================
CARBOT_RUNNING  = False
current_dist    = 0.0
current_us      = 999
current_steer   = 90
parking_active  = False   # HANYA True bila explicit trigger via /carbot/nav_cmd
parking_type    = None

pub_cmd    = None
pub_ctrl   = None
pub_result = None

park_state = 'IDLE'
next_park_state = None
state_dist = 0.0

# ==================== PARSE STATUS ====================
def parse_status(data):
    global current_dist, current_us, current_steer, CARBOT_RUNNING
    result = {}
    try:
        for part in data.split():
            if ':' in part:
                k, v = part.split(':', 1)
                result[k] = v
        if 'ST' in result:
            CARBOT_RUNNING = (result['ST'] == 'R')
        if 'D' in result:
            current_dist = float(result['D'])
        if 'US' in result:
            current_us = float(result['US'])
        if 'STR' in result:
            current_steer = int(result['STR'])
    except Exception as e:
        rospy.logwarn("[PARK] Parse error: %s" % str(e))

# ==================== HELPERS ====================
def reset_distance():
    msg = String()
    msg.data = "RESET_DIST"
    pub_ctrl.publish(msg)
    rospy.loginfo("[PARK] Distance reset")

def publish_cmd(speed, steer):
    # GUARD - jangan publish langsung kalau parking_nav tidak aktif
    if not parking_active:
        return
    twist = Twist()
    twist.linear.x  = speed
    twist.angular.z = steer
    pub_cmd.publish(twist)

def stop():
    # GUARD - jangan publish langsung kalau parking_nav tidak aktif
    if not parking_active:
        return
    twist = Twist()
    twist.linear.x  = 0.0
    twist.angular.z = 0.0
    pub_cmd.publish(twist)

def dist_since_state():
    return abs(current_dist - state_dist)

# ==================== PARALLEL PARKING ====================
def parallel_parking_step():
    global park_state, state_dist, next_park_state

    if park_state == 'WAIT_RESET':
        if current_dist < 5.0:
            rospy.loginfo("[PARK] Reset disahkan - masuk %s" % next_park_state)
            state_dist = current_dist
            park_state = next_park_state

    elif park_state == 'POSITION':
        if dist_since_state() >= 20.0:
            rospy.loginfo("[PARK] Position done - reverse steer")
            reset_distance()
            next_park_state = 'REVERSE_STEER'
            park_state = 'WAIT_RESET'
            stop()
            rospy.sleep(0.5)
        else:
            publish_cmd(PARKING_SPEED, 0.0)

    elif park_state == 'REVERSE_STEER':
        if dist_since_state() >= PARALLEL_REVERSE_DIST * 0.6:
            rospy.loginfo("[PARK] Reverse steer done - reverse straight")
            reset_distance()
            next_park_state = 'REVERSE_STRAIGHT'
            park_state = 'WAIT_RESET'
            stop()
            rospy.sleep(0.3)
        else:
            publish_cmd(PARKING_REVERSE_SPEED, -PARALLEL_STEER_ANGLE)

    elif park_state == 'REVERSE_STRAIGHT':
        if dist_since_state() >= PARALLEL_REVERSE_DIST * 0.4:
            rospy.loginfo("[PARK] Reverse straight done - adjust")
            reset_distance()
            next_park_state = 'ADJUST'
            park_state = 'WAIT_RESET'
            stop()
            rospy.sleep(0.3)
        else:
            publish_cmd(PARKING_REVERSE_SPEED, PARALLEL_STEER_ANGLE)

    elif park_state == 'ADJUST':
        if dist_since_state() >= PARALLEL_FORWARD_DIST:
            rospy.loginfo("[PARK] Parallel parking DONE")
            park_state = 'DONE'
            stop()
            finish_parking()
        else:
            publish_cmd(PARKING_SPEED, PARALLEL_STEER_ANGLE)

    elif park_state == 'DONE':
        stop()

# ==================== PERPENDICULAR PARKING ====================
def perpendicular_parking_step():
    global park_state, state_dist, next_park_state

    if park_state == 'WAIT_RESET':
        if current_dist < 5.0:
            rospy.loginfo("[PARK] Reset disahkan - masuk %s" % next_park_state)
            state_dist = current_dist
            park_state = next_park_state

    elif park_state == 'POSITION':
        if dist_since_state() >= PERP_FORWARD_DIST:
            rospy.loginfo("[PARK] Aligned - reversing")
            reset_distance()
            next_park_state = 'REVERSE'
            park_state = 'WAIT_RESET'
            stop()
            rospy.sleep(0.5)
        else:
            publish_cmd(PARKING_SPEED, 0.0)

    elif park_state == 'REVERSE':
        if dist_since_state() >= PERP_REVERSE_DIST:
            rospy.loginfo("[PARK] Perpendicular parking DONE")
            park_state = 'DONE'
            stop()
            finish_parking()
        else:
            publish_cmd(PARKING_REVERSE_SPEED, PERP_STEER_ANGLE)

    elif park_state == 'DONE':
        stop()

def finish_parking():
    # Selepas parking selesai, matikan parking_active supaya
    # lane_follow / tunnel_nav boleh ambil alih /cmd_vel semula
    global parking_active
    rospy.loginfo("[PARK] Parking selesai - melepaskan kawalan /cmd_vel")
    parking_active = False
    
    if pub_result is not None:
        pub_result.publish(String(data="DONE"))

# ==================== CALLBACKS ====================
def status_callback(msg):
    parse_status(msg.data)

    # GUARD UTAMA - terus return kalau parking tidak aktif
    # Ini elak parking_nav publish apa-apa ke /cmd_vel semasa idle
    if not parking_active:
        return

    if not CARBOT_RUNNING:
        return

    if parking_type == 'parallel':
        parallel_parking_step()
    elif parking_type == 'perpendicular':
        perpendicular_parking_step()

def nav_cmd_callback(msg):
    global parking_active, parking_type, park_state, state_dist, next_park_state

    cmd = msg.data.strip()

    if cmd == 'PARK_PARALLEL':
        parking_active = True
        parking_type   = 'parallel'
        reset_distance()
        next_park_state = 'POSITION'
        park_state = 'WAIT_RESET'
        rospy.loginfo("[PARK] START parallel parking - mengambil kawalan /cmd_vel")

    elif cmd == 'PARK_PERPENDICULAR':
        parking_active = True
        parking_type   = 'perpendicular'
        reset_distance()
        next_park_state = 'POSITION'
        park_state = 'WAIT_RESET'
        rospy.loginfo("[PARK] START perpendicular parking - mengambil kawalan /cmd_vel")

    elif cmd == 'PARK_CANCEL':
        rospy.loginfo("[PARK] Parking cancelled - melepaskan kawalan /cmd_vel")
        if parking_active:
            stop()   # hantar stop sekali sebelum lepas kawalan
        parking_active = False
        park_state     = 'IDLE'

    elif cmd == 'PARK_EXIT':
        rospy.loginfo("[PARK] Exiting parking spot")
        parking_active = False
        current_us   = 999.0
        park_state     = 'IDLE'
        reset_distance()

# ==================== MAIN ====================
def main():
    global pub_cmd, pub_ctrl, pub_result, srv
    rospy.init_node('parking_nav')

    has_saved = load_config_from_file()

    srv = Server(ParkingNavConfig, config_callback)

    if has_saved:
        srv.update_configuration({
            'parallel_reverse_dist': PARALLEL_REVERSE_DIST,
            'parallel_forward_dist': PARALLEL_FORWARD_DIST,
            'parallel_steer_angle': PARALLEL_STEER_ANGLE,
            'perp_forward_dist': PERP_FORWARD_DIST,
            'perp_reverse_dist': PERP_REVERSE_DIST,
            'perp_steer_angle': PERP_STEER_ANGLE,
            'parking_speed': PARKING_SPEED,
            'parking_reverse_speed': PARKING_REVERSE_SPEED
        })

    pub_cmd    = rospy.Publisher('/cmd_vel',         Twist,  queue_size=1)
    pub_ctrl   = rospy.Publisher('/carbot/cmd',      String, queue_size=1)
    pub_result = rospy.Publisher('/carbot/nav_result', String, queue_size=1)

    rospy.Subscriber('/carbot/status',  String, status_callback)
    rospy.Subscriber('/carbot/nav_cmd', String, nav_cmd_callback)

    rospy.loginfo("[PARK] Parking navigation ready (IDLE - tidak publish /cmd_vel)")
    rospy.loginfo("[PARK] Commands: PARK_PARALLEL / PARK_PERPENDICULAR / PARK_CANCEL / PARK_EXIT")
    rospy.loginfo("[PARK] Publish ke /carbot/nav_cmd untuk trigger")
    rospy.spin()

if __name__ == '__main__':
    main()
