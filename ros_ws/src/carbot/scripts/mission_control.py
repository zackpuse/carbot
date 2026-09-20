#!/usr/bin/env python
# Mission Control - orchestrator utama CarBot NxGV
# Kawal peralihan fasa mengikut 16 langkah mission, guna signage YOLOv5
# sebagai trigger. Publish /mission/phase - node navigasi lain boleh
# subscribe topic ini sebagai lapisan guard tambahan.
#
# Topic /tunnel_nav/active (Bool, latched) dari tunnel_nav.py adalah
# punca kebenaran untuk bila-bila TUNNEL phase selesai - bukan signage.

import rospy
from std_msgs.msg import String, Bool

# ==================== FASA MISSION ====================
PHASE_WAIT_GATE1        = "WAIT_GATE1"          # 1. Tunggu boom gate 1 buka
PHASE_LANE_TO_CHANGE    = "LANE_TO_CHANGE"       # 2-3. Lane follow, tunggu lane change sign
PHASE_LANE_CHANGED      = "LANE_CHANGED"         # 3. Selepas tukar lorong, terus lane follow
PHASE_LANE_TO_ROUND     = "LANE_TO_ROUND"        # 4. Lane follow ke roundabout
PHASE_ROUNDABOUT        = "ROUNDABOUT"           # 5. Dalam roundabout - lane_follow handle sendiri
PHASE_LANE_TO_TUNNEL    = "LANE_TO_TUNNEL"       # 5. Lane follow lepas roundabout, tunggu tunnel sign
PHASE_TUNNEL            = "TUNNEL"               # 6. Dalam tunnel - tunnel_nav ambil alih
PHASE_LANE_TO_GATE2     = "LANE_TO_GATE2"        # 6->7. Lane follow keluar tunnel ke gate 2
PHASE_WAIT_GATE2        = "WAIT_GATE2"           # 7. Tunggu boom gate 2 buka (SENTIASA sebelum hill)
PHASE_HILL              = "HILL"                 # 8. Lane follow + hill climb + speed bump
PHASE_LANE_TO_LIGHT     = "LANE_TO_LIGHT"        # 8->9. Lane follow ke traffic light
PHASE_WAIT_LIGHT        = "WAIT_LIGHT"           # 9. Tunggu lampu hijau
PHASE_LANE_TO_CHANGE2   = "LANE_TO_CHANGE2"      # 11. Lane follow, tunggu lane change sign ke-2
PHASE_LANE_TO_ROUND2    = "LANE_TO_ROUND2"       # 12. Lane follow ke roundabout ke-2
PHASE_ROUNDABOUT2       = "ROUNDABOUT2"          # 12. Roundabout ke-2, exit 2 (exit 1 tertutup)
PHASE_LANE_TO_PARALLEL  = "LANE_TO_PARALLEL"     # 12->13. Lane follow ke parallel parking
PHASE_PARK_PARALLEL     = "PARK_PARALLEL"        # 13. Parking parallel
PHASE_LANE_TO_PERP      = "LANE_TO_PERP"         # 13->14. Lane follow ke perpendicular parking
PHASE_PARK_PERP         = "PARK_PERP"            # 14-15. Parking perpendicular
PHASE_MISSION_DONE      = "MISSION_DONE"         # 16. Selesai

# ==================== STATE ====================
current_phase   = PHASE_WAIT_GATE1
CARBOT_RUNNING  = False
TUNNEL_ACTIVE   = False

# Signage confidence tracking - elak flicker dari satu frame sahaja
detection_history = {}
DETECTION_CONFIRM_FRAMES = 3

pub_phase   = None
pub_nav_cmd = None

# ==================== HELPERS ====================
def set_phase(new_phase):
    global current_phase
    if current_phase != new_phase:
        rospy.loginfo("[MISSION] Fasa: %s -> %s" % (current_phase, new_phase))
        current_phase = new_phase

def publish_phase():
    if pub_phase is not None:
        pub_phase.publish(String(data=current_phase))

def confirm_detection(label):
    global detection_history
    count = detection_history.get(label, 0) + 1
    detection_history[label] = count

    for k in list(detection_history.keys()):
        if k != label:
            detection_history[k] = 0

    return count >= DETECTION_CONFIRM_FRAMES

def reset_detection_history():
    global detection_history
    detection_history = {}

# ==================== CALLBACKS ====================
def status_callback(msg):
    global CARBOT_RUNNING
    data = msg.data
    if 'ST:R' in data and not CARBOT_RUNNING:
        CARBOT_RUNNING = True
        rospy.loginfo("[MISSION] CarBot RUNNING - mission mula")
    elif ('ST:S' in data or 'ST:E' in data) and CARBOT_RUNNING:
        CARBOT_RUNNING = False
        rospy.loginfo("[MISSION] CarBot STOPPED")

def tunnel_active_callback(msg):
    # Punca kebenaran TUNNEL entry/exit ialah tunnel_nav.py sendiri
    # (LiDAR dual-wall + hysteresis), bukan signage YOLOv5.
    global TUNNEL_ACTIVE
    was_active = TUNNEL_ACTIVE
    TUNNEL_ACTIVE = msg.data

    if TUNNEL_ACTIVE and not was_active:
        if current_phase == PHASE_LANE_TO_TUNNEL:
            set_phase(PHASE_TUNNEL)
            reset_detection_history()

    elif not TUNNEL_ACTIVE and was_active:
        if current_phase == PHASE_TUNNEL:
            set_phase(PHASE_LANE_TO_GATE2)
            reset_detection_history()

def yolo_callback(msg):
    # Format: "CAM:front label1:conf1 label2:conf2 ..."
    data = msg.data
    if not CARBOT_RUNNING:
        return

    detected_labels = []
    for part in data.split():
        if ':' in part and part.split(':')[0] != 'CAM':
            label = part.split(':')[0]
            detected_labels.append(label)

    if not detected_labels:
        return

    handle_signage(detected_labels)

def handle_signage(labels):
    global current_phase

    # ---- FASA 1: Tunggu Boom Gate 1 ----
    if current_phase == PHASE_WAIT_GATE1:
        if 'BoomGate_Open' in labels and confirm_detection('BoomGate_Open'):
            set_phase(PHASE_LANE_TO_CHANGE)
            reset_detection_history()

    # ---- FASA 2-3: Lane change pertama ----
    elif current_phase == PHASE_LANE_TO_CHANGE:
        if 'Linechanging_sign' in labels and confirm_detection('Linechanging_sign'):
            set_phase(PHASE_LANE_CHANGED)
            reset_detection_history()

    elif current_phase == PHASE_LANE_CHANGED:
        if 'Roundabout_sign' in labels and confirm_detection('Roundabout_sign'):
            set_phase(PHASE_ROUNDABOUT)
            reset_detection_history()

    # ---- FASA 4-5: Roundabout pertama ----
    elif current_phase == PHASE_LANE_TO_ROUND:
        if 'Roundabout_sign' in labels and confirm_detection('Roundabout_sign'):
            set_phase(PHASE_ROUNDABOUT)
            reset_detection_history()

    elif current_phase == PHASE_ROUNDABOUT:
        # lane_follow.py ikut garisan roundabout secara semula jadi.
        if 'Enter_Tunnel_sign' in labels and confirm_detection('Enter_Tunnel_sign'):
            set_phase(PHASE_LANE_TO_TUNNEL)
            reset_detection_history()

    # ---- FASA 6: Tunnel - peralihan dikendalikan oleh tunnel_active_callback ----

    # ---- FASA 7: Boom Gate 2 - SENTIASA sebelum naik bukit ----
    elif current_phase == PHASE_LANE_TO_GATE2:
        if 'BoomGate_Closed' in labels and confirm_detection('BoomGate_Closed'):
            set_phase(PHASE_WAIT_GATE2)
            reset_detection_history()

    elif current_phase == PHASE_WAIT_GATE2:
        if 'BoomGate_Open' in labels and confirm_detection('BoomGate_Open'):
            set_phase(PHASE_HILL)
            reset_detection_history()

    # ---- FASA 8: Hill + Speed Bump ----
    elif current_phase == PHASE_HILL:
        # lane_follow.py + Arduino BNO055 handle slope speed compensation sendiri
        if 'TrafficLight_sign' in labels and confirm_detection('TrafficLight_sign'):
            set_phase(PHASE_LANE_TO_LIGHT)
            reset_detection_history()

    # ---- FASA 9-10: Traffic Light ----
    elif current_phase == PHASE_LANE_TO_LIGHT:
        if 'red_light' in labels and confirm_detection('red_light'):
            set_phase(PHASE_WAIT_LIGHT)
            reset_detection_history()
        elif 'green_light' in labels and confirm_detection('green_light'):
            set_phase(PHASE_LANE_TO_CHANGE2)
            reset_detection_history()

    elif current_phase == PHASE_WAIT_LIGHT:
        if 'green_light' in labels and confirm_detection('green_light'):
            set_phase(PHASE_LANE_TO_CHANGE2)
            reset_detection_history()

    # ---- FASA 11: Lane Change Kedua ----
    elif current_phase == PHASE_LANE_TO_CHANGE2:
        if 'Linechanging_sign' in labels and confirm_detection('Linechanging_sign'):
            set_phase(PHASE_LANE_TO_ROUND2)
            reset_detection_history()

    # ---- FASA 12: Roundabout Kedua - Exit 2 (exit 1 tertutup barrier) ----
    elif current_phase == PHASE_LANE_TO_ROUND2:
        if 'Roundabout_sign' in labels and confirm_detection('Roundabout_sign'):
            set_phase(PHASE_ROUNDABOUT2)
            reset_detection_history()

    elif current_phase == PHASE_ROUNDABOUT2:
        if 'Parking_sign' in labels and confirm_detection('Parking_sign'):
            set_phase(PHASE_LANE_TO_PARALLEL)
            reset_detection_history()
        elif 'parallel_parking_sign' in labels and confirm_detection('parallel_parking_sign'):
            set_phase(PHASE_LANE_TO_PARALLEL)
            reset_detection_history()

    # ---- FASA 13: Parallel Parking ----
    elif current_phase == PHASE_LANE_TO_PARALLEL:
        if 'parallel_parking_sign' in labels and confirm_detection('parallel_parking_sign'):
            set_phase(PHASE_PARK_PARALLEL)
            reset_detection_history()
            trigger_parking('PARK_PARALLEL')

    # ---- FASA 14: Perpendicular Parking ----
    elif current_phase == PHASE_LANE_TO_PERP:
        if 'perpendicular_parking_sign' in labels and confirm_detection('perpendicular_parking_sign'):
            set_phase(PHASE_PARK_PERP)
            reset_detection_history()
            trigger_parking('PARK_PERPENDICULAR')

def trigger_parking(cmd):
    if pub_nav_cmd is not None:
        pub_nav_cmd.publish(String(data=cmd))
        rospy.loginfo("[MISSION] Trigger parking: %s" % cmd)

def parking_result_callback(msg):
    data = msg.data
    global current_phase

    if 'DONE' not in data:
        return

    if current_phase == PHASE_PARK_PARALLEL:
        rospy.loginfo("[MISSION] Parallel parking selesai - keluar terus")
        set_phase(PHASE_LANE_TO_PERP)
        reset_detection_history()

    elif current_phase == PHASE_PARK_PERP:
        rospy.loginfo("[MISSION] Perpendicular parking selesai - MISSION DONE")
        set_phase(PHASE_MISSION_DONE)
        reset_detection_history()

# ==================== MAIN LOOP ====================
def timer_callback(event):
    publish_phase()

def main():
    global pub_phase, pub_nav_cmd
    rospy.init_node('mission_control')

    pub_phase   = rospy.Publisher('/mission/phase',   String, queue_size=1, latch=True)
    pub_nav_cmd = rospy.Publisher('/carbot/nav_cmd',  String, queue_size=1)

    rospy.Subscriber('/carbot/status',      String, status_callback)
    rospy.Subscriber('/tunnel_nav/active',  Bool,   tunnel_active_callback)
    rospy.Subscriber('/yolo/detections',    String, yolo_callback)
    rospy.Subscriber('/carbot/nav_result',  String, parking_result_callback)

    rospy.Timer(rospy.Duration(0.5), timer_callback)

    rospy.loginfo("[MISSION] Mission Control ready")
    rospy.loginfo("[MISSION] Subscribe /tunnel_nav/active (bukan /tunnel_active)")
    rospy.loginfo("[MISSION] Fasa mula: %s" % current_phase)
    rospy.spin()

if __name__ == '__main__':
    main()
