#!/usr/bin/env python
# Mission Control - orchestrator utama CarBot NxGV
# Kawal peralihan fasa mengikut 16 langkah mission, guna signage YOLOv5
# sebagai trigger. Publish /mission/phase - node navigasi lain subscribe
# topic ini sebagai lapisan guard tambahan.
#
# Topic /tunnel_nav/active (Bool, latched) dari tunnel_nav.py adalah
# punca kebenaran untuk bila-bila TUNNEL phase selesai - bukan signage.
#
# Tambahan: watchdog timeout setiap fasa (elak freeze selama-lamanya bila
# signage gagal dikesan) + tindakan eksplisit stop bila MISSION_DONE.

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

# ==================== FASA MISSION ====================
PHASE_WAIT_GATE1        = "WAIT_GATE1"
PHASE_LANE_TO_CHANGE    = "LANE_TO_CHANGE"
PHASE_LANE_CHANGED      = "LANE_CHANGED"
PHASE_LANE_TO_ROUND     = "LANE_TO_ROUND"
PHASE_ROUNDABOUT        = "ROUNDABOUT"
PHASE_LANE_TO_TUNNEL    = "LANE_TO_TUNNEL"
PHASE_TUNNEL            = "TUNNEL"
PHASE_LANE_TO_GATE2     = "LANE_TO_GATE2"
PHASE_WAIT_GATE2        = "WAIT_GATE2"
PHASE_HILL              = "HILL"
PHASE_LANE_TO_LIGHT     = "LANE_TO_LIGHT"
PHASE_WAIT_LIGHT        = "WAIT_LIGHT"
PHASE_LANE_TO_CHANGE2   = "LANE_TO_CHANGE2"
PHASE_LANE_TO_ROUND2    = "LANE_TO_ROUND2"
PHASE_ROUNDABOUT2       = "ROUNDABOUT2"
PHASE_LANE_TO_PARALLEL  = "LANE_TO_PARALLEL"
PHASE_PARK_PARALLEL     = "PARK_PARALLEL"
PHASE_LANE_TO_PERP      = "LANE_TO_PERP"
PHASE_PARK_PERP         = "PARK_PERP"
PHASE_MISSION_DONE      = "MISSION_DONE"

# ==================== WATCHDOG CONFIG ====================
# Timeout (saat) untuk setiap fasa yang menunggu signage tertentu.
# Kalau timeout tercapai tanpa signage dikesan, log amaran dan reset
# detection history supaya cuba semula (tidak tukar fasa secara paksa -
# CarBot lebih baik terus cuba detect drpd tersasar ke fasa salah).
PHASE_TIMEOUT_SEC = 15.0

# Fasa yang tidak perlu watchdog (state transisi automatik/instant,
# atau menunggu peristiwa luaran seperti tunnel_nav sendiri)
NO_WATCHDOG_PHASES = set([PHASE_TUNNEL, PHASE_PARK_PARALLEL, PHASE_PARK_PERP, PHASE_MISSION_DONE])

# ==================== STATE ====================
current_phase    = PHASE_WAIT_GATE1
CARBOT_RUNNING   = False
TUNNEL_ACTIVE    = False
phase_enter_time = None

detection_history = {}
DETECTION_CONFIRM_FRAMES = 3

pub_phase   = None
pub_nav_cmd = None
pub_cmd     = None

# ==================== HELPERS ====================
def set_phase(new_phase):
    global current_phase, phase_enter_time
    if current_phase != new_phase:
        rospy.loginfo("[MISSION] Fasa: %s -> %s" % (current_phase, new_phase))
        current_phase    = new_phase
        phase_enter_time = rospy.Time.now().to_sec()

        if new_phase == PHASE_MISSION_DONE:
            finish_mission()

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

def finish_mission():
    # Tindakan eksplisit bila MISSION_DONE - pastikan CarBot benar-benar
    # berhenti, bukan sekadar tukar label fasa. Hantar stop beberapa kali
    # (latch tidak menjamin subscriber terima serta-merta pada frame pertama).
    rospy.logwarn("[MISSION] ================================")
    rospy.logwarn("[MISSION]      MISSION SELESAI - BERHENTI")
    rospy.logwarn("[MISSION] ================================")
    if pub_cmd is not None:
        stop_msg = Twist()
        for _ in range(5):
            pub_cmd.publish(stop_msg)
            rospy.sleep(0.05)

# ==================== WATCHDOG ====================
def check_watchdog():
    if not CARBOT_RUNNING:
        return
    if current_phase in NO_WATCHDOG_PHASES:
        return
    if phase_enter_time is None:
        return

    elapsed = rospy.Time.now().to_sec() - phase_enter_time
    if elapsed > PHASE_TIMEOUT_SEC:
        rospy.logwarn("[MISSION] WATCHDOG: fasa %s belum berubah selepas %.0fs - "
                       "signage mungkin gagal dikesan, cuba semula deteksi" %
                       (current_phase, elapsed))
        reset_detection_history()
        # Reset masa supaya amaran tidak berulang setiap frame - beri
        # tempoh baru sebelum amaran seterusnya
        globals()['phase_enter_time'] = rospy.Time.now().to_sec()

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

    if current_phase == PHASE_WAIT_GATE1:
        if 'BoomGate_Open' in labels and confirm_detection('BoomGate_Open'):
            set_phase(PHASE_LANE_TO_CHANGE)
            reset_detection_history()

    elif current_phase == PHASE_LANE_TO_CHANGE:
        if 'Linechanging_sign' in labels and confirm_detection('Linechanging_sign'):
            set_phase(PHASE_LANE_CHANGED)
            reset_detection_history()

    elif current_phase == PHASE_LANE_CHANGED:
        if 'Roundabout_sign' in labels and confirm_detection('Roundabout_sign'):
            set_phase(PHASE_ROUNDABOUT)
            reset_detection_history()

    elif current_phase == PHASE_LANE_TO_ROUND:
        if 'Roundabout_sign' in labels and confirm_detection('Roundabout_sign'):
            set_phase(PHASE_ROUNDABOUT)
            reset_detection_history()

    elif current_phase == PHASE_ROUNDABOUT:
        if 'Enter_Tunnel_sign' in labels and confirm_detection('Enter_Tunnel_sign'):
            set_phase(PHASE_LANE_TO_TUNNEL)
            reset_detection_history()

    # FASA TUNNEL: peralihan dikendalikan oleh tunnel_active_callback

    elif current_phase == PHASE_LANE_TO_GATE2:
        if 'BoomGate_Closed' in labels and confirm_detection('BoomGate_Closed'):
            set_phase(PHASE_WAIT_GATE2)
            reset_detection_history()

    elif current_phase == PHASE_WAIT_GATE2:
        if 'BoomGate_Open' in labels and confirm_detection('BoomGate_Open'):
            set_phase(PHASE_HILL)
            reset_detection_history()

    elif current_phase == PHASE_HILL:
        if 'TrafficLight_sign' in labels and confirm_detection('TrafficLight_sign'):
            set_phase(PHASE_LANE_TO_LIGHT)
            reset_detection_history()

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

    elif current_phase == PHASE_LANE_TO_CHANGE2:
        if 'Linechanging_sign' in labels and confirm_detection('Linechanging_sign'):
            set_phase(PHASE_LANE_TO_ROUND2)
            reset_detection_history()

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

    elif current_phase == PHASE_LANE_TO_PARALLEL:
        if 'parallel_parking_sign' in labels and confirm_detection('parallel_parking_sign'):
            set_phase(PHASE_PARK_PARALLEL)
            reset_detection_history()
            trigger_parking('PARK_PARALLEL')

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
    check_watchdog()

def main():
    global pub_phase, pub_nav_cmd, pub_cmd
    rospy.init_node('mission_control')

    pub_phase   = rospy.Publisher('/mission/phase',   String, queue_size=1, latch=True)
    pub_nav_cmd = rospy.Publisher('/carbot/nav_cmd',  String, queue_size=1)
    pub_cmd     = rospy.Publisher('/cmd_vel',         Twist,  queue_size=1)

    rospy.Subscriber('/carbot/status',      String, status_callback)
    rospy.Subscriber('/tunnel_nav/active',  Bool,   tunnel_active_callback)
    rospy.Subscriber('/yolo/detections',    String, yolo_callback)
    rospy.Subscriber('/carbot/nav_result',  String, parking_result_callback)

    rospy.Timer(rospy.Duration(0.5), timer_callback)

    rospy.loginfo("[MISSION] Mission Control ready")
    rospy.loginfo("[MISSION] Watchdog timeout: %.0fs per fasa" % PHASE_TIMEOUT_SEC)
    rospy.loginfo("[MISSION] Fasa mula: %s" % current_phase)
    rospy.spin()

if __name__ == '__main__':
    main()
