#!/usr/bin/env python
# Motor Test Tool - test terus via /cmd_vel
# Guna untuk pastikan motor + steering respond tanpa lane_follow/parking_nav
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

pub_cmd = None
current_status = ""

def status_callback(msg):
    global current_status
    current_status = msg.data

def send_cmd(linear_x, angular_z):
    twist = Twist()
    twist.linear.x  = linear_x
    twist.angular.z = angular_z
    pub_cmd.publish(twist)

def print_menu():
    print("")
    print("=" * 55)
    print("  CARBOT MOTOR TEST TOOL (/cmd_vel)")
    print("=" * 55)
    print("  w = Maju perlahan   (linear.x=0.2, angular.z=0.0)")
    print("  x = Undur perlahan  (linear.x=-0.2, angular.z=0.0)")
    print("  a = Maju + kiri     (linear.x=0.2, angular.z=0.5)")
    print("  d = Maju + kanan    (linear.x=0.2, angular.z=-0.5)")
    print("  s = Stop            (linear.x=0.0, angular.z=0.0)")
    print("  1-9 = Maju ikut speed (0.1 - 0.9)")
    print("  p = Publish command custom (linear,angular)")
    print("  i = Papar status Arduino semasa")
    print("  q = Keluar (auto stop)")
    print("=" * 55)
    print("  Status Arduino: %s" % current_status)
    print("")

def main():
    global pub_cmd
    rospy.init_node('motor_test', anonymous=True)

    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/carbot/status', String, status_callback)

    rospy.sleep(0.5)

    print_menu()

    if not current_status or 'ST:R' not in current_status:
        print("!! AMARAN: Butang START mungkin belum ditekan pada Mega.")
        print("!! Motor tidak akan bergerak jika state bukan RUNNING (ST:R).")
        print("")

    while not rospy.is_shutdown():
        try:
            cmd = raw_input("Masukkan arahan: ").strip().lower()
        except NameError:
            cmd = input("Masukkan arahan: ").strip().lower()

        if cmd == 'q':
            send_cmd(0.0, 0.0)
            print("Keluar - motor stop")
            break

        elif cmd == 'w':
            send_cmd(0.2, 0.0)
            print("MAJU perlahan")

        elif cmd == 'x':
            send_cmd(-0.2, 0.0)
            print("UNDUR perlahan")

        elif cmd == 'a':
            send_cmd(0.2, 0.5)
            print("MAJU + KIRI")

        elif cmd == 'd':
            send_cmd(0.2, -0.5)
            print("MAJU + KANAN")

        elif cmd == 's':
            send_cmd(0.0, 0.0)
            print("STOP")

        elif cmd in '123456789':
            speed = int(cmd) / 10.0
            send_cmd(speed, 0.0)
            print("MAJU speed=%.1f" % speed)

        elif cmd == 'p':
            try:
                lin_str = raw_input("  linear.x  = ")
                ang_str = raw_input("  angular.z = ")
            except NameError:
                lin_str = input("  linear.x  = ")
                ang_str = input("  angular.z = ")
            try:
                lin = float(lin_str)
                ang = float(ang_str)
                send_cmd(lin, ang)
                print("Publish linear.x=%.2f angular.z=%.2f" % (lin, ang))
            except ValueError:
                print("Nilai tidak sah")

        elif cmd == 'i':
            print("Status Arduino: %s" % current_status)

        else:
            print("Arahan tidak sah")
            print_menu()

if __name__ == '__main__':
    main()
