#!/usr/bin/env python
# Steering test tool - untuk kalibrasi servo dan test cmd_vel steering
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

pub_cmd = None
current_status = ""

def status_callback(msg):
    global current_status
    current_status = msg.data

def send_steer(angular_z, linear_x=0.0):
    twist = Twist()
    twist.linear.x  = linear_x
    twist.angular.z = angular_z
    pub_cmd.publish(twist)

def print_menu():
    print("")
    print("=" * 50)
    print("  CARBOT STEERING TEST TOOL")
    print("=" * 50)
    print("  a = Steer kiri penuh   (angular.z = +1.0)")
    print("  d = Steer kanan penuh  (angular.z = -1.0)")
    print("  s = Steer tengah       (angular.z =  0.0)")
    print("  1-9 = Steer sebahagian (0.1 - 0.9)")
    print("  w = Maju perlahan + steer semasa")
    print("  x = Undur perlahan + steer semasa")
    print("  0 = Stop (motor + steer tengah)")
    print("  q = Keluar")
    print("=" * 50)
    print("  Status: %s" % current_status)
    print("")

def main():
    global pub_cmd
    rospy.init_node('steering_test', anonymous=True)

    pub_cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/carbot/status', String, status_callback)

    rospy.sleep(0.5)  # tunggu publisher ready

    current_steer = 0.0

    print_menu()

    while not rospy.is_shutdown():
        try:
            cmd = raw_input("Masukkan arahan: ").strip().lower()
        except NameError:
            cmd = input("Masukkan arahan: ").strip().lower()

        if cmd == 'q':
            send_steer(0.0, 0.0)
            print("Keluar - motor stop, steer tengah")
            break

        elif cmd == 'a':
            current_steer = 1.0
            send_steer(current_steer, 0.0)
            print("Steer KIRI penuh (+1.0)")

        elif cmd == 'd':
            current_steer = -1.0
            send_steer(current_steer, 0.0)
            print("Steer KANAN penuh (-1.0)")

        elif cmd == 's':
            current_steer = 0.0
            send_steer(current_steer, 0.0)
            print("Steer TENGAH (0.0)")

        elif cmd == '0':
            current_steer = 0.0
            send_steer(0.0, 0.0)
            print("STOP - motor + steer tengah")

        elif cmd == 'w':
            send_steer(current_steer, 0.4)
            print("MAJU perlahan, steer=%.1f" % current_steer)

        elif cmd == 'x':
            send_steer(current_steer, -0.2)
            print("UNDUR perlahan, steer=%.1f" % current_steer)

        elif cmd in '123456789':
            val = int(cmd) / 10.0
            current_steer = val
            send_steer(current_steer, 0.0)
            print("Steer KIRI %.1f" % current_steer)

        elif cmd.startswith('-') and cmd[1:] in '123456789':
            val = -int(cmd[1:]) / 10.0
            current_steer = val
            send_steer(current_steer, 0.0)
            print("Steer KANAN %.1f" % current_steer)

        else:
            print("Arahan tidak sah")
            print_menu()

if __name__ == '__main__':
    main()
