
#!/usr/bin/env python
# ============================================================
# CarBot Steering Calibration
# Test: STRAIGHT / FULL LEFT / FULL RIGHT
# Based on tunnel_nav.py steering command interface
# ============================================================

import rospy
from geometry_msgs.msg import Twist

# ==================== CONFIG ====================

CMD_TOPIC = "/cmd_vel"

# Sama seperti MAX_STEER dalam kod asal
MAX_STEER = 1.0

# Sama seperti STEER_SIGN dalam kod asal
STEER_SIGN = 1.0

# ==================== STEERING ====================

pub_cmd = None


def set_steering(value):
    """
    value:
       0.0  = Straight
      +1.0  = Full steering command positive
      -1.0  = Full steering command negative
    """

    twist = Twist()

    # Pastikan motor tidak bergerak
    twist.linear.x = 0.0

    # Steering command
    twist.angular.z = value * STEER_SIGN

    pub_cmd.publish(twist)

    rospy.loginfo(
        "Steering command: angular.z = %+.3f" %
        twist.angular.z
    )


def main():

    global pub_cmd

    rospy.init_node("steering_calibrate")

    pub_cmd = rospy.Publisher(
        CMD_TOPIC,
        Twist,
        queue_size=1
    )

    rospy.sleep(1.0)

    rospy.loginfo("================================")
    rospy.loginfo(" CARBOT STEERING CALIBRATION")
    rospy.loginfo("================================")
    rospy.loginfo("1 = STRAIGHT")
    rospy.loginfo("2 = FULL LEFT")
    rospy.loginfo("3 = FULL RIGHT")
    rospy.loginfo("0 = STOP / STRAIGHT")
    rospy.loginfo("q = EXIT")
    rospy.loginfo("================================")

    try:

        while not rospy.is_shutdown():

            choice = input(
                "\nSelect steering [1/2/3/0/q]: "
            ).strip().lower()

            if choice == "1" or choice == "0":

                set_steering(0.0)
                print("STEERING: STRAIGHT")

            elif choice == "2":

                set_steering(+MAX_STEER)
                print("STEERING: FULL LEFT COMMAND")

            elif choice == "3":

                set_steering(-MAX_STEER)
                print("STEERING: FULL RIGHT COMMAND")

            elif choice == "q":

                set_steering(0.0)
                print("Exit calibration")
                break

            else:

                print("Invalid input. Select 1, 2, 3, 0 or q")

    except (KeyboardInterrupt, EOFError):

        pass

    finally:

        if pub_cmd is not None:

            set_steering(0.0)

        rospy.loginfo("Steering calibration ended")


if __name__ == "__main__":
    main()
