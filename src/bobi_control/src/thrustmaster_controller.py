#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from bobi_msgs.msg import MotorVelocities
from bobi_msgs.srv import EnableIR, EnableTemp


class ThrustmasterJoy:
    def __init__(self):
        rospy.Subscriber("joy_lurebot", Joy, self.joy_cb)
        self._vel_pub = rospy.Publisher(
            "set_velocities", MotorVelocities, queue_size=1)

        self._linear_scaler = rospy.get_param('linear_scaler', 1.0)
        self._angular_scaler = rospy.get_param('angular_scaler', 4.0)
        self._l = 0.0451
        self._mv = None

    def joy_cb(self, data):
        mv = MotorVelocities()
        if (sum(data.buttons) > 0):

            if data.buttons[1]:
                mv.left = 0
                mv.right = 0
                self._vel_pub.publish(mv)
                rospy.loginfo("Stopping")

        elif (sum(data.axes) != 0):
            v = data.axes[1] * self._linear_scaler
            w = data.axes[2] * self._angular_scaler
            mv.left = (2. * v - w * self._l) / 2.
            mv.right = (2. * v + w * self._l) / 2.
            rospy.loginfo(
                "Received (v, w) = (%f, %f) | Sending (vl, vr) = (%f, %f)", v, w, mv.left, mv.right)
        else:
            mv.left = 0.
            mv.right = 0.

        self._mv = mv

    def spin(self):
        if self._mv is not None:
            self._vel_pub.publish(self._mv)


def main():
    rospy.init_node('joy_controller')
    joy = ThrustmasterJoy()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        joy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
