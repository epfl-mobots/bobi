#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from bobi_msgs.msg import MotorVelocities
from bobi_msgs.srv import *


class LogitechJoy:
    def __init__(self):
        rospy.Subscriber("joy_lurebot", Joy, self.joy_cb)
        self._vel_pub = rospy.Publisher(
            "set_velocities", MotorVelocities, queue_size=1)
        # rospy.wait_for_service('set_high_current')

        self._linear_scaler = rospy.get_param('linear_scaler', 1.0)
        self._angular_scaler = rospy.get_param('angular_scaler', 4.0)
        self._l = 0.0451

        self._mv = None
        self._hc = False

    def joy_cb(self, data):

        if self._hc != data.buttons[0]:
            self._hc = data.buttons[0]
            try:
                hc = rospy.ServiceProxy(
                    'set_high_current', HighCurrent)
                msg = HighCurrentRequest()
                msg.enable = bool(data.buttons[0])
                hc(msg)
            except rospy.ServiceException as e:
                # print("Failed to set high current")
                pass

        mv = MotorVelocities()
        if data.buttons[2]:
            mv.left = 0
            mv.right = 0
            rospy.loginfo("Stopping")
        elif (sum(data.axes) != 0):
            v = data.axes[1] * self._linear_scaler
            w = -data.axes[2] * self._angular_scaler
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
    joy = LogitechJoy()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        joy.spin()
        rate.sleep()


if __name__ == '__main__':
    main()
