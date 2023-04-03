#!/usr/bin/env python3
import rospy
from thymiodirect import Connection
from thymiodirect import Thymio
from bobi_msgs.msg import MotorVelocities, ProximitySensors
import numpy as np


class ThymioInterface:
    def __init__(self):
        # tdmclient definitions
        self._port = Connection.serial_default_port()
        self._th = Thymio(
            serial_port=self._port,
            on_connect=lambda node_id: print(f"{node_id} is connected"))
        self._th.connect()
        self._id = self._th.first_node()

        self._lb_new = -500.
        self._ub_new = 500.
        self._lb_old = -0.2
        self._ub_old = 0.2
        self._wheel_rad = 0.022
        self._wheel_dist_2 = 0.0475

        # ROS subs/pubs
        self._motor_vel_sub = rospy.Subscriber(
            'set_velocities', MotorVelocities, self._motor_vel_cb)
        self._prox_pub = rospy.Publisher(
            'proximity_sensors', ProximitySensors, queue_size=1)

        self.stop()

    def scan(self):
        ps = ProximitySensors()
        ps.header.stamp = rospy.Time.now()
        ps.values = self._th[self._id]["prox.horizontal"]
        self._prox_pub.publish(ps)

    def __del__(self):
        self.stop()

    def _motor_vel_cb(self, data):
        left = self._rescale(data.left)
        right = self._rescale(data.right)
        self.set_velocities(left, right)

    def set_velocities(self, left, right):
        # print("(l, r): ({}, {})".format(left, right))
        left = left if left >= 0 else 2 ** 16 + left
        right = right if right >= 0 else 2 ** 16 + right
        self._th[self._id]["motor.left.target"] = left
        self._th[self._id]["motor.right.target"] = right

    def stop(self):
        self.set_velocities(0, 0)

    def _rescale(self, val):
        if val > self._ub_old:
            val = self._ub_old
        elif val < self._lb_old:
            val = self._lb_old
        nrange = self._ub_new - self._lb_new
        orange = self._ub_old - self._lb_old
        return int((((val - self._lb_old) * nrange) / orange) + self._lb_new)


def main():
    rospy.init_node('thymio_interface_node')
    ti = ThymioInterface()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ti.scan()
        rate.sleep()


if __name__ == '__main__':
    main()
