#!/usr/bin/env python
import rospy
from bobi_msgs.srv import BacIntervene


class TimedPerturbation:
    def __init__(self):
        self._toggle = False
        self._gatt = rospy.get_param(
            'timed_perturbation/gatt', 0.3)
        self._gali = rospy.get_param(
            'timed_perturbation/gali', 0.3)
        self._mspeed_coeff = rospy.get_param(
            'timed_perturbation/mspeed_coeff', 0.5)
        self._wtime = rospy.get_param(
            'timed_perturbation/wtime', 240)

        rospy.wait_for_service('bac_intervene')

    def spin_once(self):
        try:
            srv = rospy.ServiceProxy('bac_intervene', BacIntervene)
            s = BacIntervene()
            s.gatt = 0.3
            s.gali = 0.3
            s.enable = self._toggle
            if s.enable:
                print("Setting (gatt, gali, mspeed_coeff)".format(
                    self._gatt, self._gali, self._mspeed_coeff))
            else:
                print("Setting defaults")
            self._toggle = not self._toggle
            _ = srv(s)
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))

        if s.enable: 
            return 60
        else:
            return 240


def main():
    rospy.init_node('timed_perturbation_node')
    tp = TimedPerturbation()
    while not rospy.is_shutdown():
        t = tp.spin_once()
        rospy.sleep(t)


if __name__ == '__main__':
    main()
