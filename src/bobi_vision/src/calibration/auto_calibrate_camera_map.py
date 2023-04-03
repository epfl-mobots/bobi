#!/usr/bin/env python3

import rospy
import yaml
import numpy as np
from bobi_msgs.msg import PoseVec
from bobi_msgs.msg import PoseStamped


class CameraMapper:
    def __init__(self):
        self._start_p = rospy.get_param('start_point', [0.55, 0.20])
        self._end_p = rospy.get_param('end_point', [1.20, 0.90])
        print(self._end_p)
        print(self._start_p)
        self._num_samples = rospy.get_param('num_samples', 100)
        self._distance_eps = rospy.get_param('distance_eps', 0.005)
        self._waypoints = []
        self._waypoints_top = []
        self._waypoints_bottom = []
        self._idx = 0
        self._done = False

        self._top_pose = None
        self._bottom_pose = None

        # rospy.Subscriber("filtered_poses", PoseVec, self._top_poses_cb)
        rospy.Subscriber("naive_poses", PoseVec, self._top_poses_cb)
        rospy.Subscriber("robot_poses", PoseVec, self._bottom_poses_cb)
        self._goal_pub = rospy.Publisher(
            'target_position', PoseStamped, queue_size=1)

    def _top_poses_cb(self, data):
        if (len(data.poses) > 0):
            self._top_pose = data.poses[0]

    def _bottom_poses_cb(self, data):
        if (len(data.poses) > 0):
            self._bottom_pose = data.poses[0]

    def compute_grid(self):
        samples_per_dim = int(np.sqrt(self._num_samples))
        points_x = np.linspace(
            self._start_p[0], self._end_p[0], samples_per_dim).tolist()
        points_y = np.linspace(
            self._start_p[1], self._end_p[1], samples_per_dim).tolist()

        direction = 1
        self._waypoints = []
        for i in range(samples_per_dim):
            self._waypoints += zip([points_x[i]] *
                                   samples_per_dim, points_y[::direction])
            direction *= -1
        self._idx = 0

    def try_next(self):
        if len(self._waypoints) == len(self._waypoints_top):
            self._done = True
            return

        if not((self._top_pose is None) and (self._bottom_pose is None)):
            if self._idx < len(self._waypoints):
                goal = self._waypoints[self._idx]
                if np.sqrt(
                    (self._bottom_pose.pose.xyz.x - goal[0]) ** 2 +
                    (self._bottom_pose.pose.xyz.y - goal[1]) ** 2
                ) < self._distance_eps:
                    self._waypoints_bottom.append(
                        (self._bottom_pose.pose.xyz.x,
                         self._bottom_pose.pose.xyz.y)
                    )
                    self._waypoints_top.append(
                        (self._top_pose.pose.xyz.x,
                         self._top_pose.pose.xyz.y))
                    self._idx += 1
                else:
                    ps = PoseStamped()
                    ps.header.stamp = rospy.Time.now()
                    ps.pose.xyz.x = goal[0]
                    ps.pose.xyz.y = goal[1]
                    self._goal_pub.publish(ps)

    def set_start_point(self, p):
        self._start_p = p

    def set_end_point(self, p):
        self._end_p = p

    def set_num_samples(self, num_samples):
        self._num_samples = num_samples

    def done(self):
        return self._done

    def dump_yaml(self):
        wp = [list(a) for a in self._waypoints]
        wp_top = [list(a) for a in self._waypoints_top]
        wp_bottom = [list(a) for a in self._waypoints_bottom]
        mapping_dict = {
            'points_top': wp_top,
            'points_bottom': wp_bottom,
            'waypoints': wp,
        }
        with open('camera_mapping.yaml', 'w') as f:
            yaml.dump(mapping_dict, f)


def main():
    cm = CameraMapper()
    cm.compute_grid()
    rospy.init_node('autocalib_map_node', anonymous=True)

    loop_rate = rospy.Rate(10)
    while not rospy.is_shutdown() and not cm.done():
        cm.try_next()
        loop_rate.sleep()

    print('Auto-calibration done')
    cm.dump_yaml()


if __name__ == '__main__':
    main()
