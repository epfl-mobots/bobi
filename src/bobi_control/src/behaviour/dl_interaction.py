#!/usr/bin/env python
import rospy
from bobi_msgs.msg import PoseVec, PoseStamped, MotorVelocities, DLISpecs, TargetPose
from bobi_msgs.srv import ConvertCoordinates
from geometry_msgs.msg import Point

import os
import numpy as np
from random import shuffle

import tensorflow as tf
import tensorflow.keras.backend as K


class CircularCorridor:
    def __init__(self, radius=1.0, center=(0, 0)):
        self._center = center
        self._radius = radius

    def radius(self, position):
        return np.sqrt((position[0] - self._center[0]) ** 2 + (position[1] - self._center[1]) ** 2)

    def is_valid(self, radius):
        return radius < self._radius and radius > 0

    def center(self):
        return self._center


setup = CircularCorridor()


def gaussian_nll(y_true, y_pred):
    """
    :brief: Gaussian negative log likelihood loss function for probabilistic network outputs.

    :param y_true: np.array of the values the network needs to predict
    :param y_pred: np.array of the values the network predicted
    :return: float
    """

    n_dims = int(int(y_pred.shape[1]) / 2)
    mu = y_pred[:, :n_dims]
    logsigma = logbound(y_pred[:, n_dims:], 0, -10, backend='keras')

    mse = -0.5 * K.sum(K.square((y_true-mu) / K.exp(logsigma)), axis=1)
    sigma_trace = -K.sum(logsigma, axis=1)
    log2pi = -0.5 * n_dims * np.log(2 * np.pi)
    log_likelihood = mse + sigma_trace + log2pi
    return K.mean(-log_likelihood)


def multi_dim_gaussian_nll(y_true, y_pred):
    """
    :brief: Gaussian negative log likelihood loss function for probabilistic network outputs.

    :param y_true: np.array of the values the network needs to predict
    :param y_pred: np.array of the values the network predicted
    :return: float
    """

    means = []
    prediction_steps = y_pred.shape[2]
    for i in range(prediction_steps):
        n_dims = y_pred.shape[3] // 2
        mu = y_pred[:, 0, i, :n_dims]
        logsigma = logbound(y_pred[:, 0, i, n_dims:],
                            0.5, -10, backend='keras')

        # https://www.cs.cmu.edu/~epxing/Class/10701-08s/recitation/gaussian.pdf
        f = -0.5 * \
            K.sum(K.square((y_true[:, 0, i, :] - mu) /
                           (K.exp(logsigma) + 1e-8)), axis=1)
        sigma_trace = -K.sum(logsigma, axis=1)
        log2pi = -0.5 * n_dims * np.log(2 * np.pi)
        log_likelihood = f + sigma_trace + log2pi
        means.append(K.mean(-log_likelihood))

    return sum(means) / len(means)


def gaussian_mae(y_true, y_pred):
    """
    :brief: Custom mean absolute error function for the Gaussian negative log likelihood function.

    :param y_true: np.array of the values the network needs to predict
    :param y_pred: np.array of the values the network predicted
    :return: float
    """

    n_dims = y_pred.shape[1] // 2
    return K.mean(K.abs(y_pred[:, :n_dims] - y_true), axis=-1)


def gaussian_mse(y_true, y_pred):
    """
    :brief: Custom mean squared error function for the Gaussian negative log likelihood function.

    :param y_true: np.array of the values the network needs to
    :param y_pred: np.array of the values the network predicted
    :return: float
    """

    n_dims = y_pred.shape[1] // 2
    return K.mean(K.square(y_pred[:, :n_dims] - y_true), axis=-1)


losses = {
    'gaussian_nll': gaussian_nll,
    'gaussian_mse': gaussian_mse,
    'gaussian_mae': gaussian_mae,
    'multi_dim_gaussian_nll': multi_dim_gaussian_nll,
}


def gaussian(x):
    return K.exp(-K.pow(x, 2))


activations = {
    'gaussian': gaussian,
}


def logbound(val, max_logvar=0.5, min_logvar=-10, backend=None):
    if backend is None:
        logsigma = max_logvar - np.log(np.exp(max_logvar - val) + 1)
        logsigma = min_logvar + np.log(np.exp(logsigma - min_logvar) + 1)
    elif backend == 'keras':
        logsigma = max_logvar - K.log(K.exp(max_logvar - val) + 1)
        logsigma = min_logvar + K.log(K.exp(logsigma - min_logvar) + 1)
    return logsigma


def closest_individual(focal_id, individual_poses):
    distance = []
    fpos = individual_poses[(focal_id*2):(focal_id*2+2)]
    for i in range(individual_poses.shape[1] // 2):
        npos = individual_poses[(i*2):(i*2+2)]
        distance.append(np.sqrt((npos[0, 0] - fpos[0, 0])
                        ** 2 + (npos[0, 1] - fpos[0, 1]) ** 2))
    ind_idcs = [x for _, x in sorted(
        zip(distance, list(range(len(individual_poses)))))]
    ind_idcs.remove(focal_id)
    return ind_idcs


def shuffled_individuals(focal_id, individual_poses):
    ind_ids = list(range(len(individual_poses)))
    shuffle(ind_ids)
    ind_idcs.remove(focal_id)
    return ind_ids


most_influential_individual = {
    'closest': closest_individual,
    'shuffled': shuffled_individuals,
}


class DLI:
    def __init__(self):
        rate = rospy.get_param("dl_interaction/rate", 8.3)
        self._dt = 1 / rate  # hardcode for now
        self._dt = max(self._dt, 1e-3)

        self._model_path = rospy.get_param(
            'dl_interaction/model_path', './dl_interaction.h5')
        if '~' in self._model_path:
            home = os.path.expanduser('~')
            self._model_path = self._model_path.replace('~', home)

        self._num_timesteps = rospy.get_param(
            'dl_interaction/num_timesteps', 5)
        self._num_neighs = rospy.get_param(
            'dl_interaction/num_neighs', 1)
        self._most_influential_ind = rospy.get_param(
            'dl_interaction/most_influential_ind', 'closest')
        self._selection = most_influential_individual[self._most_influential_ind]
        self._radius = rospy.get_param(
            'dl_interaction/radius', 0.25)
        self._distance_inputs = rospy.get_param(
            'dl_interaction/distance_inputs', True)
        self._rate = rospy.get_param("dl_interaction/rate", 8.3)
        self._lost_count_thres = rospy.get_param(
            "dl_interaction/lost_count_thres", 5)
        self._always_current_pos = rospy.get_param(
            'dl_interaction/always_current_pos', True)
        self._id = rospy.get_param(
            'robot_id', -1)
        assert self._id >= 0, 'robot_id must be >= 0'

        self._model = self._load_keras_model(self._model_path)
        self._individual_poses = None
        self._individual_velocities = None
        self._dt = 1 / self._rate
        self._var_coef = 0.0

        self._prev_target = None
        self._lure_rescue = False
        self._lure_lost_count = 0
        self._robot_pose = None
        self._speed = None

        self._center = [0, 0]
        cx = rospy.get_param(
            'top_camera/camera_px_width_undistorted', 507)
        cy = rospy.get_param(
            'top_camera/camera_px_height_undistorted', 508)
        self._top_pix2m = rospy.get_param(
            'top_camera/pix2m', -1)
        self._bot_pix2m = rospy.get_param(
            'bottom_camera/pix2m', -1)
        assert self._top_pix2m > 0 and self._bot_pix2m > 0
        self._center = [
            (cx / 2) * self._top_pix2m, (cy / 2) * self._top_pix2m]

        self._prev_stamp = rospy.Time.now()
        self._fp_sub = rospy.Subscriber(
            'dl_interaction/filtered_poses_drop', PoseVec, self._filtered_poses_cb)
        self._rp_sub = rospy.Subscriber(
            'dl_interaction/robot_poses_drop', PoseVec, self._robot_poses_cb)
        # self._fp_sub = rospy.Subscriber(
        #     'filtered_poses', PoseVec, self._filtered_poses_cb)
        # self._rp_sub = rospy.Subscriber(
        #     'robot_poses', PoseVec, self._robot_poses_cb)

        self._target_p_pub = rospy.Publisher(
            'target_position', TargetPose, queue_size=1)
        self._target_v_pub = rospy.Publisher(
            'target_velocities', MotorVelocities, queue_size=1)
        self._dli_spec_pub = rospy.Publisher(
            'dli_specs', DLISpecs, queue_size=1)

        rospy.wait_for_service('convert_bottom2top')
        rospy.wait_for_service('convert_top2bottom')

        p = Point()
        p.x = self._center[0]
        p.y = self._center[1]
        self._bcenter = self._convert_top2bottom(p)

    def _convert_top2bottom(self, point):
        try:
            srv = rospy.ServiceProxy('convert_top2bottom', ConvertCoordinates)
            resp = srv(point)
            return resp.converted_p
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def _convert_bottom2top(self, point):
        try:
            srv = rospy.ServiceProxy('convert_bottom2top', ConvertCoordinates)
            resp = srv(point)
            return resp.converted_p
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def _load_keras_model(self, path):
        custom_objects = {
            'Y': np.empty((0, 2)),
        }

        for k, v in losses.items():
            custom_objects[k] = v
        for k, v in activations.items():
            custom_objects[k] = v

        return tf.keras.models.load_model(path, custom_objects=custom_objects)

    def spin_once(self):
        if (self._individual_poses is None or self._individual_poses.shape[0] < self._num_timesteps):
            return
        start = rospy.Time.now()

        new_p, new_v, valid, specs = self.__call__(self._id)

        if not valid:
            return

        if new_p is None:
            return

        target_p = TargetPose()
        target_p.target.header.stamp = start
        target_p.target.pose.xyz.x = new_p[0] * self._radius + self._bcenter.x
        target_p.target.pose.xyz.y = new_p[1] * self._radius + self._bcenter.y
        if self._prev_target is None:
            target_p.desired_speed = 0.05
        else:
            x_new = new_p[0] * self._radius
            x_old = self._prev_target[0] * self._radius
            y_new = new_p[1] * self._radius
            y_old = self._prev_target[1] * self._radius
            target_p.desired_speed = np.sqrt((x_new-x_old) ** 2
                                             + (y_new - y_old) ** 2) / (1 / self._rate)
            self._speed = [new_v[0], new_v[1]]

        self._prev_target = new_p
        target_p.target.header.stamp = start
        self._target_p_pub.publish(target_p)

        dli_specs = DLISpecs()
        dli_specs.gx = specs[0]
        dli_specs.gy = specs[1]
        dli_specs.sx = specs[2]
        dli_specs.sy = specs[3]
        dli_specs.target_x = target_p.target.pose.xyz.x
        dli_specs.target_y = target_p.target.pose.xyz.y
        dli_specs.agent.pose.xyz.x = self._individual_poses[-1, self._id * 2]
        dli_specs.agent.pose.xyz.y = self._individual_poses[-1,
                                                            self._id * 2 + 1]

        # ! hardcoded indices here. This model only works with 2 individuals
        dli_specs.neigh.pose.xyz.x = self._individual_poses[-1, 2]
        dli_specs.neigh.pose.xyz.y = self._individual_poses[-1, 3]
        self._dli_spec_pub.publish(dli_specs)

        # stop = (rospy.Time.now() - start).to_sec()
        # print('Query frequency: {:.1f}'.format(1 / stop))

    def _angle_to_pipi(angle):
        """
        :param angle: float angle difference between the heading of two individuals
        :return: float smallest difference within the range of -pi and pi
        """
        while True:
            if angle < -np.pi:
                angle += 2. * np.pi
            if angle > np.pi:
                angle -= 2. * np.pi
            if (np.abs(angle) <= np.pi):
                break
        return angle

    def _compute_dist_wall(self, p):
        rad = 1 - np.sqrt(p[:, 0] ** 2 + p[:, 1] ** 2).T
        zidcs = np.where(rad < 0)
        if len(zidcs[0]) > 0:
            rad[zidcs] = 0
        return rad

    def _compute_inter_dist(self, p1, p2):
        return np.sqrt((p1[:, 0] - p2[:, 0]) ** 2 + (p1[:, 1] - p2[:, 1]) ** 2)

    def __call__(self, fidx):
        if self._individual_poses is None or self._individual_velocities is None:
            return [None, None, None, None]

        X = np.empty((self._num_timesteps, 0))

        p1 = self._individual_poses[-self._num_timesteps:,
                                    (fidx*2): (fidx*2 + 2)]
        v1 = self._individual_velocities[-self._num_timesteps:,
                                         (fidx*2): (fidx*2 + 2)]

        X = np.hstack((X, p1))
        X = np.hstack((X, v1))
        if self._distance_inputs:
            rad = self._compute_dist_wall(p1[-self._num_timesteps:, :])
            X = np.hstack((X, rad.reshape(-1, 1)))

        ind_idcs = self._selection(fidx, self._individual_poses)
        for idx in ind_idcs[:self._num_neighs]:
            midx = idx * 2
            p2 = self._individual_poses[-self._num_timesteps:, midx: (
                midx + 2)]
            v2 = self._individual_velocities[-self._num_timesteps:, midx: (
                midx + 2)]
            X = np.hstack((X, p2))
            X = np.hstack((X, v2))
            if self._distance_inputs:
                rad = self._compute_dist_wall(p2)
                X = np.hstack((X, rad.reshape(-1, 1)))
                dist = self._compute_inter_dist(p1, p2)
                X = np.hstack((X, dist.reshape(-1, 1)))

        prediction = np.array(self._model.predict(
            X.reshape(1, self._num_timesteps, X.shape[1])))
        prediction[0, 2:] = list(map(logbound, prediction[0, 2:]))
        prediction[0, 2:] = list(map(np.exp, prediction[0, 2:]))

        cp = p1[-1, :]
        cv = v1[-1, :]
        return self._sample_valid_position(
            cp, cv, prediction, self._dt)

    def _sample_valid_position(self, position, velocity, prediction, timestep):
        var = 1
        it = 0

        g_x = np.random.normal(
            prediction[0, 0], prediction[0, 2] * self._var_coef * var, 1)[0]
        g_y = np.random.normal(
            prediction[0, 1], prediction[0, 3] * self._var_coef * var, 1)[0]

        v_hat = velocity
        v_hat[0] += g_x
        v_hat[1] += g_y
        p_hat = position + v_hat * timestep
        r = np.sqrt((p_hat[0] - setup.center()[0])
                    ** 2 + (p_hat[1] - setup.center()[1]) ** 2)

        # r = 1 + 1
        # while r > self._radius and it < 50:
        #     g_x = np.random.normal(
        #         prediction[0, 0], prediction[0, 2] * self._var_coef * var, 1)[0]
        #     g_y = np.random.normal(
        #         prediction[0, 1], prediction[0, 3] * self._var_coef * var, 1)[0]

        #     v_hat = velocity
        #     v_hat[0] += g_x
        #     v_hat[1] += g_y
        #     p_hat = position + v_hat * timestep
        #     r = np.sqrt((p_hat[0] - setup.center()[0])
        #                 ** 2 + (p_hat[1] - setup.center()[1]) ** 2)
        #     var += 1
        #     it += 1

        return [p_hat, v_hat, setup.is_valid(r), [g_x, g_y, prediction[0, 2] * self._var_coef, prediction[0, 3] * self._var_coef]]

    def _filtered_poses_cb(self, msg):
        if (len(msg.poses) == 0):
            return

        if self._individual_poses is None:
            self._individual_poses = np.empty((0, 2 * (self._num_neighs + 1)))

        row = []
        for i, p in enumerate(msg.poses):
            if not self._lure_rescue and not self._always_current_pos and i == self._id and self._prev_target is not None:
                row += [self._prev_target[0], self._prev_target[1]]
            else:
                if self._lure_rescue:
                    print('Rescuing lure...')
                conv = Point()
                conv.x = p.pose.xyz.x - self._center[0]
                conv.y = p.pose.xyz.y - self._center[1]
                conv.x /= self._radius
                conv.y /= self._radius
                row.append(conv.x)
                row.append(conv.y)
        row = np.array(row)
        if row.shape[0] < 4:
            if self._individual_poses.shape[0] > 0:
                row = np.vstack([row, self._individual_poses[-1, 2:]])
            else:
                return
        self._individual_poses = np.vstack([self._individual_poses, row])

        if self._individual_poses.shape[0] > self._num_timesteps:
            rolled_m = np.roll(self._individual_poses, shift=1, axis=0)
            self._individual_velocities = (
                self._individual_poses - rolled_m) / self._dt
            if self._speed is not None:
                self._individual_velocities[-1, 0] = self._speed[0]
                self._individual_velocities[-1, 1] = self._speed[1]

            self._individual_poses = self._individual_poses[-self._num_timesteps:, :]
            self._individual_velocities = self._individual_velocities[-self._num_timesteps:, :]

    def _robot_poses_cb(self, msg):
        self._robot_pose = msg.poses[self._id]
        if self._individual_poses is None:
            return

        # from copy import deepcopy
        # ind_pose = deepcopy(
        #     self._individual_poses[-1, self._id * 2: (self._id * 2 + 2)])
        # ind_pose *= self._radius
        # ind_pose[0] += self._bcenter.x
        # ind_pose[1] += self._bcenter.y

        # dist = np.sqrt((self._robot_pose.pose.xyz.x -
        #                 ind_pose[0]) ** 2 + (self._robot_pose.pose.xyz.y - ind_pose[1]) ** 2)

        # if dist > 0.1:
        #     print(dist)
        #     self._lure_lost_count += 1
        # else:
        #     self._lure_lost_count += 0
        #     self._lure_rescue = False

        # if self._lure_lost_count > self._lost_count_thres:
        #     self._lure_rescue = True


def main():
    rospy.init_node('dl_interaction_node')
    rate = rospy.get_param("dl_interaction/rate", 8.3)
    rate = rospy.Rate(rate)
    dli = DLI()
    while not rospy.is_shutdown():
        dli.spin_once()
        rate.sleep()


if __name__ == '__main__':
    main()
