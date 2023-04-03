#!/usr/bin/env python
import rospy
import geometry_msgs
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from bobi_msgs.msg import PoseVec, PoseStamped, KickSpecs, DLISpecs, MotorVelocities, NumAgents
from bobi_msgs.srv import ConvertCoordinates, GetNumAgents
from copy import deepcopy
from video_rec import Mp4Rec, AviRec

from dynamic_reconfigure.server import Server
from bobi_stats.cfg import LoggerConfig

import os
import socket
from datetime import datetime
import numpy as np
from tqdm import tqdm
import cv2

NOT_FOUND = 9999999.9
START_TIME = None


class SystemLogs:
    def __init__(self, output_folder, num_agents, num_robots, num_virtu_agents):
        self._num_agents = num_agents
        self._num_robots = num_robots
        self._num_virtu_agents = num_virtu_agents
        self._of = output_folder

        self._fp_init = False
        self._up_init = False
        self._ro_init = False
        self._ks_init = False
        self._dli_init = False
        self._sv_init = False

        # log files
        self._fp_top_file = open(
            '{}/id_poses_top.txt'.format(self._of), 'w')

        self._up_top_file = open(
            '{}/raw_poses_top.txt'.format(self._of), 'w')

        self._ro_bot_file = open(
            '{}/robot_poses_bot.txt'.format(self._of), 'w')

        self._ks_top_file = open(
            '{}/kick_specs_top.txt'.format(self._of), 'w')

        self._dli_top_file = open(
            '{}/dli_specs_top.txt'.format(self._of), 'w')

        self._sv_file = open(
            '{}/set_velocities.txt'.format(self._of), 'w')

        # subscribers
        self._fp_sub = rospy.Subscriber(
            'filtered_poses', PoseVec, self._filtered_poses_cb)
        self._up_sub = rospy.Subscriber(
            'naive_poses', PoseVec, self._unfiltered_poses_cb)
        self._ro_sub = rospy.Subscriber(
            'robot_poses', PoseVec, self._robot_poses_cb)
        self._ks_sub = rospy.Subscriber(
            'kick_specs', KickSpecs, self._kick_specs_cb)
        self._dl_sub = rospy.Subscriber(
            'dli_specs', DLISpecs, self._dli_specs_cb)

        self._sv_sub = rospy.Subscriber(
            'set_velocities', MotorVelocities, self._motor_velocities_cb)

        self._bridge = CvBridge()

        self._top_annot_vr = None
        self._top_annot_img_sub = rospy.Subscriber(
            'top_camera/image_annot', Image, self._top_img_annot_cb)

        self._top_masked_vr = None
        self._top_masked_img_sub = rospy.Subscriber(
            'top_camera/image_masked', Image, self._top_img_masked_cb)

        self._bot_annot_vr = None
        self._bot_annot_img_sub = rospy.Subscriber(
            'bottom_camera/image_annot', Image, self._bot_img_annot_cb)

        self._bot_raw_vr = None
        self._bot_raw_img_sub = rospy.Subscriber(
            'bottom_camera/image_raw', Image, self._bot_img_raw_cb)

        self._top_raw_vr = None
        self._top_raw_img_sub = rospy.Subscriber(
            'top_camera/image_raw', Image, self._top_img_raw_cb)

    def __del__(self):
        self._fp_sub.unregister()
        self._up_sub.unregister()
        self._ro_sub.unregister()
        self._ks_sub.unregister()
        self._dl_sub.unregister()
        self._top_annot_img_sub.unregister()
        self._top_masked_img_sub.unregister()
        self._bot_annot_img_sub.unregister()
        self._bot_raw_img_sub.unregister()
        self._top_raw_img_sub.unregister()

        self._fp_top_file.close()
        self._up_top_file.close()
        self._ro_bot_file.close()
        self._ks_top_file.close()
        self._dli_top_file.close()
        self._sv_file.close()

        if self._top_raw_vr is not None:
            self._top_raw_vr.release()
            del self._top_raw_vr
        if self._top_annot_vr is not None:
            self._top_annot_vr.release()
            del self._top_annot_vr
        if self._top_masked_vr is not None:
            self._top_masked_vr.release()
            del self._top_masked_vr
        if self._bot_annot_vr is not None:
            self._bot_annot_vr.release()
            del self._bot_annot_vr
        if self._bot_raw_vr is not None:
            self._bot_raw_vr.release()
            del self._bot_raw_vr

    def update(self):
        pass

    def _filtered_poses_cb(self, msg):
        data = msg.poses

        t = self._get_stamp()
        if not self._fp_init:
            self._fp_top_file.write('t ')
            for _ in range(self._num_agents + self._num_virtu_agents):
                self._fp_top_file.write('x y yaw is_filtered is_swapped ')
            self._fp_top_file.write('\n')
            self._fp_init = True

        self._fp_top_file.write('{:5f}'.format(t))

        for p in data:
            self._fp_top_file.write(' {:.6f} {:.6f} {:.6f} {} {}'.format(
                p.pose.xyz.x, p.pose.xyz.y, p.pose.rpy.yaw, int(p.pose.is_filtered == True), int(p.pose.is_swapped == True)))
        if len(data) < self._num_agents + self._num_virtu_agents:
            for _ in range(self._num_agents + self._num_virtu_agents - len(data)):
                self._fp_top_file.write(' {} {} {} {} {}'.format(
                    NOT_FOUND, NOT_FOUND, NOT_FOUND, 0, 0))

        self._fp_top_file.write('\n')
        self._fp_top_file.flush()

    def _unfiltered_poses_cb(self, msg):
        data = msg.poses

        t = self._get_stamp()
        if not self._up_init:
            self._up_top_file.write('t ')
            for _ in range(self._num_agents + self._num_virtu_agents):
                self._up_top_file.write('x y yaw is_filtered is_swapped ')
            self._up_top_file.write('\n')
            self._up_init = True

        self._up_top_file.write('{:5f}'.format(t))
        for p in data:
            self._up_top_file.write(' {:.6f} {:.6f} {:.6f} {} {}'.format(
                p.pose.xyz.x, p.pose.xyz.y, p.pose.rpy.yaw, int(p.pose.is_filtered == True), int(p.pose.is_swapped == True)))
        if len(data) < self._num_agents + self._num_virtu_agents:
            for _ in range(self._num_agents + self._num_virtu_agents - len(data)):
                self._up_top_file.write(' {} {} {} {} {}'.format(
                    NOT_FOUND, NOT_FOUND, NOT_FOUND, 0, 0))

        self._up_top_file.write('\n')
        self._up_top_file.flush()

    def _robot_poses_cb(self, msg):
        data = msg.poses

        t = self._get_stamp()
        if not self._ro_init:
            self._ro_bot_file.write('t ')
            for _ in range(self._num_agents + self._num_virtu_agents):
                self._ro_bot_file.write('x y yaw is_filtered is_swapped ')
            self._ro_bot_file.write('\n')
            self._ro_init = True

        self._ro_bot_file.write('{:5f}'.format(t))
        for p in data:
            self._ro_bot_file.write(' {:.6f} {:.6f} {:.6f} {} {}'.format(
                p.pose.xyz.x, p.pose.xyz.y, p.pose.rpy.yaw, int(p.pose.is_filtered == True), int(p.pose.is_swapped == True)))
        if len(data) < self._num_robots:
            for _ in range(self._num_robots - len(data)):
                self._ro_bot_file.write(' {} {} {} {} {}'.format(
                    NOT_FOUND, NOT_FOUND, NOT_FOUND, 0, 0))

        self._ro_bot_file.write('\n')
        self._ro_bot_file.flush()

    def _kick_specs_cb(self, msg):
        point = geometry_msgs.msg.Point(NOT_FOUND, NOT_FOUND, 0.)

        t = self._get_stamp()
        if not self._ks_init:
            self._ks_top_file.write('t x y yaw intervene gatt gali')
            for i in range(self._num_agents + self._num_virtu_agents - 1):
                self._ks_top_file.write(' n_x n_y n_yaw')
            self._ks_top_file.write(' target_x target_y dl phi dphi tau ta0')
            self._ks_top_file.write('\n')
            self._ks_init = True

        self._ks_top_file.write('{:5f}'.format(t))
        self._ks_top_file.write(' {:.6f} {:.6f} {:.6f} {} {:.6f} {:.6f}'.format(
            msg.agent.pose.xyz.x, msg.agent.pose.xyz.y, msg.agent.pose.rpy.yaw, msg.intervene, msg.gatt, msg.gali))
        for n in msg.neighs.poses:
            self._ks_top_file.write(' {:.6f} {:.6f} {:.6f}'.format(
                n.pose.xyz.x, n.pose.xyz.y, n.pose.rpy.yaw))
        if len(msg.neighs.poses) < self._num_agents + self._num_virtu_agents - 1:
            for _ in range(self._num_agents + self._num_virtu_agents - 1 - len(msg.neighs.poses)):
                self._ks_top_file.write(' {} {} {}'.format(
                    NOT_FOUND, NOT_FOUND, NOT_FOUND))

        self._ks_top_file.write(' {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}'.format(
            msg.target_x, msg.target_y, msg.dl, msg.phi, msg.dphi, msg.tau, msg.tau0))
        self._ks_top_file.write('\n')
        self._ks_top_file.flush()

    def _dli_specs_cb(self, msg):
        point = geometry_msgs.msg.Point(NOT_FOUND, NOT_FOUND, 0.)

        t = self._get_stamp()
        if not self._dli_init:
            self._dli_top_file.write('t gx gy sx sy target_x target_y')
            self._dli_top_file.write(' x y yaw')
            self._dli_top_file.write(' n_x n_y n_yaw')
            self._dli_top_file.write('\n')

        self._dli_top_file.write('{:5f}'.format(t))
        self._dli_top_file.write(' {:6f} {:6f} {:6f} {:6f}'.format(
            msg.gx, msg.gy, msg.sx, msg.sy))
        self._dli_top_file.write(' {:.6f} {:.6f} {:.6f}'.format(
            msg.agent.pose.xyz.x, msg.agent.pose.xyz.y, msg.agent.pose.rpy.yaw))
        self._dli_top_file.write(' {:.6f} {:.6f} {:.6f}'.format(
            msg.neigh.pose.xyz.x, msg.neigh.pose.xyz.y, msg.neigh.pose.rpy.yaw))
        self._dli_top_file.write('\n')
        self._dli_top_file.flush()

    def _motor_velocities_cb(self, msg):
        t = self._get_stamp()
        self._sv_file.write('{} {} {} {}'.format(
            t, msg.left, msg.right, msg.acceleration))
        self._sv_file.write('\n')
        self._sv_file.flush()

    def _top_img_annot_cb(self, msg):
        t = self._get_stamp()
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        if img.shape[0] > 0 and img.shape[1] > 0:
            if self._top_annot_vr is None:
                fps = rospy.get_param('top_camera/fps')
                self._top_annot_vr = Mp4Rec(
                    '{}/top_annot'.format(self._of), img.shape[1], img.shape[0], fps)
            self._top_annot_vr.write(img, t)

    def _top_img_masked_cb(self, msg):
        t = self._get_stamp()
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        if img.shape[0] > 0 and img.shape[1] > 0:
            if self._top_masked_vr is None:
                fps = rospy.get_param('top_camera/fps')
                self._top_masked_vr = Mp4Rec(
                    '{}/top_masked'.format(self._of), img.shape[1], img.shape[0], fps)
            self._top_masked_vr.write(img, t)

    def _bot_img_annot_cb(self, msg):
        t = self._get_stamp()
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        if img.shape[0] > 0 and img.shape[1] > 0:
            if self._bot_annot_vr is None:
                fps = rospy.get_param('bottom_camera/fps')
                self._bot_annot_vr = Mp4Rec(
                    '{}/bot_annot'.format(self._of), img.shape[1], img.shape[0], fps)
            self._bot_annot_vr.write(img, t)

    def _bot_img_raw_cb(self, msg):
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        if img.shape[0] > 0 and img.shape[1] > 0:
            if self._bot_raw_vr is None:
                fps = rospy.get_param('bottom_camera/fps')
                self._bot_raw_vr = Mp4Rec(
                    '{}/bot_raw'.format(self._of), img.shape[1], img.shape[0], fps)
            self._bot_raw_vr.write(img, 0, False)

    def _top_img_raw_cb(self, msg):
        img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        if img.shape[0] > 0 and img.shape[1] > 0:
            if self._top_raw_vr is None:
                fps = rospy.get_param('top_camera/fps')
                self._top_raw_vr = Mp4Rec(
                    '{}/top_raw'.format(self._of), img.shape[1], img.shape[0], fps)
            self._top_raw_vr.write(img, 0, False)

    def _get_stamp(self):
        global START_TIME, NOT_FOUND
        nsecs = rospy.Time.now().nsecs
        secs = rospy.Time.now().secs
        t = secs + (nsecs / 10 ** 9)
        if START_TIME is None:
            START_TIME = t
            t = 0
        else:
            t -= START_TIME
        return t


class HighResRec:
    def __init__(self, output_folder, device, num_buffers, fps=30):
        self._of = output_folder
        self._device = device
        self._num_buffers = num_buffers

        self._count = 0
        self._pbar = None
        self._dev = None
        self._vr = None
        self._done = False

        if self._num_buffers > -2:
            # self._dev = cv2.VideoCapture(device)
            self._dev = cv2.VideoCapture(device, cv2.CAP_V4L2)
            # self._dev = cv2.VideoCapture(device, cv2.CAP_GSTREAMER)
            if not self._dev.isOpened():
                assert False, 'Failed to open stream'

            self._vr = AviRec(
                '{}/hq-rec'.format(self._of), 1500, 1500, 30)

    def __del__(self):
        if self._dev is not None and self._dev.isOpened():
            self._dev.release()

        if self._vr is not None:
            self._vr.release()

        if self._pbar is not None:
            self._pbar.close()

    def update(self):
        if self._num_buffers > -2:
            if self._pbar is None:
                if self._num_buffers == -1:
                    self._num_buffers = 400000
                self._pbar = tqdm(total=self._num_buffers)

            ret, frame = self._dev.read()
            if ret:
                self._vr.write(frame, 0, False)
                self._count += 1
                self._pbar.update(1)

            if self._num_buffers > -1 and self._count >= self._num_buffers:
                self._count = 0
                self._num_buffers = -2
                self._dev.release()
                self._vr.release()
                self._pbar.close()
                self._pbar = None
                self._done = True

    def is_done(self):
        return self._done


class Logs:
    def __init__(self):
        self._device = 4
        self._num_buffers = -2

        self._sl = None
        self._hrr = None

        rospy.wait_for_service('get_num_agents')
        try:
            get_num_agents = rospy.ServiceProxy('get_num_agents', GetNumAgents)
            resp = get_num_agents()
            self._num_agents = resp.info.num_agents
            self._num_robots = resp.info.num_robots
            self._num_virtu_agents = resp.info.num_virtu_agents
        except rospy.ServiceException as e:
            pass

        self._cfg_srv = Server(LoggerConfig, self._cfg_cb)
        rospy.Subscriber("num_agents_update", NumAgents, self._num_agents_cb)

    def _cfg_cb(self, config, level):
        reset = False

        if self._device != config.device:
            self._device = config.device
            reset = True

        self._start_new_session = config.start_new_session
        if self._start_new_session:
            reset = True

        if self._start_new_session and (reset == False or self._num_buffers != config.num_buffers):
            self._num_buffers = config.num_buffers
            reset = True

        if reset:
            self._reset()

        return config

    def update(self):
        if self._sl is not None:
            self._sl.update()

        if self._hrr is not None:
            self._hrr.update()

            if self._hrr.is_done():
                self._reset()

    def _reset(self):
        self._type = 'bobi'

        self._of = rospy.get_param('logger/output_folder', '.')
        if '~' in self._of:
            home = os.path.expanduser('~')
            self._of = self._of.replace('~', home)
        now = datetime.now()
        date_time = now.strftime("%m_%d_%Y-%H_%M_%S")

        self._of = '{}/{}-{}_{}-{}-{}'.format(self._of, self._type, date_time,
                                              self._num_agents, self._num_robots, self._num_virtu_agents)
        os.makedirs(self._of)

        if self._sl is not None:
            del self._sl
            self._sl = None
        if self._hrr is not None:
            del self._hrr
            self._hrr = None

        self._sl = SystemLogs(self._of, self._num_agents,
                              self._num_robots, self._num_virtu_agents)
        self._hrr = HighResRec(self._of, self._device, self._num_buffers)

    def _num_agents_cb(self, msg):
        self._num_agents = msg.num_agents
        self._num_robots = msg.num_robots
        self._num_virtu_agents = msg.num_virtu_agents
        self._reset()


def main():
    rospy.init_node('logger_node')
    rate = rospy.get_param("top_camera/fps", 80)
    rate = rospy.Rate(rate)

    l = Logs()
    while not rospy.is_shutdown():
        l.update()
        rate.sleep()


if __name__ == '__main__':
    main()
