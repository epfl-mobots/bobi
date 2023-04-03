import rospy
import cv2
from subprocess import Popen, DEVNULL, PIPE


class VideoRec:
    def __init__(self, filename, width, height, fps, type='mp4'):
        self._filename = filename
        self._width = width
        self._height = height
        self._fps = fps

        if type == 'mp4':
            fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
            ext = '.mp4'
        elif type == 'avi':
            fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
            ext = '.avi'

        self._vr = cv2.VideoWriter(
            self._filename + ext, fourcc, fps, (width, height))

    def write(self, img, t, stamp=True):
        if stamp:
            cv2.putText(img,
                        'Log time: {:.3f}'.format(t),
                        (int(img.shape[1]*0.55), int(img.shape[0]*0.97)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 200, 200),
                        2,
                        cv2.LINE_AA)
        if self._vr.isOpened():
            self._vr.write(img)

    def release(self):
        if self._vr.isOpened():
            self._vr.release()

    def __del__(self):
        self.release()

class Mp4Rec(VideoRec):
    def __init__(self, filename, width, height, fps):
        VideoRec.__init__(self, filename, width, height, fps, type='mp4')

class AviRec(VideoRec):
    def __init__(self, filename, width, height, fps):
        VideoRec.__init__(self, filename, width, height, fps, type='avi')