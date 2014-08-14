#!/usr/bin/env
'''
emulate a camera, getting images from a playback tool

The API represents generic camera interface, but takes images from fake_chameleon.pgm
'''

from cuav.camera import pgm_utils
from camera import CameraError

import time, os, sys, cv, numpy

from cuav.lib import cuav_util
from cuav.image import scanner

class FakeCamera:
    """File sourced fake camera"""

    continuous_mode = False
    fake = 'fake_chameleon.pgm'
    frame_counter = 0
    trigger_time = 0
    frame_rate = 7.5
    chameleon_gamma = 950
    last_frame_time = 0
    source_dir = None
    filename_iterator = None

    def __init__(self, source_dir='.'):
        self.source_dir = source_dir

    def open(self, colour, depth, brightness):
        self.filename_iterator = iter(self.image_filenames())
        return 0

    def is_open(self):
        """True if camera is opened for capture."""
        return self.filename_iterator is not None

    def trigger(self, continuous):
        self.continuous_mode = continuous
        self.trigger_time = time.time()

    def load_image(self, filename):
        if filename.endswith('.pgm'):
            fake_img = cuav_util.PGM(filename)
            return fake_img.array
        img = cv.LoadImage(filename)
        array = numpy.asarray(cv.GetMat(img))
        grey = numpy.zeros((960,1280), dtype='uint8')
        scanner.rebayer(array, grey)
        return grey
        

    def capture(self, timeout, img):
        tnow = time.time()
        due = self.trigger_time + (1.0/self.frame_rate)
        if tnow < due:
            time.sleep(due - tnow)
            timeout -= int(due*1000)
        # wait for a new image to appear
        #rel_path = os.path.join(self.source_dir, self.fake)
        #filename = os.path.realpath(rel_path)
        #frame_time = cuav_util.parse_frame_time(filename)
        filename = self.filename_iterator.next()
        #frame_time = 0
#	print timeout
#        while frame_time == self.last_frame_time and timeout > 0:
#            timeout -= 10
#            time.sleep(0.01)
#            #rel_path = os.path.join(self.source_dir, self.fake)
#            #filename = os.path.realpath(rel_path)
#            #frame_time = cuav_util.parse_frame_time(filename)
#            frame_time += 1
#
#	print self.last_frame_time, frame_time
#        if self.last_frame_time == frame_time:
#            raise CameraError("timeout waiting for fake image")
        #self.last_frame_time = frame_time
        try:
            fake_img = self.load_image(filename)
        except Exception, msg:
            raise CameraError('missing %s' % self.fake)
        self.frame_counter += 1
        img.data = fake_img.data
        if self.continuous_mode:
            self.trigger_time = time.time()
        return self.trigger_time, self.frame_counter, 0

    def close(self):
        self.filename_iterator = None

    def set_gamma(self, gamma):
        self.chameleon_gamma = gamma

    def set_framerate(self, framerate):
        if self.framerate >= 15:
            self.frame_rate = 15
        elif self.framerate >= 7:
            self.frame_rate = 7.5
        elif self.framerate >= 3:
            self.frame_rate = 3.75
        else:
            self.frame_rate = 1.875;

    def save_pgm(self, filename, img):
        pass

    def save_file(self, filename, bytes):
        pass

    def image_filenames(self):
        """Provide an ordered list of all the images in source_dir"""
        pgm_files = filter(lambda fn: fn.lower().endswith('.pgm'), os.listdir(self.source_dir))
        abspath_pgm_files = map(lambda fn: os.path.join(self.source_dir,
                                                         fn), pgm_files)
        abspath_pgm_files.sort()
        return abspath_pgm_files


if __name__ == '__main__':

    import time
    import cv2

    cam = FakeCamera(source_dir = '../data/sample')
    cam.open(1, 1, 1)
    width, height = (1280, 960)
    img = numpy.zeros((height,width),dtype='uint8')

    cv2.namedWindow('dst_rt', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('dst_rt', width, height)

    for i in range(600):
        cam.capture(10, img)
        cv2.imshow('dst_rt', img)
        time.sleep(1/30.0)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
