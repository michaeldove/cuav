from camera import source

################################################################################
# Exercise camera module using a fake camera
################################################################################

class MAV:
   
    def data16_send(self, x, length, buf):
        import sys
	func_name = sys._getframe().f_code.co_name
        print func_name, length

    def data32_send(self, x, length, buf):
        import sys
	func_name = sys._getframe().f_code.co_name
        print func_name, length

    def data64_send(self, x, length, buf):
        import sys
	func_name = sys._getframe().f_code.co_name
        print func_name, length

    def data96_send(self, x, length, buf):
        import sys
	func_name = sys._getframe().f_code.co_name
        print func_name, length

class Connection:
    mav = None

    def __init__(self, mav):
        self.mav = mav

class MPStatus:

    logdir = ''

class MPSettings:

    completion = None

class MPState:

    status = MPStatus()
    continue_mode = None
    command_map = {}
    completions = {}
    completion_functions = {}
    settings = MPSettings()
    mav_master = []

from cuav.camera.fake_camera import FakeCamera
camera_mod = source.init(MPState(), FakeCamera(source_dir='../data/sample'))
camera_mod.mpstate.command_map['camera'][0](['start'])
camera_mod.mpstate.mav_master.append(Connection(MAV()))
camera_mod.cmd_camera(['start'])
import time
time.sleep(5)
