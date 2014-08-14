from . import chameleon
from camera import CameraError

def translate_error(func):
    """Translate an error from one type to another."""
    def error_rewrite(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except chameleon.error, e:
            raise CameraError(*e.args)
    return error_rewrite


class ChameleonCamera:

    camera_fd = None

    @translate_error
    def open(self, colour, depth, brightness):
        self.camera_fd = chameleon.open(colour, depth, brightness)

    @translate_error
    def close(self):
        return chameleon.close(self.camera_fd)

    @translate_error
    def trigger(self, continuous):
        chameleon.trigger(self.camera_fd, continuous)

    @translate_error
    def capture(self, timeout, img):
        return chameleon.capture(self.camera_fd, timeout, img)

    @translate_error
    def set_brightness(self, brightness):
        chameleon.set_brightness(brightness)

    @translate_error
    def set_gamma(self, gamma):
        chameleon.set_gamma(self.camera_fd, gamma)

    @translate_error
    def set_framerate(self, framerate):
        chameleon.set_framerate(self.camera_fd, framerate)
