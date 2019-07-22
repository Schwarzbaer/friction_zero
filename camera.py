from direct.showbase.DirectObject import DirectObject


class CameraController(DirectObject):
    def init(self, camera, vehicle, control):
        self.vehicle = vehicle
        self.control = control
        self.camera = camera

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle
        self.set_camera_mode(self.camera_mode)
