from panda3d.core import TextNode
from panda3d.core import Vec3
from panda3d.core import VBase3
from panda3d.core import Point3

from direct.showbase.DirectObject import DirectObject
from direct.gui.OnscreenText import OnscreenText

from vehicle import REPULSOR_RAY_ACTIVE
from vehicle import REPULSOR_RAY_POS
from vehicle import REPULSOR_RAY_DIR
from vehicle import REPULSOR_RAY_FRAC

from keybindings import GE_CAMERA_MODE

from controller import DM_STUNT
from controller import DM_CRUISE


CAM_MODE_FOLLOW = 1
CAM_MODE_DIRECTION = 2
CAM_MODE_MIXED = 3
CAM_MODES = [CAM_MODE_FOLLOW, CAM_MODE_DIRECTION, CAM_MODE_MIXED]


class CameraController(DirectObject):
    def __init__(self, camera, vehicle, control):
        self.camera = camera
        self.vehicle = vehicle
        self.control = control
        self.camera.reparent_to(base.render)

        self.camera_mode = 0
        self.accept(GE_CAMERA_MODE, self.switch_camera_mode)

        self.speed = OnscreenText(
            text = '',
            pos = (1.3, 0.8),
            scale = 0.15,
            fg = (1.0, 1.0, 0.5, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )

        self.driving_mode = OnscreenText(
            text = '',
            pos = (1.3, 0.7),
            scale = 0.1,
            fg = (0.0, 0.0, 0.0, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )

        self.repulsor_hud = self.camera.attach_new_node('repulsor HUD')
        self.repulsor_hud.set_pos(-2, 10, 2)
        self.repulsor_hud.set_p(90)
        self.repulsor_models = []

    def switch_camera_mode(self):
        self.camera_mode = (self.camera_mode + 1) % len(CAM_MODES)

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def update(self):
        self.update_camera()
        self.update_gui()

    def update_camera(self):
        horiz_dist = 20
        cam_offset = Vec3(0, 0, 5)
        focus_offset = Vec3(0, 0, 2)
        vehicle_pos = self.vehicle.np().get_pos(base.render)
        if CAM_MODES[self.camera_mode] == CAM_MODE_FOLLOW:
            vehicle_back = base.render.get_relative_vector(
                self.vehicle.np(),
                Vec3(0, -1, 0),
            )
        elif CAM_MODES[self.camera_mode] == CAM_MODE_DIRECTION:
            vehicle_back = -self.vehicle.physics_node.get_linear_velocity()
        elif CAM_MODES[self.camera_mode] == CAM_MODE_MIXED:
            vehicle_back = base.render.get_relative_vector(
                self.vehicle.np(),
                Vec3(0, -1, 0),
            )
            movement = self.vehicle.physics_node.get_linear_velocity()
            movement_back = -movement / movement.length()
            vehicle_back = vehicle_back + movement_back
        vehicle_back.z = 0
        vehicle_back = vehicle_back / vehicle_back.length()

        cam_pos = vehicle_pos + vehicle_back * horiz_dist + cam_offset
        focus = vehicle_pos + focus_offset
        self.camera.set_pos(cam_pos)
        self.camera.look_at(focus)

    def update_gui(self):
        # Speed
        mps = self.vehicle.physics_node.get_linear_velocity().length()
        kmh = mps * 60 * 60 / 1000
        self.speed['text'] = "{:03.1f} km/h".format(kmh)

        # Driving mode
        mode = self.control.driving_mode
        if mode == DM_STUNT:
            self.driving_mode['text'] = 'STUNT MODE!'
            self.driving_mode['fg'] = (1.0, 0.0, 0.0, 1.0)
        elif mode == DM_CRUISE:
            self.driving_mode['text'] = 'Cruise mode'
            self.driving_mode['fg'] = (0.5, 1.0, 0.5, 1.0)

        # Repulsors
        ray_poss = self.vehicle.sensors[REPULSOR_RAY_POS]
        ray_dirs = self.vehicle.sensors[REPULSOR_RAY_DIR]
        ray_actives = self.vehicle.sensors[REPULSOR_RAY_ACTIVE]
        ray_fracs = self.vehicle.sensors[REPULSOR_RAY_FRAC]
        ray_data = zip(ray_poss, ray_dirs, ray_actives, ray_fracs)
        num_models_delta = len(ray_dirs) - len(self.repulsor_models)
        if num_models_delta > 0:
            # We need more models
            for _ in range(num_models_delta):
                on_model = base.loader.load_model('models/smiley')
                on_model.reparent_to(self.repulsor_hud)
                on_model.set_scale(0.1)
                on_model.set_p(-90)
                on_model.hide()
                off_model = base.loader.load_model('models/frowney')
                off_model.reparent_to(self.repulsor_hud)
                off_model.set_scale(0.1)
                off_model.set_p(-90)
                off_model.hide()
                self.repulsor_models.append((on_model, off_model))
        elif num_models_delta < 0:
            for on_model, off_model in self.repulsor_models[num_models_delta:]:
                on_model.remove_node()
                off_model.remove_node()
            self.repulsor_models = self.repulsor_models[:len(ray_dirs)]
        for (ray_pos, ray_dir, ray_active, ray_frac), (on_model, off_model) in zip(ray_data, self.repulsor_models):
            offset_dir = self.vehicle.np().get_relative_vector(
                base.render,
                ray_dir,
            )
            offset = ray_pos * 0.1 + offset_dir * ray_frac * 0.1
            if ray_active:
                on_model.set_pos(offset)
                on_model.show()
                off_model.hide()
            else:
                off_model.set_pos(offset)
                off_model.show()
                on_model.hide()
