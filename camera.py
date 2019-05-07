from enum import Enum

from panda3d.core import TextNode
from panda3d.core import NodePath
from panda3d.core import Vec3
from panda3d.core import VBase3
from panda3d.core import Point3

from direct.showbase.DirectObject import DirectObject
from direct.gui.OnscreenText import OnscreenText

from vehicle import REPULSOR_RAY_ACTIVE
from vehicle import REPULSOR_RAY_POS
from vehicle import REPULSOR_RAY_DIR
from vehicle import REPULSOR_RAY_FRAC
from vehicle import LOCAL_UP
from vehicle import FLIGHT_HEIGHT
from vehicle import TARGET_FLIGHT_HEIGHT
from vehicle import CLIMB_SPEED
from vehicle import HEIGHT_OVER_TARGET
from vehicle import HEIGHT_OVER_TARGET_PROJECTED
from vehicle import REPULSOR_POWER_FRACTION_NEEDED

from keybindings import GE_CAMERA_MODE

from controller import DM_STUNT
from controller import DM_CRUISE


COCKPIT_CAMERA = 'fz_cockpit_camera'


class CameraModes(Enum):
    FOLLOW = 1
    FIXED = 2
    COCKPIT = 3
    DIRECTION = 4
    MIXED = 5


class CameraController(DirectObject):
    def __init__(self, camera, vehicle, control):
        self.camera = camera
        self.vehicle = vehicle
        self.control = control
        self.camera.reparent_to(base.render)

        self.camera_mode = CameraModes.FOLLOW
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

        self.flight_height = OnscreenText(
            text = '',
            pos = (1.3, -0.65),
            scale = 0.1,
            fg = (0.0, 0.0, 0.0, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )
        self.target_flight_height = OnscreenText(
            text = '',
            pos = (1.3, -0.75),
            scale = 0.1,
            fg = (0.2, 0.8, 0.2, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )
        self.climb_rate = OnscreenText(
            text = '',
            pos = (1.3, -0.85),
            scale = 0.1,
            fg = (0.0, 0.0, 0.0, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )
        self.repulsor_power_needed = OnscreenText(
            text = '',
            pos = (1.3, -0.95),
            scale = 0.1,
            fg = (0.0, 0.0, 0.0, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )

        # Height meters and repulsor self-control
        self.heights = base.render2d.attach_new_node("height meters")
        self.heights.set_pos(-0.8, 0.0, -0.6)

        self.height_null = base.loader.load_model('models/box')
        self.height_null.set_scale(0.3, 0.3, 0.01)
        self.height_null.set_pos(-0.15, -0.15, -0.005)
        self.height_null.reparent_to(self.heights)

        self.height_over_target = base.loader.load_model('models/smiley')
        self.height_over_target.set_scale(0.01, 0.1, 0.1)
        self.height_over_target.set_pos(-0.1, 0, 0)
        self.height_over_target.reparent_to(self.heights)
        self.height_over_target.hide()

        self.height_over_target_projected = base.loader.load_model('models/smiley')
        self.height_over_target_projected.set_scale(0.01, 0.1, 0.1)
        self.height_over_target_projected.set_pos(-0.05, 0, 0)
        self.height_over_target_projected.reparent_to(self.heights)
        self.height_over_target_projected.hide()

        self.repulsor_hud = self.camera.attach_new_node('repulsor HUD')
        self.repulsor_hud.set_pos(-2, 10, 2)
        self.repulsor_hud.set_p(90)
        self.repulsor_models = []

    def switch_camera_mode(self):
        new_mode = ((self.camera_mode.value) % len(CameraModes)) + 1
        self.camera_mode = CameraModes(new_mode)

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def update(self):
        self.update_camera()
        self.update_gui()

    def update_camera(self):
        self.camera.reparent_to(base.render)
        horiz_dist = 20
        cam_offset = Vec3(0, 0, 5)
        focus_offset = Vec3(0, 0, 2)
        vehicle_pos = self.vehicle.np().get_pos(base.render)
        if self.camera_mode == CameraModes.FOLLOW:
            vehicle_back = base.render.get_relative_vector(
                self.vehicle.np(),
                Vec3(0, -1, 0),
            )
        elif self.camera_mode == CameraModes.FIXED:
            self.camera.reparent_to(self.vehicle.np())
            self.camera.set_pos(0, -10, 3)
            self.camera.look_at(0, 0, 2)
            return
        elif self.camera_mode == CameraModes.COCKPIT:
            # FIXME: Symbolize
            self.vehicle.np().find("**/fz_window").hide()
            self.camera.reparent_to(self.vehicle.np().find('**/{}'.format(
                COCKPIT_CAMERA,
            )))
            self.camera.set_pos(0, 0, 0)
            self.camera.set_hpr(0, -90, 0)
            self.camera.node().get_lens().set_near(0.1)
            return
        elif self.camera_mode == CameraModes.DIRECTION:
            vehicle_back = -self.vehicle.physics_node.get_linear_velocity()
        elif self.camera_mode == CameraModes.MIXED:
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
        # FIXME: Fudge factor
        kmh /= 2
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

        # Flight height / climb rate
        target_flight_height = self.vehicle.inputs[TARGET_FLIGHT_HEIGHT]
        self.target_flight_height['text'] = "{:2.1f}m target height".format(
            target_flight_height,
        )
        flight_height = self.vehicle.sensors[FLIGHT_HEIGHT]
        climb_rate = self.vehicle.sensors[CLIMB_SPEED]
        repulsor_power_needed = self.vehicle.commands[REPULSOR_POWER_FRACTION_NEEDED]
        if self.vehicle.sensors[LOCAL_UP]:
            self.flight_height['text'] = "{:3.1f}m to ground".format(
                flight_height,
            )
            self.flight_height['fg'] = (0.2, 0.8, 0.2, 1.0)
            self.climb_rate['text'] = "{:3.1f}m/s climb".format(climb_rate)
            self.climb_rate['fg'] = (0.8, 0.8, 0.8, 1.0)
            self.repulsor_power_needed['text'] = "{:3.1f}% repulsor power".format(repulsor_power_needed * 100)
            if 0.0 < repulsor_power_needed <= 1.0:
                self.repulsor_power_needed['fg'] = (0.0, 1.0, 0.0, 1.0)
            elif repulsor_power_needed <= 0.0:
                self.repulsor_power_needed['fg'] = (0.3, 0.3, 0.3, 1.0)
            else: # repulsor_power_needed > 1.0
                self.repulsor_power_needed['fg'] = (1.0, 0.0, 0.0, 1.0)
            self.height_over_target.show()
            self.height_over_target.set_z(
                self.vehicle.commands[HEIGHT_OVER_TARGET] * 0.1,
            )
            self.height_over_target_projected.show()
            self.height_over_target_projected.set_z(
                self.vehicle.commands[HEIGHT_OVER_TARGET_PROJECTED] * 0.01,
            )

        else:
            self.flight_height['text'] = "NO GROUND CONTACT"
            self.flight_height['fg'] = (0.8, 0.2, 0.2, 1.0)
            self.climb_rate['text'] = "--- m/s climb"
            self.climb_rate['fg'] = (0.3, 0.3, 0.3, 1.0)
            self.repulsor_power_needed['text'] = "---% repulsor power"
            self.repulsor_power_needed['fg'] = (0.3, 0.3, 0.3, 1.0)
            self.height_over_target.hide()
            self.height_over_target_projected.hide()
