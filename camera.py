from enum import Enum

from panda3d.core import TextNode
from panda3d.core import NodePath
from panda3d.core import Vec3
from panda3d.core import VBase3
from panda3d.core import Point3
from panda3d.core import ConfigVariableDouble

from direct.showbase.DirectObject import DirectObject
from direct.gui.OnscreenText import OnscreenText
from direct.gui.DirectGui import DirectWaitBar

from vehicle import REPULSOR_DATA
from vehicle import LOCAL_UP
from vehicle import FLIGHT_HEIGHT
from vehicle import TARGET_FLIGHT_HEIGHT
from vehicle import CLIMB_SPEED
from vehicle import HEIGHT_OVER_TARGET
from vehicle import HEIGHT_OVER_TARGET_PROJECTED
from vehicle import REPULSOR_POWER_FRACTION_NEEDED
from vehicle import GYRO_ROTATION
from vehicle import THRUSTER_HEATING
from vehicle import THRUSTER_COOLING

from keybindings import GE_CAMERA_MODE

from controller import DM_STUNT
from controller import DM_CRUISE


class CameraController(DirectObject):
    def init(self, camera, vehicle, control):
        self.vehicle = vehicle
        self.control = control
        self.camera = camera

        # Height meters and repulsor self-control
        #self.heights = base.render2d.attach_new_node("height meters")
        #self.heights.set_pos(-0.8, 0.0, -0.6)
       #
        #self.height_null = base.loader.load_model('models/box')
        #self.height_null.set_scale(0.3, 0.3, 0.01)
        #self.height_null.set_pos(-0.15, -0.15, -0.005)
        #self.height_null.reparent_to(self.heights)
       #
        #self.height_over_target = base.loader.load_model('models/smiley')
        #self.height_over_target.set_scale(0.01, 0.1, 0.1)
        #self.height_over_target.set_pos(-0.1, 0, 0)
        #self.height_over_target.reparent_to(self.heights)
        #self.height_over_target.hide()
       #
        #self.height_over_target_projected = base.loader.load_model('models/smiley')
        #self.height_over_target_projected.set_scale(0.01, 0.1, 0.1)
        #self.height_over_target_projected.set_pos(-0.05, 0, 0)
        #self.height_over_target_projected.reparent_to(self.heights)
        #self.height_over_target_projected.hide()
       #
        #self.repulsor_hud = self.camera.attach_new_node('repulsor HUD')
        #self.repulsor_hud.set_pos(-2, 10, 2)
        #self.repulsor_hud.set_p(90)
        #self.repulsor_models = []

    def set_camera_mode(self, mode):
        self.camera_mode = mode
        if mode == CameraModes.FOLLOW:
            self.camera_gimbal.reparent_to(self.vehicle.np())
            self.camera_gimbal.set_pos_hpr((0, 0, 0), (0, 0, 0))
            # Position, focus, and FOV are set frame by frame.
        elif mode == CameraModes.FIXED:
            self.camera_gimbal.set_pos_hpr((0, 0, 0), (0, 0, 0))
            self.camera_gimbal.reparent_to(self.vehicle.np())
            self.camera.set_pos(0, -10, 3)
            self.camera.look_at(0, 0, 2)
            self.camera.node().get_lens().set_fov(third_person_fov)
        elif mode == CameraModes.COCKPIT:
            self.camera_gimbal.reparent_to(
                self.vehicle.np().find('**/{}'.format(COCKPIT_CAMERA)),
            )
            self.camera_gimbal.set_pos_hpr((0, 0, 0), (0, 0, 0))
            self.camera.set_pos_hpr((0, 0, 0), (0, 0, 0))
            self.camera.node().get_lens().set_fov(first_person_fov)

    def switch_camera_mode(self):
        new_mode_idx = ((self.camera_mode.value) % len(CameraModes)) + 1
        new_mode = CameraModes(new_mode_idx)
        self.set_camera_mode(new_mode)

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle
        self.set_camera_mode(self.camera_mode)

    def update(self):
        pass
    #def update_gui(self):
        # # Repulsors
        # ray_data = self.vehicle.sensors[REPULSOR_DATA]
        # num_models_delta = len(ray_data) - len(self.repulsor_models)
        # if num_models_delta > 0:
        #     # We need more models
        #     for _ in range(num_models_delta):
        #         on_model = base.loader.load_model('models/smiley')
        #         on_model.reparent_to(self.repulsor_hud)
        #         on_model.set_scale(0.1)
        #         on_model.set_p(-90)
        #         on_model.hide()
        #         off_model = base.loader.load_model('models/frowney')
        #         off_model.reparent_to(self.repulsor_hud)
        #         off_model.set_scale(0.1)
        #         off_model.set_p(-90)
        #         off_model.hide()
        #         self.repulsor_models.append((on_model, off_model))
        # elif num_models_delta < 0:
        #     for on_model, off_model in self.repulsor_models[num_models_delta:]:
        #         on_model.remove_node()
        #         off_model.remove_node()
        #     self.repulsor_models = self.repulsor_models[:len(ray_dirs)]
        # for data, (on_model, off_model) in zip(ray_data, self.repulsor_models):
        #     offset_dir = self.vehicle.np().get_relative_vector(
        #         base.render,
        #         data.direction,
        #     )
        #     offset = data.position * 0.1 + offset_dir * data.fraction * 0.1
        #     if data.active:
        #         on_model.set_pos(offset)
        #         on_model.show()
        #         off_model.hide()
        #     else:
        #         off_model.set_pos(offset)
        #         off_model.show()
        #         on_model.hide()
       # 
        # # Flight height / climb rate
        # target_flight_height = self.vehicle.inputs[TARGET_FLIGHT_HEIGHT]
        # self.target_flight_height['text'] = "{:2.1f}m target height".format(
        #     target_flight_height,
        # )
        # flight_height = self.vehicle.sensors[FLIGHT_HEIGHT]
        # climb_rate = self.vehicle.sensors[CLIMB_SPEED]
        # repulsor_power_needed = self.vehicle.commands[REPULSOR_POWER_FRACTION_NEEDED]
        # if self.vehicle.sensors[LOCAL_UP]:
        #     self.flight_height['text'] = "{:3.1f}m to ground".format(
        #         flight_height,
        #     )
        #     self.flight_height['fg'] = (0.2, 0.8, 0.2, 1.0)
        #     self.climb_rate['text'] = "{:3.2f}m/s climb".format(climb_rate)
        #     self.climb_rate['fg'] = (0.8, 0.8, 0.8, 1.0)
        #     clamped_power = min(max(repulsor_power_needed, 0), 1)
        #     self.repulsor_power_needed['value'] = clamped_power * 100
        #     power_needed_str = "{:3.1f}% repulsor power".format(
        #         repulsor_power_needed * 100,
        #     )
        #     self.repulsor_power_needed['text'] = power_needed_str
        #     if 0.0 < repulsor_power_needed <= 1.0:
        #         self.repulsor_power_needed['barColor'] = color_gradient(
        #             repulsor_power_needed,
        #         )
        #     elif repulsor_power_needed <= 0.0:
        #         self.repulsor_power_needed['barColor'] = (0.3, 0.3, 0.3, 1.0)
        #     else: # repulsor_power_needed > 1.0
        #         self.repulsor_power_needed['barColor'] = (1.0, 0.0, 0.0, 1.0)
        #     self.height_over_target.show()
        #     self.height_over_target.set_z(
        #         self.vehicle.commands[HEIGHT_OVER_TARGET] * 0.1,
        #     )
        #     self.height_over_target_projected.show()
        #     self.height_over_target_projected.set_z(
        #         self.vehicle.commands[HEIGHT_OVER_TARGET_PROJECTED] * 0.01,
        #     )
       # 
        # else:
        #     self.flight_height['text'] = "NO GROUND CONTACT"
        #     self.flight_height['fg'] = (0.8, 0.2, 0.2, 1.0)
        #     self.climb_rate['text'] = "--- m/s climb"
        #     self.climb_rate['fg'] = (0.3, 0.3, 0.3, 1.0)
        #     self.repulsor_power_needed['value'] = 0.0
        #     self.repulsor_power_needed['text'] = "---% repulsor power"
        #     self.repulsor_power_needed['barColor'] = (0.3, 0.3, 0.3, 1.0)
        #     self.height_over_target.hide()
        #     self.height_over_target_projected.hide()
