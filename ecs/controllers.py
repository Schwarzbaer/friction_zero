from dataclasses import field
from enum import Enum

from panda3d.core import ConfigVariableDouble
from panda3d.core import NodePath
from panda3d.core import Vec3
from panda3d.core import TextNode

from direct.gui.OnscreenText import OnscreenText
from direct.gui.DirectGui import DirectWaitBar

from wecs.core import Component
from wecs.core import System
from wecs.core import and_filter
from wecs.panda3d import DirectSystem
from wecs.panda3d import Model
from wecs.panda3d import PhysicsBody

from keybindings import DeviceListener
from keybindings import GE_CAMERA_MODE

from controller import DM_STUNT
from controller import DM_CRUISE

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


@Component()
class FZVehicle:
    pyobj: object


@Component()
class InputControllerECS:
    listener: DeviceListener = field(default_factory=DeviceListener)


@Component()
class VehicleControllerECS:
    pyobj: object


class GatherInputs(System):
    entity_filters = {
        'vehicle': and_filter([
            FZVehicle,
            InputControllerECS,
            VehicleControllerECS,
        ]),
    }

    def init_entity(self, filter_name, entity):
        vehicle = entity[FZVehicle].pyobj
        controller = entity[InputControllerECS].listener
        entity[VehicleControllerECS].pyobj.init(vehicle, controller)

    def update(self, entities_by_filter):
        for entity in entities_by_filter['vehicle']:
            entity[VehicleControllerECS].pyobj.gather_inputs()


class UpdateVehicles(System):
    entity_filters = {
        'vehicle': and_filter([
            VehicleControllerECS,
        ]),
    }

    def update(self, entities_by_filter):
        for entity in entities_by_filter['vehicle']:
            pyobj = entity[FZVehicle].pyobj
            pyobj.gather_sensors(entity)
            pyobj.ecu()
            pyobj.apply_air_drag()
            pyobj.apply_repulsors()
            pyobj.apply_gyroscope()
            pyobj.apply_thrusters()
            pyobj.apply_airbrake()
            pyobj.apply_stabilizer_fins()


# Camera

COCKPIT_CAMERA = 'fz_cockpit_camera'
first_person_fov = ConfigVariableDouble('first_person_fov', 90).value
third_person_fov = ConfigVariableDouble('third_person_fov', 60).value


def color_gradient(v):
    red = v*2
    if red > 1.0:
        red = 1.0
    green = 2 - v*2
    if green > 1.0:
        green = 1.0
    blue = 0.0
    alpha = 1.0
    return (red, green, blue, alpha)


class CameraModes(Enum):
    FOLLOW = 1
    FIXED = 2
    COCKPIT = 3


@Component()
class CameraControllerECS:
    pyobj: object
    camera: object # FIXME: Specific type
    mode: int = CameraModes.FOLLOW
    gui_nodes: dict = field(default_factory=dict)


class UpdateCamera(DirectSystem):
    entity_filters = {
        'vehicle': and_filter([
            FZVehicle,
            InputControllerECS,
            VehicleControllerECS,
            CameraControllerECS,
        ]),
    }

    def __init__(self, *args, **kwargs):
        super().__init__(self, *args, **kwargs)
        self._switch_mode = False
        self.accept(GE_CAMERA_MODE, self.switch_camera_mode)

    def switch_camera_mode(self):
        self._switch_mode = True

    def init_entity(self, filter_name, entity):
        camera = entity[CameraControllerECS].camera
        vehicle = entity[FZVehicle].pyobj
        control = entity[VehicleControllerECS].pyobj
        camera.node().get_lens().set_near(0.1)
        # camera_gimbal = NodePath("camera gimbal")
        # camera.reparent_to(camera_gimbal)
        entity[CameraControllerECS].pyobj.init(camera, vehicle, control)
        entity[CameraControllerECS].pyobj.camera_gimbal = NodePath("camera gimbal")
        camera.reparent_to(entity[CameraControllerECS].pyobj.camera_gimbal)
        self.set_camera_mode(CameraModes.FOLLOW, entity)

        self.setup_gui(entity)

    def set_camera_mode(self, mode, entity):
        self.camera_mode = mode
        vehicle = entity[Model].node
        camera = entity[CameraControllerECS].camera
        gimbal = entity[CameraControllerECS].pyobj.camera_gimbal
        if mode == CameraModes.FOLLOW:
            gimbal.reparent_to(vehicle)
            gimbal.set_pos_hpr((0, 0, 0), (0, 0, 0))
            # Position, focus, and FOV are set frame by frame.
        elif mode == CameraModes.FIXED:
            gimbal.set_pos_hpr((0, 0, 0), (0, 0, 0))
            gimbal.reparent_to(vehicle)
            camera.set_pos(0, -10, 3)
            camera.look_at(0, 0, 2)
            camera.node().get_lens().set_fov(third_person_fov)
        elif mode == CameraModes.COCKPIT:
            gimbal.reparent_to(
                vehicle.find('**/{}'.format(COCKPIT_CAMERA)),
            )
            gimbal.set_pos_hpr((0, 0, 0), (0, 0, 0))
            camera.set_pos_hpr((0, 0, 0), (0, 0, 0))
            camera.node().get_lens().set_fov(first_person_fov)

    def setup_gui(self, entity):
        gui = entity[CameraControllerECS].gui_nodes
        gui['speed'] = OnscreenText(
            text = '',
            pos = (1.3, 0.8),
            scale = 0.15,
            fg = (1.0, 1.0, 0.5, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )
        gui['driving_mode'] = OnscreenText(
            text = '',
            pos = (1.3, 0.7),
            scale = 0.1,
            fg = (0.0, 0.0, 0.0, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )
        gui['thruster_heat'] = DirectWaitBar(
            text = "",
            value = 0,
            pos = (0.9, 0, 0.65),
            scale = 0.4,
        )

        gui['flight_height'] = OnscreenText(
            text = '',
            pos = (1.3, -0.6),
            scale = 0.1,
            fg = (0.0, 0.0, 0.0, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )
        gui['target_flight_height'] = OnscreenText(
            text = '',
            pos = (1.3, -0.7),
            scale = 0.1,
            fg = (0.2, 0.8, 0.2, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )
        gui['climb_rate'] = OnscreenText(
            text = '',
            pos = (1.3, -0.8),
            scale = 0.1,
            fg = (0.0, 0.0, 0.0, 1.0),
            shadow = (0.2, 0.2, 0.2, 1.0),
            align = TextNode.ARight,
        )
        gui['repulsor_power_needed'] = DirectWaitBar(
            text = "",
            value = 0,
            pos = (0.9, 0, -0.87),
            scale = 0.4,
        )
        gui['gyro_power_needed'] = DirectWaitBar(
            text = "",
            value = 0,
            pos = (0.9, 0, -0.94),
            scale = 0.4,
        )

    def update(self, entities_by_filter):
        for entity in entities_by_filter['vehicle']:
            if self._switch_mode:
                old_mode_idx = entity[CameraControllerECS].mode.value
                new_mode_idx = (old_mode_idx % len(CameraModes)) + 1
                entity[CameraControllerECS].mode = CameraModes(new_mode_idx)
                self.set_camera_mode(entity[CameraControllerECS].mode, entity)
            self.update_camera(entity)
            self.update_gui(entity)
        self._switch_mode = False

    def update_camera(self, entity):
        # entity[CameraControllerECS].pyobj.update()
        camera_mode = entity[CameraControllerECS].mode
        if camera_mode == CameraModes.FOLLOW:
            model = entity[Model].node
            # The numbers that we're gonna build our camera offset from
            forward = Vec3(0, 1, 0)
            forward_global = base.render.get_relative_vector(
                #entity[FZVehicle].pyobj.np(),
                model,
                forward,
            )
            forward_to_horizon_global = Vec3(forward_global)
            forward_to_horizon_global.z = 0
            forward_to_horizon_global.normalize()
            forward_to_horizon = model.get_relative_vector(
                base.render,
                forward_to_horizon_global,
            )
            up = model.get_relative_vector(
                base.render,
                Vec3(0, 0, 1),
            )
            movement = model.get_relative_vector(
                base.render,
                entity[PhysicsBody].body.get_linear_velocity(),
            )
            movement_dir = Vec3(movement)
            movement_dir.normalize()
            ground_movement_global = entity[PhysicsBody].body.get_linear_velocity()
            ground_movement_global.z = 0
            ground_movement = model.get_relative_vector(
                base.render,
                ground_movement_global,
            )
            ground_speed = ground_movement.length()
            ground_dir = Vec3(ground_movement)
            ground_dir.normalize()
            ground_movement_angle = ground_dir.angle_deg(forward_to_horizon)

            speed_mps = entity[PhysicsBody].body.get_linear_velocity().length()
            speed_kmh = speed_mps * 60 * 60 / 1000
            speed_base_factor = speed_kmh / 1200
            speed_factor = 1 + speed_base_factor

            # Now for the position of the camera and the focal point
            cam_pos = (forward_to_horizon * -15 + up * 5) * speed_factor
            cam_up = up
            focal = up * 3
            fov = third_person_fov * speed_factor

            # ...and applying it.
            camera = entity[CameraControllerECS].camera
            camera.set_pos(cam_pos)
            camera.look_at(focal, cam_up)
            entity[CameraControllerECS].pyobj.camera_gimbal.set_h(
                model,
                ground_movement_angle * speed_base_factor,
            )
            camera.node().get_lens().set_fov(fov)

    def update_gui(self, entity):
        gui = entity[CameraControllerECS].gui_nodes
        vehicle = entity[FZVehicle].pyobj

        # Speed
        mps = entity[PhysicsBody].body.get_linear_velocity().length()
        kmh = mps * 60 * 60 / 1000
        # FIXME: Fudge factor
        kmh /= 2
        gui['speed']['text'] = "{:03.1f} km/h".format(kmh)

        # Driving mode
        mode = entity[CameraControllerECS].mode
        if mode == DM_STUNT:
            gui['driving_mode']['text'] = 'STUNT MODE!'
            gui['driving_mode']['fg'] = (1.0, 0.0, 0.0, 1.0)
        elif mode == DM_CRUISE:
            gui['driving_mode']['text'] = 'Cruise mode'
            gui['driving_mode']['fg'] = (0.5, 1.0, 0.5, 1.0)

        # Gyro power usage
        gyro_power_needed = vehicle.commands[GYRO_ROTATION].length()
        gyro_power_max = vehicle.vehicle_data.max_gyro_torque
        gyro_frac_needed = gyro_power_needed / gyro_power_max
        clamped_frac = min(max(gyro_frac_needed, 0), 1)
        gui['gyro_power_needed']['value'] = clamped_frac * 100
        power_needed_str = "{:3.1f}% gyroscope power".format(
            gyro_frac_needed * 100,
        )
        gui['gyro_power_needed']['text'] = power_needed_str
        if 0.0 < gyro_frac_needed <= 1.0:
            gui['gyro_power_needed']['barColor'] = color_gradient(
                gyro_frac_needed,
            )
        elif gyro_frac_needed <= 0.0:
            gui['gyro_power_needed']['barColor'] = (0.3, 0.3, 0.3, 1.0)
        else: # repulsor_power_needed > 1.0
            gui['gyro_power_needed']['barColor'] = (1.0, 0.0, 0.0, 1.0)

        thruster_heat = vehicle.thruster_heat
        gui['thruster_heat']['value'] = thruster_heat * 100

        if thruster_heat <= 1.0:
            gui['thruster_heat']['barColor'] = color_gradient(thruster_heat)
        else:
            gui['thruster_heat']['barColor'] = (1.0, 1.0, 1.0, 1.0)
        thruster_heat_str = "{:3.1f}% thruster heat".format(
            vehicle.thruster_heat * 100,
        )
        gui['thruster_heat']['text'] = thruster_heat_str
