from panda3d.core import InputDevice
from panda3d.core import VBase3

from keybindings import GE_TOGGLE_REPULSOR
from keybindings import GE_FORWARD
from keybindings import GE_BACKWARD
from keybindings import GE_TURN
from keybindings import GE_TURN_LEFT
from keybindings import GE_TURN_RIGHT
from keybindings import GE_STRAFE
from keybindings import GE_STRAFE_LEFT
from keybindings import GE_STRAFE_RIGHT
from keybindings import GE_HOVER
from keybindings import GE_FULL_REPULSORS
from keybindings import GE_SWITCH_DRIVING_MODE
from keybindings import GE_STABILIZE
from keybindings import GE_GYRO_YAW
from keybindings import GE_GYRO_PITCH
from keybindings import GE_GYRO_PITCH_UP
from keybindings import GE_GYRO_PITCH_DOWN
from keybindings import GE_GYRO_ROLL
from keybindings import GE_GYRO_ROLL_LEFT
from keybindings import GE_GYRO_ROLL_RIGHT
from keybindings import GE_THRUST
from keybindings import GE_AIRBRAKE
from keybindings import GE_STABILIZER_FINS
from keybindings import GE_CAMERA_MODE
from keybindings import GE_NEXT_VEHICLE

from vehicle import REPULSOR_ACTIVATION
from vehicle import ACCELERATE
from vehicle import TURN
from vehicle import STRAFE
from vehicle import HOVER
from vehicle import FULL_REPULSORS
from vehicle import ACTIVE_STABILIZATION_ON_GROUND
from vehicle import ACTIVE_STABILIZATION_CUTOFF_ANGLE
from vehicle import ACTIVE_STABILIZATION_IN_AIR
from vehicle import TO_GROUND
from vehicle import TO_HORIZON
from vehicle import PASSIVE
from vehicle import TARGET_ORIENTATION
from vehicle import THRUST
from vehicle import AIRBRAKE
from vehicle import STABILIZER_FINS
from vehicle import TARGET_FLIGHT_HEIGHT
from vehicle import TARGET_FLIGHT_HEIGHT_TAU


DM_CRUISE = 'dm_cruise'
DM_STUNT = 'dm_stunt'


class VehicleController:
    def __init__(self, app, vehicle, controller):
        self.app = app
        self.vehicle = vehicle
        self.controller = controller
        self.repulsors_active = False
        self.driving_mode = DM_CRUISE
        self.app.accept(GE_NEXT_VEHICLE, self.next_vehicle)
        self.app.accept(GE_TOGGLE_REPULSOR, self.toggle_repulsors)
        self.app.accept(GE_SWITCH_DRIVING_MODE, self.switch_driving_mode)

        # self.app.accept('x', self.shock, [10000, 0, 0])
        # self.app.accept('y', self.shock, [0, 10000, 0])
        # self.app.accept('z', self.shock, [0, 0, 10000])
        # self.app.accept('shift-x', self.shock, [-10000, 0, 0])
        # self.app.accept('shift-y', self.shock, [0, -10000, 0])
        # self.app.accept('shift-z', self.shock, [0, 0, -10000])

    def gather_inputs(self):
        if self.controller.method == InputDevice.DeviceClass.keyboard:
            if self.repulsors_active:
                repulsor_activation = 1
            else:
                repulsor_activation = 0

            repulsor_forward = 0.0
            repulsor_turn = 0.0
            repulsor_strafe = 0.0
            repulsor_hover = 0.0
            target_orientation = VBase3(0, 0, 0)

            if self.controller.is_pressed(GE_FORWARD):
                repulsor_forward += 1.0
            if self.controller.is_pressed(GE_BACKWARD):
                repulsor_forward -= 1.0

            if self.driving_mode == DM_STUNT:
                # Repulsor control
                if self.controller.is_pressed(GE_TURN_LEFT):
                    repulsor_turn += 1.0
                if self.controller.is_pressed(GE_TURN_RIGHT):
                    repulsor_turn -= 1.0
                # In stunt mode, the gyro gives a yaw steering assist to
                # repulsor steering, as those will often lose ground
                # contact,
                gyro_yaw = 0.0
                if self.controller.is_pressed(GE_TURN_LEFT):
                    gyro_yaw += 1.0
                if self.controller.is_pressed(GE_TURN_RIGHT):
                    gyro_yaw -= 1.0
                target_orientation.z += gyro_yaw * 90 * 0.35

                # Gyro control
                stabilizer_active = self.controller.is_pressed(GE_STABILIZE)

                gyro_pitch = 0.0
                gyro_roll = 0.0
                if self.controller.is_pressed(GE_GYRO_PITCH_UP):
                    gyro_pitch += 1.0
                if self.controller.is_pressed(GE_GYRO_PITCH_DOWN):
                    gyro_pitch -= 1.0
                if self.controller.is_pressed(GE_GYRO_ROLL_LEFT):
                    gyro_roll -= 1.0
                if self.controller.is_pressed(GE_GYRO_ROLL_RIGHT):
                    gyro_roll += 1.0
                target_orientation.x += gyro_pitch * 90 * 0.35
                target_orientation.y += gyro_roll * 90 * 0.35

            elif self.driving_mode == DM_CRUISE:
                # Repulsor control
                if self.controller.is_pressed(GE_TURN_LEFT):
                    repulsor_strafe -= 1.0
                if self.controller.is_pressed(GE_TURN_RIGHT):
                    repulsor_strafe += 1.0
                # Gyro control
                gyro_yaw = 0.0
                gyro_pitch = 0.0
                if self.controller.is_pressed(GE_GYRO_ROLL_LEFT):
                    gyro_yaw += 1.0
                if self.controller.is_pressed(GE_GYRO_ROLL_RIGHT):
                    gyro_yaw -= 1.0
                if self.controller.is_pressed(GE_GYRO_PITCH_UP):
                    gyro_pitch += 1.0
                if self.controller.is_pressed(GE_GYRO_PITCH_DOWN):
                    gyro_pitch -= 1.0
                target_orientation.x += gyro_pitch * 90 * 0.35
                target_orientation.z += gyro_yaw * 90 * 0.35

            if self.controller.is_pressed(GE_HOVER):
                repulsor_hover = 1.0

            thrust = 0
            if self.controller.is_pressed(GE_THRUST):
                thrust = 1

            airbrake = 0
            if self.controller.is_pressed(GE_AIRBRAKE):
                airbrake = 1

            stabilizer_fins = 0
            if self.controller.is_pressed(GE_STABILIZER_FINS):
                stabilizer_fins = 1

        elif self.controller.method == InputDevice.DeviceClass.gamepad:
            if self.repulsors_active:
                repulsor_activation = 1
            else:
                repulsor_activation = 0

            repulsor_forward = self.controller.axis_value(GE_FORWARD)
            repulsor_strafe = 0.0
            repulsor_hover = 0.0
            target_orientation = VBase3(0, 0, 0)

            if self.driving_mode == DM_STUNT:
                # Repulsor control
                if self.controller.is_pressed(GE_STRAFE):
                    # Repulsor strafing instead of turning
                    repulsor_strafe = self.controller.axis_value(GE_TURN)
                    repulsor_turn = 0.0
                else:
                    # Regular turning
                    repulsor_turn = self.controller.axis_value(GE_TURN)
                    repulsor_strafe = 0.0
                # Gyro control
                stabilizer_active = self.controller.is_pressed(GE_STABILIZE)
                # In stunt mode, the gyro gives a yaw steering assist to
                # repulsor steering, as those will often lose ground
                # contact,
                gyro_yaw = -self.controller.axis_value(GE_TURN)
                target_orientation.z -= -gyro_yaw * 90 * 0.35
                if not stabilizer_active:
                    gyro_pitch = self.controller.axis_value(GE_GYRO_PITCH)
                    target_orientation.x -= gyro_pitch * 90 * 0.35
                    gyro_roll = self.controller.axis_value(GE_GYRO_ROLL)
                    target_orientation.y += gyro_roll * 90 * 0.35

            elif self.driving_mode == DM_CRUISE:
                if not self.controller.is_pressed(GE_STRAFE):
                    # In cruise mode repulsors strafe by default.
                    repulsor_strafe = self.controller.axis_value(GE_TURN)
                    repulsor_turn = 0.0
                else:
                    # Regular turning, the strafing of cruise mode...
                    repulsor_turn = self.controller.axis_value(GE_TURN)
                    repulsor_strafe = 0.0
                # Gyro control
                # By default, stabilization is active.
                # Instead of rolling, the horizontal gyro control is
                # to change the heading.
                gyro_yaw = self.controller.axis_value(GE_GYRO_ROLL)
                target_orientation.z -= gyro_yaw * 90 * 0.35
                gyro_pitch = self.controller.axis_value(GE_GYRO_PITCH)
                target_orientation.x -= gyro_pitch * 90 * 0.35

            if self.controller.is_pressed(GE_HOVER):
                repulsor_hover = 1.0

            thrust = 0
            if self.controller.is_pressed(GE_THRUST):
                thrust = 1

            airbrake = self.controller.pressed_or_value(GE_AIRBRAKE)
            if airbrake is True:
                airbrake = 1
            elif airbrake is False:
                airbrake = 0

            stabilizer_fins = 0
            if self.controller.is_pressed(GE_STABILIZER_FINS):
                stabilizer_fins = 1

        elif self.controller.method == InputDevice.DeviceClass.flight_stick:
            if self.repulsors_active:
                repulsor_activation = 1
            else:
                repulsor_activation = 0

            repulsor_forward = 0.0
            repulsor_turn = 0.0
            repulsor_strafe = 0.0
            repulsor_hover = 0.0
            target_orientation = VBase3(0, 0, 0)

            repulsor_forward -= self.controller.axis_value(GE_FORWARD)
            repulsor_strafe += self.controller.axis_value(GE_STRAFE)
            gyro_yaw = self.controller.axis_value(GE_TURN)
            gyro_pitch = 0.0
            gyro_roll = 0.0
            if self.controller.is_pressed(GE_GYRO_PITCH_DOWN):
                gyro_pitch -= 1.0
            if self.controller.is_pressed(GE_GYRO_PITCH_UP):
                gyro_pitch += 1.0
            if self.controller.is_pressed(GE_GYRO_ROLL_LEFT):
                gyro_roll -= 1.0
            if self.controller.is_pressed(GE_GYRO_ROLL_RIGHT):
                gyro_roll += 1.0
            target_orientation.x += gyro_pitch * 90 * 0.35
            target_orientation.y += gyro_roll * 90 * 0.35
            target_orientation.z += gyro_yaw * 90 * 0.35

            if self.driving_mode == DM_STUNT:
                repulsor_turn -= self.controller.axis_value(GE_TURN)
            elif self.driving_mode == DM_CRUISE:
                stabilizer_active = not self.controller.is_pressed(GE_STABILIZE)

            thrust = 0.0
            if self.controller.is_pressed(GE_THRUST):
                thrust = 1.0

            airbrake = self.controller.pressed_or_value(GE_AIRBRAKE)

            stabilizer_fins = 0
            if self.controller.is_pressed(GE_STABILIZER_FINS):
                stabilizer_fins = 1

        stabilizer_active = self.controller.is_pressed(GE_STABILIZE)
        if self.driving_mode == DM_CRUISE:
            if not stabilizer_active:
                active_stabilization_on_ground = TO_GROUND
                active_stabilization_cutoff_angle = 20
                active_stabilization_in_air = TO_HORIZON
            else:
                active_stabilization_on_ground = TO_HORIZON
                active_stabilization_cutoff_angle = -1
                active_stabilization_in_air = PASSIVE
        elif self.driving_mode == DM_STUNT:
            if not stabilizer_active:
                active_stabilization_on_ground = TO_GROUND
                active_stabilization_cutoff_angle = 20
                active_stabilization_in_air = PASSIVE
            else:
                active_stabilization_on_ground = TO_HORIZON
                active_stabilization_cutoff_angle = -1
                active_stabilization_in_air = TO_HORIZON

        # Repulsor damping
        if self.driving_mode == DM_CRUISE:
            target_flight_height = 3.5
            target_flight_height_tau = 0.15
        elif self.driving_mode == DM_STUNT:
            target_flight_height = 3.0
            target_flight_height_tau = 0.1

        full_repulsors = False
        if self.controller.is_pressed(GE_FULL_REPULSORS):
            full_repulsors = True

        self.vehicle.set_inputs(
            {
                # Repulsors
                REPULSOR_ACTIVATION: repulsor_activation,
                ACCELERATE: repulsor_forward,
                TURN: repulsor_turn,
                STRAFE: repulsor_strafe,
                HOVER: repulsor_hover,
                FULL_REPULSORS: full_repulsors,
                # Repulsor damping
                TARGET_FLIGHT_HEIGHT: target_flight_height,
                TARGET_FLIGHT_HEIGHT_TAU: target_flight_height_tau,
                # Gyro
                ACTIVE_STABILIZATION_ON_GROUND: active_stabilization_on_ground,
                ACTIVE_STABILIZATION_CUTOFF_ANGLE: active_stabilization_cutoff_angle,
                ACTIVE_STABILIZATION_IN_AIR: active_stabilization_in_air,
                TARGET_ORIENTATION: target_orientation,
                # Thrust
                THRUST: thrust,
                # Airbrake
                AIRBRAKE: airbrake,
                STABILIZER_FINS: stabilizer_fins,
            }
        )

    def next_vehicle(self):
        self.app.next_vehicle()

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def toggle_repulsors(self):
        self.repulsors_active = not self.repulsors_active

    def switch_driving_mode(self):
        if self.driving_mode == DM_CRUISE:
            self.driving_mode = DM_STUNT
        elif self.driving_mode == DM_STUNT:
            self.driving_mode = DM_CRUISE

    def shock(self, x=0, y=0, z=0):
        self.vehicle.shock(x, y, z)
