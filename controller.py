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
from keybindings import GE_SWITCH_DRIVING_MODE
from keybindings import GE_STABILIZE
from keybindings import GE_GYRO_PITCH
from keybindings import GE_GYRO_ROLL
from keybindings import GE_THRUST
from keybindings import GE_AIRBRAKE
from keybindings import GE_CAMERA_MODE
from keybindings import GE_NEXT_VEHICLE

from vehicle import REPULSOR_ACTIVATION
from vehicle import ACCELERATE
from vehicle import TURN
from vehicle import STRAFE
from vehicle import HOVER
from vehicle import ACTIVE_STABILIZATION
from vehicle import TARGET_ORIENTATION
from vehicle import THRUST


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

        self.app.accept('x', self.shock, [10000, 0, 0])
        self.app.accept('y', self.shock, [0, 10000, 0])
        self.app.accept('z', self.shock, [0, 0, 10000])
        self.app.accept('shift-x', self.shock, [-10000, 0, 0])
        self.app.accept('shift-y', self.shock, [0, -10000, 0])
        self.app.accept('shift-z', self.shock, [0, 0, -10000])

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

            if self.controller.is_pressed(GE_FORWARD):
                repulsor_forward += 1.0
            if self.controller.is_pressed(GE_BACKWARD):
                repulsor_forward -= 1.0
            if self.controller.is_pressed(GE_TURN_LEFT):
                repulsor_turn -= 1
            if self.controller.is_pressed(GE_TURN_RIGHT):
                repulsor_turn += 1
            if self.controller.is_pressed(GE_STRAFE_LEFT):
                repulsor_strafe -= 1
            if self.controller.is_pressed(GE_STRAFE_RIGHT):
                repulsor_strafe += 1
            if self.controller.is_pressed(GE_HOVER):
                repulsor_hover += 1

            stabilizer_active = self.controller.is_pressed(GE_STABILIZE)
            target_orientation = VBase3(0, 0, 0)
            if self.controller.is_pressed(GE_TURN_LEFT):
                target_orientation.z += 90 * 0.35
            if self.controller.is_pressed(GE_TURN_RIGHT):
                target_orientation.z -= 90 * 0.35

            thrust = 0
            if self.controller.is_pressed(GE_THRUST):
                thrust = 1

        if self.controller.method == InputDevice.DeviceClass.gamepad:
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
                # FIXME: Axes are swapped and flipped because Jess produces
                # weird gamepads, so GE_GYRO_PITCH is GE_GYRO_ROLL and vice
                # versa, and 1.0 means -1.0 and vice versa.
                gyro_yaw = -self.controller.axis_value(GE_TURN)
                target_orientation.z -= -gyro_yaw * 90 * 0.35
                if not stabilizer_active:
                    # FIXME: Axes are swapped and flipped, see above.
                    gyro_pitch = self.controller.axis_value(GE_GYRO_ROLL)
                    target_orientation.x += gyro_pitch * 90 * 0.35
                    gyro_roll = self.controller.axis_value(GE_GYRO_PITCH)
                    target_orientation.y += -gyro_roll * 90 * 0.35

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
                stabilizer_active = not self.controller.is_pressed(GE_STABILIZE)
                # Instead of rolling, the horizontal gyro control is
                # to change the heading.
                # FIXME: Axes are swapped and flipped, see above.
                gyro_yaw = self.controller.axis_value(GE_GYRO_PITCH)
                target_orientation.z += gyro_yaw * 90 * 0.35
                #if not stabilizer_active:
                gyro_pitch = self.controller.axis_value(GE_GYRO_ROLL)
                target_orientation.x += gyro_pitch * 90 * 0.35

            if self.controller.is_pressed(GE_HOVER):
                repulsor_hover = 1.0

            thrust = 0
            if self.controller.is_pressed(GE_THRUST):
                thrust = 1

        self.vehicle.set_inputs(
            {
                # Repulsors
                REPULSOR_ACTIVATION: repulsor_activation,
                ACCELERATE: repulsor_forward,
                TURN: repulsor_turn,
                STRAFE: repulsor_strafe,
                HOVER: repulsor_hover,
                # Gyro
                ACTIVE_STABILIZATION: stabilizer_active,
                TARGET_ORIENTATION: target_orientation,
                # Thrust
                THRUST: thrust,
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
