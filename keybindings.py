import sys
from enum import Enum

from panda3d.core import load_prc_file
from panda3d.core import Filename
from panda3d.core import InputDevice
from panda3d.core import ConfigVariableString
from panda3d.core import KeyboardButton
from panda3d.core import ButtonRegistry

from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject


load_prc_file(
    Filename.expand_from('$MAIN_DIR/keybindings.prc')
)


class Methods(Enum):
    FLIGHTSTICK = InputDevice.DeviceClass.flight_stick
    GAMEPAD = InputDevice.DeviceClass.gamepad
    KEYBOARD = InputDevice.DeviceClass.keyboard


priorities = {cls.value: priority for priority, cls in enumerate(Methods)}


event_prefixes = {
    InputDevice.DeviceClass.keyboard: '',
    InputDevice.DeviceClass.gamepad: 'gamepad',
    InputDevice.DeviceClass.flight_stick: 'flight_stick',
}


UNBOUND = 'none'
GE_SWITCH_DRIVING_MODE = 'switch_driving_mode'
GE_TOGGLE_REPULSOR = 'toggle_repulsor'
GE_FULL_REPULSORS = 'full_repulsors'
GE_FORWARD = 'forward'
GE_BACKWARD = 'backward'
GE_TURN = 'turn'
GE_TURN_LEFT = 'turn_left'
GE_TURN_RIGHT = 'turn_right'
GE_STRAFE = 'strafe'
GE_STRAFE_LEFT = 'strafe_left'
GE_STRAFE_RIGHT = 'strafe_right'
GE_HOVER = 'hover'
GE_TARGET_HEIGHT_UP = 'target_height_up'
GE_TARGET_HEIGHT_DOWN = 'target_height_down'
GE_STABILIZE = 'stabilize'
GE_GYRO_YAW = 'gyro_yaw'
GE_GYRO_PITCH = 'gyro_pitch'
GE_GYRO_PITCH_UP = 'gyro_pitch_up'
GE_GYRO_PITCH_DOWN = 'gyro_pitch_down'
GE_GYRO_ROLL = 'gyro_roll'
GE_GYRO_ROLL_LEFT = 'gyro_roll_left'
GE_GYRO_ROLL_RIGHT = 'gyro_roll_right'
GE_THRUST = 'thrust'
GE_AIRBRAKE = 'airbrake'
GE_STABILIZER_FINS = 'stabilizer_fins'
GE_CAMERA_MODE = 'camera_mode'
GE_NEXT_VEHICLE = 'next_vehicle'


keyboard_bindings = {
    GE_TOGGLE_REPULSOR: ConfigVariableString('keyboard_toggle_repulsor', 'r'),
    GE_FORWARD: ConfigVariableString('keyboard_forward', 'w'),
    GE_BACKWARD: ConfigVariableString('keyboard_backward', 's'),
    GE_STRAFE: ConfigVariableString('keyboard_strafe', 'lshift'),
    GE_TURN_LEFT: ConfigVariableString('keyboard_turn_left', 'a'),
    GE_TURN_RIGHT: ConfigVariableString('keyboard_turn_right', 'd'),
    GE_HOVER: ConfigVariableString('keyboard_hover', 'none'),
    GE_FULL_REPULSORS: ConfigVariableString('keyboard_full_repulsors', 'e'),
    GE_SWITCH_DRIVING_MODE: ConfigVariableString('keyboard_switch_driving_mode', 'q'),
    GE_TARGET_HEIGHT_UP: ConfigVariableString('keyboard_target_height_up', 'f'),
    GE_TARGET_HEIGHT_DOWN: ConfigVariableString('keyboard_target_height_down', 'v'),
    GE_STABILIZE: ConfigVariableString('keyboard_stabilize', 'none'),
    GE_GYRO_PITCH_DOWN: ConfigVariableString('keyboard_gyro_pitch_down', 'arrow_up'),
    GE_GYRO_PITCH_UP: ConfigVariableString('keyboard_gyro_pitch_up', 'arrow_down'),
    GE_GYRO_ROLL_LEFT: ConfigVariableString('keyboard_gyro_roll_left', 'arrow_left'),
    GE_GYRO_ROLL_RIGHT: ConfigVariableString('keyboard_gyro_roll_right', 'arrow_right'),
    GE_THRUST: ConfigVariableString('keyboard_thrust', 'space'),
    GE_AIRBRAKE: ConfigVariableString('keyboard_airbrake', 'tab'),
    GE_STABILIZER_FINS: ConfigVariableString('keyboard_stabilizer_fins', 'none'),
    GE_CAMERA_MODE: ConfigVariableString('keyboard_camera_mode', 'c'),
    GE_NEXT_VEHICLE: ConfigVariableString('keyboard_next_vehicle', 'n'),
}


gamepad_bindings = {
    GE_TOGGLE_REPULSOR: ConfigVariableString('gamepad_repulsor_on', 'face_b'),
    GE_FORWARD: ConfigVariableString('gamepad_forward', 'left_y'),
    GE_TURN: ConfigVariableString('gamepad_turn', 'left_x'),
    GE_STRAFE: ConfigVariableString('gamepad_strafe', 'lstick'),
    GE_HOVER: ConfigVariableString('gamepad_hover', 'none'),
    GE_SWITCH_DRIVING_MODE: ConfigVariableString('gamepad_switch_driving_mode', 'face_a'),
    GE_TARGET_HEIGHT_UP: ConfigVariableString('gamepad_target_height_up', 'face_y'),
    GE_TARGET_HEIGHT_DOWN: ConfigVariableString('gamepad_target_height_down', 'face_x'),
    GE_STABILIZE: ConfigVariableString('gamepad_stabilize', 'rstick'),
    GE_GYRO_YAW: ConfigVariableString('gamepad_gyro_yaw', 'none'),
    GE_GYRO_PITCH: ConfigVariableString('gamepad_gyro_pitch', 'right_y'),
    GE_GYRO_ROLL: ConfigVariableString('gamepad_gyro_roll', 'right_x'),
    GE_FULL_REPULSORS: ConfigVariableString('gamepad_full_repulsors', 'lshoulder'),
    GE_THRUST: ConfigVariableString('gamepad_thrust', 'ltrigger'),
    GE_STABILIZER_FINS: ConfigVariableString('gamepad_stabilizers', 'rshoulder'),
    GE_AIRBRAKE: ConfigVariableString('gamepad_airbrake', 'rtrigger'),
    GE_CAMERA_MODE: ConfigVariableString('gamepad_camera_mode', 'dpad_down'),
    GE_NEXT_VEHICLE: ConfigVariableString('gamepad_next_vehicle', 'dpad_up'),
}


flight_stick_bindings = {
    GE_TOGGLE_REPULSOR: ConfigVariableString('flight_stick_toggle_repulsor', 'joystick2'),
    GE_STABILIZE: ConfigVariableString('flight_stick_stabilize', 'joystick4'),
    GE_FORWARD: ConfigVariableString('flight_stick_forward', 'pitch'),
    GE_TURN: ConfigVariableString('flight_stick_turn', 'yaw'),
    GE_STRAFE: ConfigVariableString('flight_stick_strafe', 'roll'),
    GE_GYRO_PITCH_UP: ConfigVariableString('flight_stick_gyro_pitch_up', 'hat_down'),
    GE_GYRO_PITCH_DOWN: ConfigVariableString('flight_stick_gyro_pitch_down', 'hat_up'),
    GE_GYRO_ROLL_LEFT: ConfigVariableString('flight_stick_gyro_roll_left', 'hat_left'),
    GE_GYRO_ROLL_RIGHT: ConfigVariableString('flight_stick_gyro_roll_right', 'hat_right'),
    GE_SWITCH_DRIVING_MODE: ConfigVariableString('flight_stick_switch_driving_mode', 'joystick3'),
    GE_TARGET_HEIGHT_UP: ConfigVariableString('flight_stick_target_height_up', 'none'),
    GE_TARGET_HEIGHT_DOWN: ConfigVariableString('flight_stick_target_height_down', 'none'),
    GE_THRUST: ConfigVariableString('flight_stick_thrust', 'trigger'),
    GE_AIRBRAKE: ConfigVariableString('flight_stick_airbrake', 'joystick7'),
}


device_bindings = {
    InputDevice.DeviceClass.gamepad: gamepad_bindings,
    InputDevice.DeviceClass.flight_stick: flight_stick_bindings,
    InputDevice.DeviceClass.keyboard: keyboard_bindings,
}


class DeviceListener(DirectObject):
    def __init__(self):
        self.controller = None
        self.method = None
        self.bindings = {}
        self.elect_control_method()
        self.accept("connect-device", self.connect)
        self.accept("disconnect-device", self.disconnect)

    def connect(self, device):
        """Event handler that is called when a device is discovered."""

        print("{} found".format(device.device_class.name))
        self.elect_control_method()

    def disconnect(self, device):
        """Event handler that is called when a device is removed."""

        print("{} disconnected".format(device.device_class.name))
        if device == self.controller:
            self.controller = None
            self.elect_control_method()

    def elect_control_method(self):
        for method in Methods:
            devices = base.devices.get_devices(method.value)
            if devices:
                device = devices[0]
                self.set_controller(device)
                break
            # Keyboards are not usually mounted as user-readable HID
            # devices for security reasons, as running a keystroke
            # sniffer would be trivial. So we have to just assume that
            # one is conected, and use None as the device handle.
            if method == Methods.KEYBOARD:
                self.set_controller(None)
                break
        else:
            # No usable controllers were found, so... Let's just wait?
            print("No controllers found.")

    def set_controller(self, device):
        if device is not None:
            # It's not the keyboard
            # But we only accept this device as the new controller if it's of a
            # higher-prioritized class than the current controller. If it's the
            # same or lower-prioritized one, we want to keep the current one.
            if device is None:
                new_device_class = InputDevice.DeviceClass.keyboard
            else:
                new_device_class = device.device_class
            if self.controller is None:
                old_device_class = InputDevice.DeviceClass.keyboard
            else:
                old_device_class = self.controller.device_class
            if priorities[new_device_class] < priorities[old_device_class]:
                print("{} elected".format(device.device_class.name))
                if self.controller is not None:
                    base.detach_input_device(self.controller)
                # Attach new controller
                self.controller = device
                event_prefix = event_prefixes[new_device_class]
                base.attach_input_device(device, prefix=event_prefix)
                self.map_bindings(new_device_class)
        else:
            # It's the keybard
            print("Assuming keyboard")
            self.map_bindings(InputDevice.DeviceClass.keyboard)

    def map_bindings(self, device_class):
        # Ignore old bindings
        for game_event, control_event in self.bindings.items():
            if control_event != UNBOUND:
                self.ignore(control_event)
        self.bindings = {}
        # Listen for new bindings
        bindings = device_bindings[device_class]
        event_prefix = event_prefixes[device_class]
        for game_event, control_event in bindings.items():
            if control_event != UNBOUND:
                if self.controller is None:
                    full_event_name = control_event.value
                else:
                    full_event_name = event_prefix + '-' + control_event.value
                self.accept(full_event_name, self.map_control_event, [game_event])
            self.bindings[game_event] = control_event
            print("{} = {}".format(game_event, control_event))
        self.method = device_class

    def map_control_event(self, event):
        base.messenger.send(event)

    def is_pressed(self, game_event):
        button_name = self.bindings[game_event].value
        if button_name == UNBOUND:
            return False
        if self.controller is None:
            # We're working on a keyboard
            button = ButtonRegistry.ptr().find_button(button_name)
            return base.mouseWatcherNode.is_button_down(button)
        else:
            button = self.controller.find_button(button_name)
            return button.pressed

    def axis_value(self, game_event, square_factor=1.0):
        axis_name = self.bindings[game_event].value
        if axis_name == UNBOUND:
            return 0.0
        axis = self.controller.find_axis(InputDevice.Axis[axis_name])
        v = axis.value
        return (v * (1-square_factor)) + v*abs(v) * square_factor

    def pressed_or_value(self, game_event, square_factor=1.0):
        input_name = self.bindings[game_event].value
        if input_name == UNBOUND:
            return 0.0
        if self.controller is None:
            button = ButtonRegistry.ptr().find_button(input_name)
            if base.mouseWatcherNode.is_button_down(button):
                return 1.0
            else:
                return 0.0
        axes_names = [axis.axis.name for axis in self.controller.axes]
        if input_name in axes_names:
            axis = self.controller.find_axis(InputDevice.Axis[input_name])
            v = axis.value
            return (v * (1-square_factor)) + v*abs(v) * square_factor
        else:
            button = self.controller.find_button(input_name)
            return button.pressed
