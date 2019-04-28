import sys
from enum import Enum

from panda3d.core import load_prc_file
from panda3d.core import Filename
from panda3d.core import InputDevice
from panda3d.core import ConfigVariableString
from panda3d.core import KeyboardButton

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


GE_TOGGLE_REPULSOR = 'toggle_repulsor'
GE_FORWARD = 'forward'
GE_BACKWARD = 'backward'
GE_TURN = 'turn'
GE_TURN_LEFT = 'turn_left'
GE_TURN_RIGHT = 'turn_right'
GE_STRAFE = 'strafe'
GE_STRAFE_LEFT = 'strafe_left'
GE_STRAFE_RIGHT = 'strafe_right'
GE_HOVER = 'hover'
GE_STABILIZE = 'stabilize'
GE_GYRO_PITCH = 'gyro_pitch'
GE_GYRO_ROLL = 'gyro_roll'
GE_THRUST = 'thrust'
GE_AIRBRAKE = 'airbrake'
GE_CAMERA_MODE = 'camera_mode'
GE_NEXT_VEHICLE = 'next_vehicle'


keyboard_bindings = {
    GE_TOGGLE_REPULSOR: ConfigVariableString('keyboard_toggle_repulsor', 'r'),
    GE_FORWARD: ConfigVariableString('keyboard_forward', 'arrow_up'),
    GE_BACKWARD: ConfigVariableString('keyboard_backward', 'arrow_down'),
    GE_TURN_LEFT: ConfigVariableString('keyboard_turn_left', 'arrow_left'),
    GE_TURN_RIGHT: ConfigVariableString('keyboard_turn_right', 'arrow_right'),
    GE_STRAFE_LEFT: ConfigVariableString('keyboard_strafe_left', 'a'),
    GE_STRAFE_RIGHT: ConfigVariableString('keyboard_strafe_right', 'd'),
    GE_HOVER: ConfigVariableString('keyboard_hover', 's'),
    GE_STABILIZE: ConfigVariableString('keyboard_stabilize', 'lshift'),
    GE_THRUST: ConfigVariableString('keyboard_thrust', 'space'),
    GE_AIRBRAKE: ConfigVariableString('keyboard_airbrake', 'w'),
    GE_CAMERA_MODE: ConfigVariableString('keyboard_camera_mode', 'c'),
    GE_NEXT_VEHICLE: ConfigVariableString('keyboard_next_vehicle', 'n'),
}


gamepad_bindings = {
    GE_TOGGLE_REPULSOR: ConfigVariableString('gamepad_repulsor_on', 'face_a'),
    GE_FORWARD: ConfigVariableString('gamepad_forward', 'left_y'),
    GE_TURN: ConfigVariableString('gamepad_turn', 'left_x'),
    GE_GYRO_PITCH: ConfigVariableString('gamepad_gyro_pitch', 'left_trigger'),
    GE_GYRO_ROLL: ConfigVariableString('gamepad_gyro_roll', 'right_trigger'),
    GE_STRAFE: ConfigVariableString('gamepad_strafe', 'rtrigger'),
    GE_HOVER: ConfigVariableString('gamepad_hover', 'face_b'),
    GE_STABILIZE: ConfigVariableString('gamepad_stabilize', 'rshoulder'),
    GE_THRUST: ConfigVariableString('gamepad_thrust', 'lshoulder'),
    GE_AIRBRAKE: ConfigVariableString('gamepad_airbrake', 'ltrigger'),
    GE_CAMERA_MODE: ConfigVariableString('gamepad_camera_mode', 'face_b'),
    GE_NEXT_VEHICLE: ConfigVariableString('gamepad_next_vehicle', 'face_y'),
}


flight_stick_bindings = {}


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
            self.ignore(control_event)
        self.bindings = {}
        # Listen for new bindings
        bindings = device_bindings[device_class]
        event_prefix = event_prefixes[device_class]
        for game_event, control_event in bindings.items():
            full_event_name = event_prefix + '-' + control_event.value
            self.accept(full_event_name, self.map_control_event, [game_event])
            self.bindings[game_event] = control_event
            print("{} = {}".format(game_event, control_event))
        self.method = device_class

    def map_control_event(self, event):
        base.messenger.send(event)

    def is_pressed(self, game_event):
        button_name = self.bindings[game_event].value
        # FIXME: Cursor key events have their key name prefixed with 'arrow-',
        # So we strip it here. Maybe we should add it in the event mapping bit
        # instead?
        if button_name.startswith('arrow_'):
            button_name = button_name[6:]
        if self.controller is None:
            # We're working on a keyboard
            if len(button_name) == 1:
                button = KeyboardButton.ascii_key(button_name.encode('UTF-8'))
            else:
                # FIXME: This can't be the Panda way.
                button = getattr(KeyboardButton, button_name)()
            return base.mouseWatcherNode.is_button_down(button)
        else:
            return self.controller.find_button(button_name).pressed

    def axis_value(self, game_event):
        axis_name = self.bindings[game_event].value
        # FIXME: Can't be the Panda way, either!
        axis_id = getattr(InputDevice.Axis, axis_name)
        value = self.controller.find_axis(axis_id).value
        return value
