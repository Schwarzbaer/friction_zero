import sys
from enum import Enum

from panda3d.core import load_prc_file
from panda3d.core import Filename
from panda3d.core import InputDevice
from panda3d.core import ConfigVariableString

from direct.showbase.ShowBase import ShowBase
from direct.showbase.DirectObject import DirectObject


# FIXME: Replace with pman's expand_from $MAIN_DIR
load_prc_file(
    Filename('keybindings.prc')
)


class Methods(Enum):
    GAMEPAD = InputDevice.DeviceClass.gamepad
    FLIGHTSTICK = InputDevice.DeviceClass.flight_stick
    KEYBOARD = InputDevice.DeviceClass.keyboard


priorities = {cls.value: priority for priority, cls in enumerate(Methods)}


event_prefixes = {
    InputDevice.DeviceClass.gamepad: 'gamepad',
    InputDevice.DeviceClass.flight_stick: 'flight_stick',
}


GE_TOGGLE_REPULSOR = 'toggle_repulsor'
GE_FORWARD = 'forward'
GE_TURN = 'turn'
GE_STRAFE = 'strafe'
GE_STRAFE_LEFT = 'strafe_left'
GE_STRAFE_RIGHT = 'strafe_right'
GE_HOVER = 'hover'
GE_STABILIZE = 'stabilize'
GE_THRUST = 'thrust'
GE_AIRBRAKE = 'airbrake'
GE_CAMERA_MODE = 'camera_mode'
GE_NEXT_VEHICLE = 'next_vehicle'


keyboard_bindings = {
    GE_TOGGLE_REPULSOR: ConfigVariableString('keyboard_toggle_repulsor', 'r'),
    GE_FORWARD: ConfigVariableString('keyboard_forward', 'left_y'),
    GE_TURN: ConfigVariableString('keyboard_turn', 'left_x'),
    GE_STRAFE_LEFT: ConfigVariableString('keyboard_strafe_left', 'a'),
    GE_STRAFE_RIGHT: ConfigVariableString('keyboard_strafe_right', 'd'),
    GE_HOVER: ConfigVariableString('keyboard_hover', 's'),
    GE_STABILIZE: ConfigVariableString('keyboard_stabilize', 'g'),
    GE_THRUST: ConfigVariableString('keyboard_thrust', 'space'),
    GE_AIRBRAKE: ConfigVariableString('keyboard_airbrake', 'w'),
    GE_CAMERA_MODE: ConfigVariableString('keyboard_camera_mode', 'rstick'),
    GE_NEXT_VEHICLE: ConfigVariableString('keyboard_next_vehicle', 'face_y'),
}


gamepad_bindings = {
    GE_TOGGLE_REPULSOR: ConfigVariableString('gamepad_repulsor_on', 'face_a'),
    GE_FORWARD: ConfigVariableString('gamepad_forward', 'left_y'),
    GE_TURN: ConfigVariableString('gamepad_turn', 'left_x'),
    GE_STRAFE: ConfigVariableString('gamepad_strafe', 'lstick'),
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
                new_device_class = InputDevice.device_class.keyboard
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
                event_prefix = event_prefixes[device.device_class]
                base.attach_input_device(device, prefix=event_prefix)
                self.map_bindings(device.device_class)
        else:
            # It's the keybard
            print("Assuming keyboard")

    def map_bindings(self, device_class):
        # TODO: Ignore old bindings
        for game_event, control_event in self.bindings.items():
            self.ignore(control_event)
        # Listen for new bindings
        bindings = device_bindings[device_class]
        event_prefix = event_prefixes[device_class]
        for game_event, control_event in bindings.items():
            control_event_name = event_prefix + '-' + control_event.value
            self.accept(control_event_name, self.map_control_event, [game_event])
            self.bindings[game_event] = control_event_name

    def map_control_event(self, event):
        print(event)
        base.messenger.send(event)


if __name__ == '__main__':
    app = ShowBase()
    app.accept('escape', sys.exit)
    # base.messenger.toggle_verbose()
    dl = DeviceListener()
    app.run()