import sys
from enum import Enum

from panda3d.core import load_prc_file
from panda3d.core import Filename
from panda3d.core import InputDevice
from panda3d.core import ConfigVariableString

from direct.showbase.ShowBase import ShowBase


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


class DeviceListener:
    def __init__(self):
        self.controller = None
        self.elect_control_method()
        base.accept("connect-device", self.connect)
        base.accept("disconnect-device", self.disconnect)

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
        else:
            # It's the keybard
            print("Assuming keyboard")


if __name__ == '__main__':
    app = ShowBase()
    app.accept('escape', sys.exit)
    dl = DeviceListener()
    app.run()
