from dataclasses import field

from wecs.core import Component
from wecs.core import System
from wecs.core import and_filter

from keybindings import DeviceListener


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


@Component()
class CameraControllerECS:
    pyobj: object
    camera: object # FIXME: Specific type


class UpdateCamera(System):
    entity_filters = {
        'vehicle': and_filter([
            FZVehicle,
            InputControllerECS,
            VehicleControllerECS,
            CameraControllerECS,
        ]),
    }

    def init_entity(self, filter_name, entity):
        camera = entity[CameraControllerECS].camera
        vehicle = entity[FZVehicle].pyobj
        control = entity[VehicleControllerECS].pyobj
        entity[CameraControllerECS].pyobj.init(camera, vehicle, control)

    def update(self, entities_by_filter):
        for entity in entities_by_filter['vehicle']:
            entity[CameraControllerECS].pyobj.update()
