import os
import sys

from direct.showbase.Audio3DManager import Audio3DManager

import panda3d

import pman.shim

from panda3d.core import NodePath
from panda3d.bullet import BulletDebugNode

from wecs.core import Component
from wecs import panda3d as wecs_panda3d

from environment import Environment
from vehicle import Vehicle
from keybindings import DeviceListener
from camera import CameraController
from controller import VehicleController
from ecs.controllers import FZVehicle
from ecs.controllers import InputControllerECS
from ecs.controllers import GatherInputs
from ecs.controllers import VehicleControllerECS
from ecs.controllers import CameraControllerECS
from ecs.controllers import UpdateCamera
from ecs.controllers import UpdateVehicles


panda3d.core.load_prc_file(
    panda3d.core.Filename.expand_from('$MAIN_DIR/settings.prc')
)


class GameApp(wecs_panda3d.ECSShowBase):
    def __init__(self, map="lab"):
        super().__init__(self)
        pman.shim.init(self)
        # self.render.setShaderAuto()

        self.audio3d = Audio3DManager(
            base.sfxManagerList[0],
            base.cam,
        )

        self.accept('escape', sys.exit)
        self.set_frame_rate_meter(True)
        self.accept('f12', self.debug)

        # Systems

        base.add_system(wecs_panda3d.SetUpPhysics(), 0)
        base.add_system(wecs_panda3d.LoadModels(), 1)
        base.add_system(wecs_panda3d.DetermineTimestep(), 44)
        base.add_system(GatherInputs(), 45)
        base.add_system(UpdateVehicles(), 46)
        base.add_system(UpdateCamera(), 47)
        base.add_system(wecs_panda3d.DoPhysics(), 55)

        # Environment

        self.environment = Environment(self, map)
        self.bullet_debug()
        self.accept("f1", self.toggle_bullet_debug)
        env_entity = base.ecs_world.create_entity(
            wecs_panda3d.PhysicsWorld(
                world=self.environment.physics_world,
                clock=globalClock,
            ),
            wecs_panda3d.Scene(node=base.render),
            wecs_panda3d.PhysicsBody(
                node=self.environment.np,
                body=self.environment.body,
            ),
            wecs_panda3d.Model(node=self.environment.model),
            wecs_panda3d.Position(value=panda3d.core.Vec3(0, 0, 0)),
        )

        # Vehicles

        vehicle_files = [
            'Ricardeaut_Magnesium',
            'Ricardeaut_Himony',
            #'Psyoni_Culture',
            #'Texopec_Nako',
            #'Texopec_Reaal',
            # 'Doby_Phalix',
        ]
        spawn_points = self.environment.get_spawn_points()
        SPAWN_POINT_CONNECTOR = 'fz_spawn_point_connector'

        self.vehicles = []
        self.vehicle_entities = []
        for vehicle_file in vehicle_files:
            vehicle = Vehicle(
                self,
                vehicle_file,
            )
            self.vehicles.append(vehicle)

        for vehicle, spawn_point in zip(self.vehicles, spawn_points):
            connector = vehicle.model.find("**/"+SPAWN_POINT_CONNECTOR)
            hpr = -connector.get_hpr(spawn_point)
            pos = -connector.get_pos(spawn_point)
            self.vehicle_entities.append(base.ecs_world.create_entity(
                FZVehicle(pyobj=vehicle),
                wecs_panda3d.Scene(node=self.environment.model),
                wecs_panda3d.PhysicsBody(
                    node=vehicle.vehicle,
                    body=vehicle.physics_node,
                    world=env_entity._uid,
                ),
                wecs_panda3d.Model(node=vehicle.model),
                wecs_panda3d.Position(value=None, xyz=pos, hpr=hpr),
            ))

        # Controller objects

        self.player_vehicle_idx = 0
        player_entity = self.vehicle_entities[self.player_vehicle_idx]
        player_entity.add_component(InputControllerECS())
        player_entity.add_component(
            VehicleControllerECS(
                pyobj=VehicleController(self),
            ),
        )
        player_entity.add_component(
            CameraControllerECS(
                pyobj=CameraController(),
                camera=base.cam,
            ),
        )

    def next_vehicle(self):
        old_entity = self.vehicle_entities[self.player_vehicle_idx]
        num_vehicles = len(self.vehicles)
        self.player_vehicle_idx = (self.player_vehicle_idx + 1) % num_vehicles
        new_entity = self.vehicle_entities[self.player_vehicle_idx]

        new_entity.add_component(old_entity[InputControllerECS])
        new_entity.add_component(old_entity[VehicleControllerECS])
        new_entity.add_component(old_entity[CameraControllerECS])
        del old_entity[InputControllerECS]
        del old_entity[VehicleControllerECS]
        del old_entity[CameraControllerECS]

        # FIXME: WECSification: This should be looked up each frame by
        # the System.
        vehicle = new_entity[FZVehicle].pyobj
        new_entity[CameraControllerECS].pyobj.set_vehicle(vehicle)
        new_entity[VehicleControllerECS].pyobj.set_vehicle(vehicle)

    def bullet_debug(self):
        debug_node = BulletDebugNode('Debug')
        debug_node.show_wireframe(True)
        debug_node.show_constraints(True)
        debug_node.show_bounding_boxes(False)
        debug_node.show_normals(False)
        self.debug_np = self.render.attach_new_node(debug_node)
        self.environment.physics_world.set_debug_node(debug_node)

    def toggle_bullet_debug(self):
        if self.debug_np.is_hidden():
            self.debug_np.show()
        else:
            self.debug_np.hide()

    def debug(self):
        import pdb
        pdb.set_trace()


def main():
    if len(sys.argv) > 1:
        map = sys.argv[1]
        app = GameApp(map)
    else:
        app = GameApp()
    app.run()


if __name__ == '__main__':
    main()
