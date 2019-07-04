import os
import sys

from direct.showbase.Audio3DManager import Audio3DManager

import panda3d

import pman.shim

from panda3d.core import NodePath
from panda3d.bullet import BulletDebugNode

from wecs import panda3d as wecs_panda3d

from environment import Environment
from vehicle import Vehicle
from keybindings import DeviceListener
from camera import CameraController
from controller import VehicleController


panda3d.core.load_prc_file(
    panda3d.core.Filename.expand_from('$MAIN_DIR/settings.prc')
)


class GameApp(wecs_panda3d.ECSShowBase):
    def __init__(self, map="lab"):
        super().__init__(self)
        pman.shim.init(self)

        self.audio3d = Audio3DManager(
            base.sfxManagerList[0],
            base.cam,
        )

        self.accept('escape', sys.exit)
        #self.render.setShaderAuto()
        self.set_frame_rate_meter(True)
        self.accept('f12', self.debug)

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
            wecs_panda3d.Model(node=self.environment.model),
            wecs_panda3d.Position(value=panda3d.core.Vec3(0, 0, 0)),
        )
        env_entity[wecs_panda3d.PhysicsBody] = wecs_panda3d.PhysicsBody(
            node=self.environment.np,
            body=self.environment.body,
            world=env_entity._uid,
            scene=env_entity._uid,
        )

        base.add_system(wecs_panda3d.SetUpPhysics(), 0)
        base.add_system(wecs_panda3d.LoadModels(), 1)

        # Vehicles

        self.vehicles = []
        vehicle_files = [
            'Ricardeaut_Magnesium',
            #'Ricardeaut_Himony',
            #'Psyoni_Culture',
            #'Texopec_Nako',
            #'Texopec_Reaal',
            # 'Doby_Phalix',
        ]

        for vehicle_file in vehicle_files:
            vehicle = Vehicle(
                self,
                vehicle_file,
            )
            self.vehicles.append(vehicle)

        spawn_points = self.environment.get_spawn_points()
        for vehicle, spawn_point in zip(self.vehicles, spawn_points):
            vehicle.place(spawn_point)

        self.controller_listener = DeviceListener()

        self.player_vehicle_idx = 0
        self.player_controller = VehicleController(
            self,
            self.vehicles[self.player_vehicle_idx],
            self.controller_listener,
        )
        self.player_camera = CameraController(
            base.cam,
            self.vehicles[self.player_vehicle_idx],
            self.player_controller,
        )

        base.add_system(wecs_panda3d.DetermineTimestep(), 44)
        base.task_mgr.add(
            self.game_loop_pre_render,
            "game_loop_pre_render",
            sort=45,
        )

        base.add_system(wecs_panda3d.DoPhysics(), 55)

    def game_loop_pre_render(self, task):
        self.player_controller.gather_inputs()
        for vehicle in self.vehicles:
            vehicle.game_loop()
        self.player_camera.update()
        return task.cont

    def next_vehicle(self):
        num_vehicles = len(self.vehicles)
        self.player_vehicle_idx = (self.player_vehicle_idx + 1) % num_vehicles
        new_vehicle = self.vehicles[self.player_vehicle_idx]
        self.player_camera.set_vehicle(new_vehicle)
        self.player_controller.set_vehicle(new_vehicle)

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
