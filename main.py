import os
import sys
from random import random

from direct.showbase.ShowBase import ShowBase
from direct.actor.Actor import Actor
import panda3d
import pman.shim

from panda3d.core import NodePath
from panda3d.bullet import BulletDebugNode

from environment import Environment
from vehicle import Vehicle
from keybindings import DeviceListener
from camera import CameraController
from controller import VehicleController


panda3d.core.load_prc_file(
    panda3d.core.Filename.expand_from('$MAIN_DIR/settings.prc')
)


class GameApp(ShowBase):
    def __init__(self, map="assets/maps/lab.bam"):
        ShowBase.__init__(self)
        pman.shim.init(self)
        self.accept('escape', sys.exit)
        #self.render.setShaderAuto()
        self.set_frame_rate_meter(True)

        self.environment = Environment(self, map)
        self.bullet_debug()
        self.accept("b", self.toggle_bullet_debug)

        self.vehicles = []
        vehicle_files = [
            'assets/cars/Ricardeaut_Magnesium.bam',
            'assets/cars/Cadarache_DiamondMII.bam',
            # 'assets/cars/Doby_Phalix.bam',
            'assets/cars/Texopec_Nako.bam',
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

        base.task_mgr.add(self.game_loop, "game_loop", sort=5)

    def game_loop(self, task):
        self.player_controller.gather_inputs()
        for vehicle in self.vehicles:
            vehicle.game_loop()
        self.environment.update_physics()
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


def main():
    if len(sys.argv) > 1:
        map = "maps/"+sys.argv[1]
        app = GameApp(map)
    else:
        app = GameApp()
    app.run()


if __name__ == '__main__':
    main()
