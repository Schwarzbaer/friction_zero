import os
import sys
from math import cos, pi
from random import random

from direct.showbase.ShowBase import ShowBase
import panda3d
import pman.shim

from panda3d.core import NodePath
from panda3d.core import Vec3
from panda3d.core import VBase4
from panda3d.core import BitMask32
from panda3d.core import DirectionalLight
from panda3d.core import GeomVertexReader
from panda3d.core import KeyboardButton
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
from panda3d.bullet import BulletConvexHullShape
from panda3d.bullet import BulletPlaneShape
from panda3d.bullet import BulletDebugNode

panda3d.core.load_prc_file(
    panda3d.core.Filename.expand_from('$MAIN_DIR/settings.prc')
)


SPAWN_POINTS = 'fz_spawn_point'
SPAWN_POINT_CONNECTOR = 'fz_spawn_point_connector'
REPULSOR = 'fz_repulsor'
THRUSTER = 'fz_thruster'
SKYSPHERE = 'fz_skysphere'
TERRAIN_COLLIDER = 'fz_collision'
FORCE = 'force'
ACTIVATION_DISTANCE = 'activation_distance'
CM_TERRAIN = BitMask32.bit(0)


class GameApp(ShowBase):
    def __init__(self, map="assets/maps/hills.bam"):
        ShowBase.__init__(self)
        pman.shim.init(self)
        self.accept('escape', sys.exit)
        self.set_frame_rate_meter(True)

        self.physics_world = BulletWorld()
        self.physics_world.setGravity(Vec3(0, 0, -9.81))
        self.bullet_debug()

        self.environment = Environment(self, map)
        spawn_points = self.environment.get_spawn_points()

        self.vehicles = []
        vehicle_1 = Vehicle(self, "assets/cars/Ricardeaut_Magnesium.bam")
        self.vehicles.append(vehicle_1)
        vehicle_2 = Vehicle(self, "assets/cars/Cadarache_DiamondMII.bam")
        self.vehicles.append(vehicle_2)

        for vehicle, spawn_point in zip(self.vehicles, spawn_points):
            vehicle.place(spawn_point)

        self.player_vehicle_idx = 0
        self.player_camera = CameraController(
            self,
            base.cam,
            self.vehicles[self.player_vehicle_idx],
        )

        self.player_controller = VehicleController(
            self,
            self.vehicles[self.player_vehicle_idx],
        )

        base.task_mgr.add(self.game_loop, "game_loop", sort=5)

    def game_loop(self, task):
        self.player_controller.gather_inputs()
        self.run_repulsors()
        self.run_gyroscopes()
        self.run_thrusters()
        self.update_physics()
        self.player_camera.update()
        return task.cont

    def next_vehicle(self):
        self.player_vehicle_idx = (self.player_vehicle_idx + 1) % len(self.vehicles)
        self.player_camera.set_vehicle(self.vehicles[self.player_vehicle_idx])
        self.player_controller.set_vehicle(self.vehicles[self.player_vehicle_idx])

    def run_repulsors(self):
        for vehicle in self.vehicles:
            vehicle.apply_repulsors()

    def run_gyroscopes(self):
        for vehicle in self.vehicles:
            vehicle.apply_gyroscope()

    def run_thrusters(self):
        for vehicle in self.vehicles:
            vehicle.apply_thrusters()

    def update_physics(self):
        dt = globalClock.getDt()
        self.physics_world.doPhysics(dt)

    def bullet_debug(self):
        debugNode = BulletDebugNode('Debug')
        debugNode.showWireframe(True)
        debugNode.showConstraints(True)
        debugNode.showBoundingBoxes(False)
        debugNode.showNormals(False)
        debugNP = self.render.attachNewNode(debugNode)
        debugNP.show()

        self.physics_world.setDebugNode(debugNP.node())


class Environment:
    def __init__(self, app, map_file):
        self.app = app

        node = BulletRigidBodyNode('Ground')
        self.np = self.app.render.attach_new_node(node)
        self.np.setPos(0, 0, 0)
        self.app.physics_world.attachRigidBody(node)

        self.model = loader.load_model(map_file)
        self.model.reparent_to(self.np)

        sky = self.model.find(SKYSPHERE)
        sky.reparent_to(base.cam)
        sky.set_bin('background', 0)
        sky.set_depth_write(False)
        sky.set_compass()
        sky.setLightOff()

        # Bullet collision mesh
        collision_solids = self.model.find_all_matches(
            '{}*'.format(TERRAIN_COLLIDER)
        )
        collision_solids.hide()
        for collision_solid in collision_solids:
            collision_solid.flatten_strong()
            for geom_node in collision_solid.find_all_matches('**/+GeomNode'):
                mesh = BulletTriangleMesh()
                # FIXME: Is this universally correct?
                mesh.addGeom(geom_node.node().get_geom(0))
                shape = BulletTriangleMeshShape(mesh, dynamic=False)
                terrain_node = BulletRigidBodyNode('terrain')
                terrain_node.addShape(shape)
                terrain_np = geom_node.attach_new_node(terrain_node)
                terrain_np.setCollideMask(CM_TERRAIN)
                self.app.physics_world.attachRigidBody(terrain_node)

    def get_spawn_points(self):
        spawn_nodes = [
            sp
            for sp in self.np.find_all_matches("**/{}*".format(SPAWN_POINTS))
        ]
        spawn_points = {}
        for sn in spawn_nodes:
            _, _, idx = sn.name.partition(':')
            idx = int(idx)
            spawn_points[idx] = sn
        sorted_spawn_points = [
            spawn_points[key]
            for key in sorted(spawn_points.keys())
        ]
        return sorted_spawn_points


class Vehicle:
    def __init__(self, app, model_file):
        self.app = app

        self.model = app.loader.load_model(model_file)

        self.physics_node = BulletRigidBodyNode('vehicle')
        self.physics_node.setLinearDamping(0.1)
        #self.physics_node.setAngularDamping(0.5)
        self.physics_node.setLinearSleepThreshold(0)
        self.physics_node.setAngularSleepThreshold(0)
        self.physics_node.setMass(100.0)

        shape = BulletConvexHullShape()
        for geom_node in self.model.find_all_matches("**/+GeomNode"):
            for geom in geom_node.node().get_geoms():
                vertices = GeomVertexReader(geom.get_vertex_data(), 'vertex')
                while not vertices.is_at_end():
                    v_geom = vertices.getData3f()
                    v_model = self.model.get_relative_point(geom_node, v_geom)
                    shape.add_point(v_model)
        self.physics_node.addShape(shape)
        self.vehicle = NodePath(self.physics_node)

        self.model.reparent_to(self.vehicle)

        self.repulsor_nodes = []
        for repulsor in self.model.find_all_matches('**/{}*'.format(REPULSOR)):
            self.add_repulsor(repulsor)

        self.thruster_nodes = []
        for thruster in self.model.find_all_matches('**/{}*'.format(THRUSTER)):
            self.add_thruster(thruster)

        self.repulsors_active = False
        self.gyroscope_active = True
        self.rot_target = Vec3(0, 0, 0)
        self.thrust = 0

    def np(self):
        return self.vehicle

    def place(self, spawn_point):

        self.vehicle.reparent_to(self.app.environment.model)
        connector = self.model.find(SPAWN_POINT_CONNECTOR)
        self.vehicle.set_hpr(-connector.get_hpr(spawn_point))
        self.vehicle.set_pos(-connector.get_pos(spawn_point))
        self.app.physics_world.attachRigidBody(self.physics_node)

    def toggle_repulsors(self):
        self.repulsors_active = not self.repulsors_active

    def toggle_gyroscope(self):
        self.gyroscope_active = not self.gyroscope_active

    def set_rot_target(self, rot_target):
        self.rot_target = rot_target

    def set_thrust(self, strength):
        self.thrust = strength

    def add_repulsor(self, repulsor):
        force = float(repulsor.get_tag(FORCE))
        activation_distance = float(repulsor.get_tag(ACTIVATION_DISTANCE))
        repulsor_np = repulsor.attach_new_node('repulsor')
        repulsor_np.set_python_tag(FORCE, force)
        repulsor_np.set_python_tag(ACTIVATION_DISTANCE, activation_distance)
        self.repulsor_nodes.append(repulsor_np)

    def apply_repulsors(self):
        dt = globalClock.dt
        for repulsor in self.repulsor_nodes:
            max_distance = repulsor.get_python_tag(ACTIVATION_DISTANCE)
            repulsor_pos = repulsor.get_pos(self.app.render)
            repulsor_dir = self.app.render.get_relative_vector(
                repulsor,
                Vec3(0, 0, -max_distance),
            )
            feeler = self.app.physics_world.ray_test_closest(
                repulsor_pos,
                repulsor_pos + repulsor_dir,
                CM_TERRAIN,
            )
            if feeler.hasHit() and self.repulsors_active:
                # Repulsor power at zero distance
                base_strength = repulsor.get_python_tag(FORCE)
                # Fraction of the repulsor beam above the ground
                activation_frac = feeler.get_hit_fraction()
                # Effective fraction of repulsors force
                activation = cos(0.5*pi * activation_frac)
                # Effective repulsor force
                strength = activation * base_strength
                # Resulting impulse
                impulse = self.vehicle.get_relative_vector(
                    repulsor,
                    Vec3(0, 0, strength),
                )
                # Apply
                repulsor_pos = repulsor.get_pos(self.vehicle)
                self.physics_node.apply_impulse(impulse * dt, repulsor_pos)

    def add_thruster(self, thruster):
        force = float(thruster.get_tag(FORCE))
        thruster.set_python_tag(FORCE, force)
        self.thruster_nodes.append(thruster)

    def apply_thrusters(self):
        dt = globalClock.dt
        for thruster in self.thruster_nodes:
            max_force = thruster.get_python_tag(FORCE)
            real_force = max_force * self.thrust
            thruster_pos = thruster.get_pos(self.vehicle)
            thrust_direction = self.app.render.get_relative_vector(
                thruster,
                Vec3(0, 0, 1)
            )
            self.physics_node.apply_impulse(
                thrust_direction * real_force * dt,
                thruster_pos,
            )

    def apply_gyroscope(self):
        if self.gyroscope_active:
            current_rot = self.physics_node.get_angular_velocity()
            unwanted_rot = current_rot - self.rot_target
            dt = globalClock.dt
            self.physics_node.apply_torque_impulse(-unwanted_rot * dt * 1500)

    def shock(self):
        self.physics_node.apply_impulse(
            Vec3(0,0,0),
            Vec3(random(), random(), random()) * 10,
        )
        self.physics_node.apply_torque_impulse(
            (Vec3(random(), random(), random()) - Vec3(0.5, 0.5, 0.5)) * 1000,
        )


CAM_MODE_FOLLOW = 1
CAM_MODE_DIRECTION = 2
CAM_MODE_MIXED = 3
CAM_MODES = [CAM_MODE_FOLLOW, CAM_MODE_DIRECTION, CAM_MODE_MIXED]


class CameraController:
    def __init__(self, app, camera, vehicle):
        self.app = app
        self.camera = camera
        self.vehicle = vehicle
        self.camera.reparent_to(self.app.render)

        self.camera_mode = 0
        self.app.accept("c", self.switch_camera_mode)

    def switch_camera_mode(self):
        self.camera_mode = (self.camera_mode + 1) % len(CAM_MODES)

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def update(self):
        horiz_dist = 15
        cam_offset = Vec3(0, 0, 5)
        focus_offset = Vec3(0, 0, 2)
        vehicle_pos = self.vehicle.np().get_pos(self.app.render)
        if CAM_MODES[self.camera_mode] == CAM_MODE_FOLLOW:
            vehicle_back = self.app.render.get_relative_vector(
                self.vehicle.np(),
                Vec3(0, -1, 0),
            )
        elif CAM_MODES[self.camera_mode] == CAM_MODE_DIRECTION:
            vehicle_back = -self.vehicle.physics_node.get_linear_velocity()
        elif CAM_MODES[self.camera_mode] == CAM_MODE_MIXED:
            vehicle_back = self.app.render.get_relative_vector(
                self.vehicle.np(),
                Vec3(0, -1, 0),
            )
            movement = self.vehicle.physics_node.get_linear_velocity()
            movement_back = -movement / movement.length()
            vehicle_back = vehicle_back + movement_back
        vehicle_back.z = 0
        vehicle_back = vehicle_back / vehicle_back.length()

        cam_pos = vehicle_pos + vehicle_back * horiz_dist + cam_offset
        focus = vehicle_pos + focus_offset
        base.cam.set_pos(cam_pos)
        base.cam.look_at(focus)


class VehicleController:
    def __init__(self, app, vehicle):
        self.app = app
        self.vehicle = vehicle
        self.app.accept("n", self.next_vehicle)
        self.app.accept("r", self.toggle_repulsors)
        self.app.accept("g", self.toggle_gyroscope)
        self.app.accept("s", self.shock)
        # self.app.accept("1", self.set_thrust, [0])
        # self.app.accept("2", self.set_thrust, [0.33])
        # self.app.accept("3", self.set_thrust, [0.66])
        # self.app.accept("4", self.set_thrust, [1])

    def gather_inputs(self):
        rot_target = Vec3(0, 0, 0)
        if self.app.mouseWatcherNode.is_button_down(KeyboardButton.left()):
            rot_target.z += 2
        if self.app.mouseWatcherNode.is_button_down(KeyboardButton.right()):
            rot_target.z -= 2
        self.vehicle.set_rot_target(rot_target)

        thrust = 0
        if self.app.mouseWatcherNode.is_button_down(KeyboardButton.up()):
            thrust = 1
        self.vehicle.set_thrust(thrust)

    def next_vehicle(self):
        self.app.next_vehicle()

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def shock(self):
        self.vehicle.shock()

    def toggle_repulsors(self):
        self.vehicle.toggle_repulsors()

    def toggle_gyroscope(self):
        self.vehicle.toggle_gyroscope()

    def set_thrust(self, strength):
        self.vehicle.set_thrust(strength)


def main():
    if len(sys.argv) > 1:
        map = "assets/maps/"+sys.argv[1]
        app = GameApp(map)
    else:
        app = GameApp()
    app.run()


if __name__ == '__main__':
    main()
