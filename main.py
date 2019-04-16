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
from panda3d.core import Plane
from panda3d.core import DirectionalLight
from panda3d.core import CollisionTraverser
from panda3d.core import CollisionHandlerQueue
from panda3d.core import CollisionNode
from panda3d.core import CollisionRay
from panda3d.core import CollisionSegment
from panda3d.core import CollisionPlane
from panda3d.core import GeomVertexReader
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


class GameApp(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)
        pman.shim.init(self)
        self.accept('escape', sys.exit)

        self.physics_world = BulletWorld()
        self.physics_world.setGravity(Vec3(0, 0, -9.81))
        self.bullet_debug()

        self.repulsor_traverser = CollisionTraverser('repulsor')
        #self.repulsor_traverser.show_collisions(base.render)

        environment = Environment(self, "assets/maps/plane.bam")
        spawn_points = environment.get_spawn_points()

        self.vehicles = []
        vehicle_1 = Vehicle(self, "assets/cars/Ricardeaut_Magnesium.bam")
        self.vehicles.append(vehicle_1)
        vehicle_2 = Vehicle(self, "assets/cars/Ricardeaut_Magnesium.bam")
        self.vehicles.append(vehicle_2)

        for vehicle, spawn_point in zip(self.vehicles, spawn_points):
            vehicle.place(
                spawn_point.get_pos(),
                spawn_point.get_hpr(),
            )

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

        #base.task_mgr.add(self.player_controller.run, "input", sort=0)
        base.task_mgr.add(self.run_repulsors, 'run repulsors', sort=1)
        base.task_mgr.add(self.run_gyroscopes, 'run gyroscopes', sort=2)
        base.task_mgr.add(self.run_thrusters, 'run thrusters', sort=3)
        base.task_mgr.add(self.update_physics, 'physics', sort=4)
        base.task_mgr.add(self.player_camera.update, "camera", sort=5)

    def next_vehicle(self):
        self.player_vehicle_idx = (self.player_vehicle_idx + 1) % len(self.vehicles)
        self.player_camera.set_vehicle(self.vehicles[self.player_vehicle_idx])
        self.player_controller.set_vehicle(self.vehicles[self.player_vehicle_idx])

    def run_repulsors(self, task):
        self.repulsor_traverser.traverse(base.render)
        for vehicle in self.vehicles:
            vehicle.apply_repulsors()
        return task.cont

    def run_gyroscopes(self, task):
        for vehicle in self.vehicles:
            vehicle.apply_gyroscope()
        return task.cont

    def run_thrusters(self, task):
        for vehicle in self.vehicles:
            vehicle.apply_thrusters()
        return task.cont

    def update_physics(self, task):
        dt = globalClock.getDt()
        self.physics_world.doPhysics(dt)
        return task.cont

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

        shape = BulletPlaneShape(Vec3(0, 0, 1), 0)
        node = BulletRigidBodyNode('Ground')
        node.addShape(shape)
        self.np = self.app.render.attach_new_node(node)
        self.np.setPos(0, 0, 0)
        self.app.physics_world.attachRigidBody(node)

        model = loader.load_model(map_file)
        model.reparent_to(self.np)

        coll_solid = CollisionPlane(Plane((0, 0, 1), (0, 0, 0)))
        coll_node = CollisionNode('ground')
        coll_node.set_from_collide_mask(0)
        coll_node.add_solid(coll_solid)
        coll_np = self.np.attach_new_node(coll_node)
        #coll_np.show()

        dlight = DirectionalLight('dlight')
        dlight.setColor(VBase4(1, 1, 1, 1))
        dlnp = self.app.render.attachNewNode(dlight)
        dlnp.setHpr(20, -75, 0)
        self.app.render.setLight(dlnp)

    def get_spawn_points(self):
        spawn_nodes = [sp for sp in self.np.find_all_matches("**/spawn_point*")]
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

        model = app.loader.load_model(model_file)

        self.physics_node = BulletRigidBodyNode('vehicle')
        self.physics_node.setLinearDamping(0.5)
        #self.physics_node.setAngularDamping(0.5)
        self.physics_node.setLinearSleepThreshold(0)
        self.physics_node.setAngularSleepThreshold(0)
        self.physics_node.setMass(100.0)

        shape = BulletConvexHullShape()
        for geom_node in model.find_all_matches("**/+GeomNode"):
            for geom in geom_node.node().get_geoms():
                vertices = GeomVertexReader(geom.get_vertex_data(), 'vertex')
                while not vertices.is_at_end():
                    v_geom = vertices.getData3f()
                    v_model = model.get_relative_point(geom_node, v_geom)
                    shape.add_point(v_model)
        self.physics_node.addShape(shape)
        self.vehicle = NodePath(self.physics_node)

        model.reparent_to(self.vehicle)

        self.repulsor_queue = CollisionHandlerQueue()
        for repulsor in model.find_all_matches('**/fz_repulsor:*'):
            self.add_repulsor(repulsor)

        self.thruster_nodes = []
        for thruster in model.find_all_matches('**/fz_thruster:*'):
            self.add_thruster(thruster)

        self.repulsors_active = False
        self.gyroscope_active = True
        self.thrust = 0

    def np(self):
        return self.vehicle

    def place(self, coordinate, orientation):
        self.vehicle.reparent_to(self.app.render)
        self.vehicle.set_pos(coordinate)
        self.vehicle.set_hpr(orientation)
        self.app.physics_world.attachRigidBody(self.physics_node)

    def toggle_repulsors(self):
        self.repulsors_active = not self.repulsors_active

    def toggle_gyroscope(self):
        self.gyroscope_active = not self.gyroscope_active

    def set_thrust(self, strength):
        self.thrust = strength

    def add_repulsor(self, repulsor):
        force = float(repulsor.get_tag('force'))
        activation_distance = float(repulsor.get_tag('activation_distance'))
        repulsor_solid = CollisionSegment(
            Vec3(0, 0, 0),
            Vec3(0, 0, -activation_distance),
        )
        repulsor_node = CollisionNode('repulsor')
        repulsor_node.add_solid(repulsor_solid)
        repulsor_node.set_into_collide_mask(0)
        repulsor_np = repulsor.attach_new_node(repulsor_node)
        repulsor_np.set_python_tag('force', force)
        repulsor_np.set_python_tag('activation_distance', activation_distance)
        repulsor_np.show()
        self.app.repulsor_traverser.addCollider(
            repulsor_np, self.repulsor_queue,
        )

    def apply_repulsors(self):
        dt = globalClock.dt
        for entry in self.repulsor_queue.entries:
            repulsor = entry.get_from_node_path()
            hit_feeler = entry.get_surface_point(repulsor)
            max_distance = repulsor.get_python_tag('activation_distance')
            if hit_feeler.length() < max_distance and self.repulsors_active:
                # Repulsor power at zero distance
                base_strength = repulsor.get_python_tag('force')
                # Fraction of the repulsor beam above the ground
                activation_frac = hit_feeler.length() / max_distance
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
        force = float(thruster.get_tag('force'))
        thruster.set_python_tag('force', force)
        self.thruster_nodes.append(thruster)

    def apply_thrusters(self):
        dt = globalClock.dt
        for thruster in self.thruster_nodes:
            max_force = thruster.get_python_tag('force')
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
        rot = self.physics_node.get_angular_velocity()
        dt = globalClock.dt
        self.physics_node.apply_torque_impulse(-rot * dt * 1500)

    def shock(self):
        #self.physics_node.apply_impulse(
        #    Vec3(0,0,0),
        #    Vec3(random(), random(), random()) * 10,
        #)
        self.physics_node.apply_torque_impulse(
            Vec3(0, 0, 1000),
            #(Vec3(random(), random(), random()) - Vec3(0.5, 0.5, 0.5)) * 1000,
        )


class CameraController:
    def __init__(self, app, camera, vehicle):
        self.app = app
        self.camera = camera
        self.vehicle = vehicle
        self.camera.reparent_to(self.app.render)

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def update(self, task):
        horiz_dist = 15
        cam_offset = Vec3(0, 0, 5)
        focus_offset = Vec3(0, 0, 2)
        vehicle_pos = self.vehicle.np().get_pos(self.app.render)
        vehicle_back = self.app.render.get_relative_vector(
            self.vehicle.np(),
            Vec3(0, -1, 0),
        )
        vehicle_back.z = 0
        vehicle_back = vehicle_back / vehicle_back.length()

        cam_pos = vehicle_pos + vehicle_back * horiz_dist + cam_offset
        focus = vehicle_pos + focus_offset
        base.cam.set_pos(cam_pos)
        base.cam.look_at(focus)

        return task.cont


class VehicleController:
    def __init__(self, app, vehicle):
        self.app = app
        self.vehicle = vehicle
        self.app.accept("n", self.next_vehicle)
        self.app.accept("r", self.toggle_repulsors)
        self.app.accept("g", self.toggle_gyroscope)
        self.app.accept("s", self.shock)
        self.app.accept("1", self.set_thrust, [0])
        self.app.accept("2", self.set_thrust, [0.33])
        self.app.accept("3", self.set_thrust, [0.66])
        self.app.accept("4", self.set_thrust, [1])

    def gather_inputs(self, task):
        return task.cont

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
    app = GameApp()
    app.run()

if __name__ == '__main__':
    main()
