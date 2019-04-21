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
from panda3d.core import Spotlight
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


VEHICLE = 'fz_body'
TERRAIN = 'fz_terrain'
SPAWN_POINTS = 'fz_spawn_point'
SPAWN_POINT_CONNECTOR = 'fz_spawn_point_connector'
REPULSOR = 'fz_repulsor'
THRUSTER = 'fz_thruster'
SKYSPHERE = 'fz_skysphere'
TERRAIN_COLLIDER = 'fz_collision'
FORCE = 'force'
ACTIVATION_DISTANCE = 'activation_distance'
FRICTION = 'friction'
MASS = 'mass'
GRAVITY = 'gravity'
CM_TERRAIN = BitMask32.bit(0)


class GameApp(ShowBase):
    def __init__(self, map="assets/maps/hills.bam"):
        ShowBase.__init__(self)
        pman.shim.init(self)
        self.accept('escape', sys.exit)
        #self.render.setShaderAuto()
        self.set_frame_rate_meter(True)

        self.environment = Environment(self, map)
        self.bullet_debug()
        self.accept("b", self.toggle_bullet_debug)

        self.vehicles = []
        vehicle_1 = Vehicle(self, "assets/cars/Ricardeaut_Magnesium.bam")
        self.vehicles.append(vehicle_1)
        vehicle_2 = Vehicle(self, "assets/cars/Cadarache_DiamondMII.bam")
        self.vehicles.append(vehicle_2)

        spawn_points = self.environment.get_spawn_points()
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
        debugNode = BulletDebugNode('Debug')
        debugNode.showWireframe(True)
        debugNode.showConstraints(True)
        debugNode.showBoundingBoxes(False)
        debugNode.showNormals(False)
        self.debugNP = self.render.attachNewNode(debugNode)
        self.environment.physics_world.setDebugNode(debugNode)

    def toggle_bullet_debug(self):
        if self.debugNP.is_hidden():
            self.debugNP.show()
        else:
            self.debugNP.hide()


class Environment:
    def __init__(self, app, map_file):
        self.app = app

        self.physics_world = BulletWorld()

        node = BulletRigidBodyNode('Ground')
        self.np = self.app.render.attach_new_node(node)
        self.np.setPos(0, 0, 0)
        self.physics_world.attachRigidBody(node)

        self.model = loader.load_model(map_file)
        self.model.reparent_to(self.np)

        gravity_node = self.model.find('**/={}'.format(GRAVITY))
        gravity_str = gravity_node.get_tag(GRAVITY)
        gravity = Vec3(0, 0, -float(gravity_str))
        self.physics_world.setGravity(gravity)

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
                friction_node = collision_solid.find('**/={}'.format(FRICTION))
                friction_str = friction_node.get_tag('friction')
                friction = float(friction_str)
                terrain_node.set_friction(friction)
                terrain_np = geom_node.attach_new_node(terrain_node)
                terrain_np.setCollideMask(CM_TERRAIN)
                self.physics_world.attachRigidBody(terrain_node)

    def add_physics_node(self, node):
        self.physics_world.attachRigidBody(node)

    def update_physics(self):
        dt = globalClock.getDt()
        self.physics_world.doPhysics(dt)

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


CURRENT_ORIENTATION = 'current_orientation'
CURRENT_MOVEMENT = 'current_movement'
CURRENT_ROTATION = 'current_rotation'
TARGET_ORIENTATION = 'target_orientation'
TARGET_ROTATION = 'target_rotation'
REPULSOR_ACTIVATION = 'repulsor_activation'
REPULSOR_RAY_ACTIVE = 'repulsor_ray_active'
REPULSOR_RAY_FRAC = 'repulsor_ray_frac'
GYRO_ROTATION = 'gyro_rotation'
THRUST = 'thrust'
THRUSTER_ACTIVATION = 'thruster_activation'


class Vehicle:
    def __init__(self, app, model_file):
        self.app = app

        self.model = app.loader.load_model(model_file)

        self.physics_node = BulletRigidBodyNode('vehicle')
        friction_node = self.model.find('**/={}'.format(FRICTION))
        friction_str = friction_node.get_tag('friction')
        friction = float(friction_str)
        self.physics_node.set_friction(friction)
        # FIXME: This will be replaced by air drag.
        self.physics_node.setLinearDamping(0.2)
        self.physics_node.setLinearSleepThreshold(0)
        self.physics_node.setAngularSleepThreshold(0)
        mass_node = self.model.find('**/={}'.format(MASS))
        mass_str = mass_node.get_tag('mass')
        mass = float(mass_str)
        self.physics_node.setMass(mass)

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

        self.inputs = {
            TARGET_ORIENTATION: Vec3(0, 0, 0),
            TARGET_ROTATION: Vec3(0, 0, 0),
            REPULSOR_ACTIVATION: 0,
            THRUST: 0,
        }
        self.sensors = {}
        self.commands = {}

        # self.light = self.app.render.attachNewNode(Spotlight("Spot"))
        # self.light.node().setScene(base.render)
        # self.light.node().setShadowCaster(True)
        # self.light.node().showFrustum()
        # self.light.node().getLens().setFov(40)
        # self.light.node().getLens().setNearFar(10, 100)
        # render.setLight(self.light)
        # self.light.set_pos(self.model.get_pos() + Vec3(0, 0, 40))
        # self.light.set_p(90)

    def np(self):
        return self.vehicle

    def place(self, spawn_point):
        self.vehicle.reparent_to(self.app.environment.model)
        connector = self.model.find("**/"+SPAWN_POINT_CONNECTOR)
        self.vehicle.set_hpr(-connector.get_hpr(spawn_point))
        self.vehicle.set_pos(-connector.get_pos(spawn_point))
        self.app.environment.add_physics_node(self.physics_node)

    def set_inputs(self, inputs):
        self.inputs = inputs

    def add_repulsor(self, repulsor):
        force = float(repulsor.get_tag(FORCE))
        activation_distance = float(repulsor.get_tag(ACTIVATION_DISTANCE))
        repulsor_np = repulsor.attach_new_node('repulsor')
        repulsor_np.set_python_tag(FORCE, force)
        repulsor_np.set_python_tag(ACTIVATION_DISTANCE, activation_distance)
        self.repulsor_nodes.append(repulsor_np)

    def add_thruster(self, thruster):
        force = float(thruster.get_tag(FORCE))
        thruster.set_python_tag(FORCE, force)
        self.thruster_nodes.append(thruster)

    def game_loop(self):
        self.gather_sensors()
        self.ecu()
        self.apply_repulsors()
        self.apply_gyroscope()
        self.apply_thrusters()

    def gather_sensors(self):
        repulsor_ray_active = []
        repulsor_ray_frac = []
        for repulsor in self.repulsor_nodes:
            max_distance = repulsor.get_python_tag(ACTIVATION_DISTANCE)
            repulsor_pos = repulsor.get_pos(self.app.render)
            repulsor_dir = self.app.render.get_relative_vector(
                repulsor,
                Vec3(0, 0, -max_distance),
            )
            # FIXME: `self.app.environment.physics_world` is ugly.
            feeler = self.app.environment.physics_world.ray_test_closest(
                repulsor_pos,
                repulsor_pos + repulsor_dir,
                CM_TERRAIN,
            )
            if feeler.hasHit():
                repulsor_ray_active.append(True)
                repulsor_ray_frac.append(feeler.get_hit_fraction())
            else:
                repulsor_ray_active.append(False)
                repulsor_ray_frac.append(1.0)

        self.sensors = {
            CURRENT_ORIENTATION: self.vehicle.get_hpr(self.app.render),
            CURRENT_MOVEMENT: self.physics_node.get_linear_velocity(),
            CURRENT_ROTATION: self.physics_node.get_angular_velocity(),
            REPULSOR_RAY_ACTIVE: repulsor_ray_active,
            REPULSOR_RAY_FRAC: repulsor_ray_frac,
        }

    def ecu(self):
        repulsor_activation = [self.inputs[REPULSOR_ACTIVATION]
                               for _ in self.repulsor_nodes]

        current_rot = self.sensors[CURRENT_ROTATION]
        wanted_rot = self.inputs[TARGET_ROTATION]
        unwanted_rot = current_rot - wanted_rot

        thruster_activation = [self.inputs[THRUST]
                               for _ in self.thruster_nodes]

        self.commands = {
            REPULSOR_ACTIVATION: repulsor_activation,
            GYRO_ROTATION: -unwanted_rot,
            THRUSTER_ACTIVATION: thruster_activation,
        }

    def apply_repulsors(self):
        dt = globalClock.dt
        repulsor_data = zip(
            self.repulsor_nodes,
            self.sensors[REPULSOR_RAY_ACTIVE],
            self.sensors[REPULSOR_RAY_FRAC],
            self.commands[REPULSOR_ACTIVATION],
        )
        for node, active, frac, activation in repulsor_data:
            if active:
                # Repulsor power at zero distance
                base_strength = node.get_python_tag(FORCE)
                # Effective fraction of repulsors force
                transfer_frac = cos(0.5*pi * frac)
                # Effective repulsor force
                strength = base_strength * activation * transfer_frac
                # Resulting impulse
                impulse = self.vehicle.get_relative_vector(
                    node,
                    Vec3(0, 0, strength),
                )
                # Apply
                repulsor_pos = node.get_pos(self.vehicle)
                self.physics_node.apply_impulse(impulse * dt, repulsor_pos)

    def apply_gyroscope(self):
        gyro_rotation = self.commands[GYRO_ROTATION]
        target_torque = gyro_rotation * 15000
        max_torque = 10000
        capped_torque = target_torque
        if capped_torque > max_torque:
            capped_torque = capped_torque / capped_torque.length() * max_torque
        dt = globalClock.dt
        self.physics_node.apply_torque_impulse(capped_torque * dt)

    def apply_thrusters(self):
        dt = globalClock.dt
        thruster_data = zip(
            self.thruster_nodes,
            self.commands[THRUSTER_ACTIVATION],
        )
        for node, thrust in thruster_data:
            max_force = node.get_python_tag(FORCE)
            real_force = max_force * thrust
            thruster_pos = node.get_pos(self.vehicle)
            thrust_direction = self.app.render.get_relative_vector(
                node,
                Vec3(0, 0, 1)
            )
            self.physics_node.apply_impulse(
                thrust_direction * real_force * dt,
                thruster_pos,
            )

    def shock(self):
        self.physics_node.apply_impulse(
            Vec3(0,0,0),
            Vec3(random(), random(), random()) * 10,
        )
        self.physics_node.apply_torque_impulse(
            (Vec3(random(), random(), random()) - Vec3(0.5, 0.5, 0.5)) * 10000,
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
        # Camera
        horiz_dist = 20
        cam_offset = Vec3(0, 0, 4)
        focus_offset = Vec3(0, 0, 3)
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

        # Speed
        mps = self.vehicle.physics_node.get_linear_velocity().length()
        kmh = mps * 60 * 60 / 1000
        #print('{:4.1f}'.format(kmh))


class VehicleController:
    def __init__(self, app, vehicle):
        self.app = app
        self.vehicle = vehicle
        self.app.accept("n", self.next_vehicle)
        self.app.accept("r", self.toggle_repulsors)
        self.app.accept("g", self.toggle_gyroscope)
        self.app.accept("s", self.shock)
        self.repulsors_active = False
        self.gyroscope_active = True

    def gather_inputs(self):
        target_orientation = Vec3(0, 0, 0)
        target_rotation = Vec3(0, 0, 0)
        if self.app.mouseWatcherNode.is_button_down(KeyboardButton.left()):
            target_rotation.z += 2
        if self.app.mouseWatcherNode.is_button_down(KeyboardButton.right()):
            target_rotation.z -= 2

        repulsor_activation = 0
        if self.repulsors_active:
            repulsor_activation = 1

        thrust = 0
        if self.app.mouseWatcherNode.is_button_down(KeyboardButton.up()):
            thrust = 1

        self.vehicle.set_inputs(
            {
                TARGET_ORIENTATION: target_orientation,
                TARGET_ROTATION: target_rotation,
                REPULSOR_ACTIVATION: repulsor_activation,
                THRUST: thrust,
            }
        )

    def next_vehicle(self):
        self.app.next_vehicle()

    def set_vehicle(self, vehicle):
        self.vehicle = vehicle

    def toggle_repulsors(self):
        self.repulsors_active = not self.repulsors_active

    def toggle_gyroscope(self):
        self.gyroscope_active = not self.gyroscope_active

    def shock(self):
        self.vehicle.shock()


def main():
    if len(sys.argv) > 1:
        map = "assets/maps/"+sys.argv[1]
        app = GameApp(map)
    else:
        app = GameApp()
    app.run()


if __name__ == '__main__':
    main()
