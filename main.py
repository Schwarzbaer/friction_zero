import os
import sys
from math import cos, pi

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
from panda3d.core import CollisionPlane
from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletBoxShape
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape
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

        self.environment = Environment(self, "maps/hills.bam")

        vehicle = Vehicle(self, "cars/Ricardeaut_Magnesium.bam")
        vehicle.place(Vec3(0, 0, 2))
        camera = CameraController(self, base.cam, vehicle)

        base.task_mgr.add(self.run_repulsors, 'run repulsors', sort=0)
        base.task_mgr.add(vehicle.apply_repulsors, 'apply repulsors', sort=1)
        base.task_mgr.add(self.update_physics, 'physics', sort=2)
        base.task_mgr.add(camera.update, "camera", sort=3)

    def run_repulsors(self, task):
        self.repulsor_traverser.traverse(base.render)
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

def triangleShape(model, dynamic=False):
    mesh = BulletTriangleMesh()
    geomNodeCollection = model.findAllMatches('**/+GeomNode')
    for nodePath in geomNodeCollection:
        geomNode = nodePath.node()
        for i in range(geomNode.getNumGeoms()):
            geom = geomNode.getGeom(i)
            state = geomNode.getGeomState(i)
            mesh.addGeom(geom)
    shape = BulletTriangleMeshShape(mesh, dynamic=dynamic)
    return shape

class Environment:
    def __init__(self, app, model_file):
        self.app = app
        self.model = app.loader.load_model(model_file)
        self.physics_node = BulletRigidBodyNode('environment')

        shape = triangleShape(self.model, False)
        node = BulletRigidBodyNode('Ground')
        node.addShape(shape)
        self.app.physics_world.attachRigidBody(node)
        self.environment = NodePath(self.physics_node)
        self.model.reparent_to(self.environment)
        self.environment.reparent_to(self.app.render)
        self.app.physics_world.attachRigidBody(self.physics_node)

class Vehicle:
    def __init__(self, app, model_file):
        self.app = app

        model = app.loader.load_model(model_file)

        self.physics_node = BulletRigidBodyNode('vehicle')
        self.physics_node.setMass(5.0)

        #shape = BulletBoxShape(Vec3(2, 3, 1))
        collision_solid = model.find("collision_solid")
        try:
            collision_solid.hide()
        except AssertionError:
            pass
        shape = triangleShape(collision_solid, dynamic=True)

        self.physics_node.setLinearSleepThreshold(0)
        self.physics_node.setAngularSleepThreshold(0)
        self.physics_node.setMass(100.0)
        # mesh = BulletTriangleMesh()
        # for geom in model.node().get_child(0).get_geoms():
        #     mesh.addGeom(geom)
        # shape = BulletTriangleMeshShape(mesh, dynamic=False)
        #shape = BulletBoxShape(Vec3(0.5, 0.5, 0.5))
        self.physics_node.addShape(shape)
        self.vehicle = NodePath(self.physics_node)
        model.reparent_to(self.vehicle)

        self.repulsor_queue = CollisionHandlerQueue()
        self.add_repulsor(Vec3( 0.45,  0.45, -0.3), Vec3( 0.5,  0.5, -1))
        self.add_repulsor(Vec3(-0.45,  0.45, -0.3), Vec3(-0.5,  0.5, -1))
        self.add_repulsor(Vec3( 0.45, -0.45, -0.3), Vec3( 0.5, -0.5, -1))
        self.add_repulsor(Vec3(-0.45, -0.45, -0.3), Vec3(-0.5, -0.5, -1))
        # self.add_repulsor(Vec3( 0.4,  0.4, -0.4), Vec3(0, 0, -1))
        # self.add_repulsor(Vec3(-0.4,  0.4, -0.4), Vec3(0, 0, -1))
        # self.add_repulsor(Vec3( 0.4, -0.4, -0.4), Vec3(0, 0, -1))
        # self.add_repulsor(Vec3(-0.4, -0.4, -0.4), Vec3(0, 0, -1))

    def add_repulsor(self, coord, vec):
        repulsor_solid = CollisionRay(Vec3(0, 0, 0), vec)
        repulsor_node = CollisionNode('repulsor')
        repulsor_node.add_solid(repulsor_solid)
        repulsor_node.set_into_collide_mask(0)
        repulsor_np = self.vehicle.attach_new_node(repulsor_node)
        repulsor_np.set_pos(coord)
        repulsor_np.show()
        self.app.repulsor_traverser.addCollider(
            repulsor_np, self.repulsor_queue,
        )

    def apply_repulsors(self, task):
        dt = globalClock.dt
        for entry in self.repulsor_queue.entries:
            # Distance below which the repulsor strength is > 0
            activation_distance = 3
            repulsor_feeler = entry.get_surface_point(entry.from_node_path)
            if repulsor_feeler.length() < activation_distance:
                # Direction of the impulse
                impulse_vec = -repulsor_feeler / repulsor_feeler.length()
                # Repulsor power at zero distance
                base_strength = 400
                # Fraction of the repulsor beam above the ground
                activation_frac = repulsor_feeler.length() / activation_distance
                # Effective fraction of repulsors force
                activation = cos(0.5*pi * activation_frac)
                # Effective repulsor force
                strength = activation * base_strength
                # Resulting impulse
                impulse = impulse_vec * strength
                # Apply
                repulsor_pos = entry.from_node_path.get_pos(self.vehicle)
                self.physics_node.apply_impulse(impulse * dt, repulsor_pos)
        return task.cont

    def place(self, coordinate):
        self.vehicle.reparent_to(self.app.environment.model)
        self.vehicle.set_pos(coordinate)
        self.app.physics_world.attachRigidBody(self.physics_node)

    def np(self):
        return self.vehicle


class CameraController:
    def __init__(self, app, camera, vehicle):
        self.app = app
        self.camera = camera
        self.vehicle = vehicle
        self.camera.reparent_to(self.app.render)

    def update(self, task):
        horiz_dist = 7
        cam_offset = Vec3(0, -10, 10)
        focus_offset = Vec3(0, -2, 2)
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

def main():
    app = GameApp()
    app.run()

if __name__ == '__main__':
    main()
