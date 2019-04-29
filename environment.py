from panda3d.core import Vec3

from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape

from common_vars import FRICTION
from common_vars import DEFAULT_FRICTION_VALUE
from common_vars import CM_TERRAIN
from common_vars import CM_COLLIDE


TERRAIN = 'fz_terrain'
SPAWN_POINTS = 'fz_spawn_point'
SKYSPHERE = 'fz_skysphere'
TERRAIN_COLLIDER = 'fz_collision'
GRAVITY = 'gravity'


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
        # FIXME: This quietly eats what should be an error, and tested for by a
        # model linter
        if gravity_str == '':
            gravity = Vec3(0, 0, -9.81)
        else:
            gravity = Vec3(0, 0, -float(gravity_str))
        self.physics_world.setGravity(gravity)

        sky = self.model.find(SKYSPHERE)
        sky.reparent_to(base.cam)
        sky.set_bin('background', 0)
        sky.set_depth_write(False)
        sky.set_compass()
        sky.set_light_off()

        # Bullet collision mesh
        collision_solids = self.model.find_all_matches(
            '{}*'.format(TERRAIN_COLLIDER)
        )

        #collision_solids.hide()

        for collision_solid in collision_solids:
            collision_solid.flatten_strong()
            for geom_node in collision_solid.find_all_matches('**/+GeomNode'):
                mesh = BulletTriangleMesh()
                for geom in geom_node.node().get_geoms():
                    mesh.addGeom(geom)
                shape = BulletTriangleMeshShape(mesh, dynamic=False)
                terrain_node = BulletRigidBodyNode('terrain')
                terrain_node.add_shape(shape)
                friction_node = collision_solid.find('**/={}'.format(FRICTION))
                friction_str = friction_node.get_tag('friction')
                if len(friction_str) == 0:
                    friction = DEFAULT_FRICTION_VALUE
                else:
                    friction = float(friction_str)
                terrain_node.set_friction(friction)
                terrain_np = geom_node.attach_new_node(terrain_node)
                terrain_np.set_collide_mask(CM_TERRAIN | CM_COLLIDE)
                self.physics_world.attach_rigid_body(terrain_node)

    def add_physics_node(self, node):
        self.physics_world.attach_rigid_body(node)

    def update_physics(self):
        dt = globalClock.dt
        # FIXME: Pull from settings
        min_frame_rate = 30
        max_frame_time = 1.0 / min_frame_rate
        if dt <= max_frame_time:
            self.physics_world.do_physics(dt)
        else:
            self.physics_world.do_physics(max_frame_time)

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
