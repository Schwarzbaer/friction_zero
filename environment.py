from panda3d.core import Vec3

from panda3d.bullet import BulletWorld
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletTriangleMesh
from panda3d.bullet import BulletTriangleMeshShape

from common_vars import FRICTION
from common_vars import AIR_DENSITY
from common_vars import DEFAULT_FRICTION_VALUE
from common_vars import CM_TERRAIN
from common_vars import CM_COLLIDE

from model_data import ModelData


TERRAIN = 'fz_terrain'
SPAWN_POINTS = 'fz_spawn_point'
SKYSPHERE = 'fz_skysphere'
TERRAIN_COLLIDER = 'fz_collision'
GRAVITY = 'gravity'
X = '_x'
Y = '_y'
Z = '_z'


class EnvironmentData(ModelData):
    def read_model(self, node, model_name, specs):
        self.friction = self.get_value(FRICTION, node, specs, default=1.25)
        self.gravity = Vec3(
            self.get_value(GRAVITY+X, node, specs, default=0),
            self.get_value(GRAVITY+Y, node, specs, default=0),
            self.get_value(
                GRAVITY+Z,
                node,
                specs,
                default=-9.81, # Earth gravity
            ),
        )
        self.air_density = self.get_value(
            AIR_DENSITY,
            node,
            specs,
            default=1.225, # Air density, kg/m**3 at 15 degree C and 1 atm
        )


class Environment:
    def __init__(self, app, map_name):
        map_file = 'assets/maps/{}/{}.bam'.format(map_name, map_name)
        map_file_yabee = 'assets/maps/{}/{}_y.bam'.format(map_name, map_name)
        self.app = app

        self.physics_world = BulletWorld()

        node = BulletRigidBodyNode('Ground')
        self.np = self.app.render.attach_new_node(node)
        self.np.setPos(0, 0, 0)
        self.physics_world.attachRigidBody(node)

        self.model = loader.load_model(map_file)
        self.model.reparent_to(self.np)

        try:
            self.model_yabee = loader.load_model(map_file_yabee)
            self.model_yabee.reparent_to(self.model)
        except OSError:
            pass

        self.env_data = EnvironmentData(self.model, map_name, 'maps')

        self.physics_world.setGravity(self.env_data.gravity)

        #sky = self.model.find(SKYSPHERE)
        #sky.reparent_to(base.cam)
        #sky.set_bin('background', 0)
        #sky.set_depth_write(False)
        #sky.set_compass()
        #sky.set_light_off()

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
                terrain_node.set_friction(self.env_data.friction)
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
