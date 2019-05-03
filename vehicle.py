from random import random
from math import pi
from math import cos
from math import isnan
from math import copysign
from functools import reduce

import numpy

from panda3d.core import NodePath
from panda3d.core import VBase3
from panda3d.core import Vec3
from panda3d.core import GeomVertexReader
from panda3d.core import invert
from panda3d.bullet import BulletRigidBodyNode
from panda3d.bullet import BulletConvexHullShape

from direct.actor.Actor import Actor

from common_vars import FRICTION
from common_vars import DEFAULT_FRICTION_VALUE
from common_vars import CM_TERRAIN
from common_vars import CM_VEHICLE
from common_vars import CM_COLLIDE


SPAWN_POINT_CONNECTOR = 'fz_spawn_point_connector'
REPULSOR = 'fz_repulsor'
ACTIVATION_DISTANCE = 'activation_distance'
THRUSTER = 'fz_thruster'
FORCE = 'force'

CURRENT_ORIENTATION = 'current_orientation'
CURRENT_MOVEMENT = 'current_movement'
CURRENT_ROTATION = 'current_rotation'
TARGET_ORIENTATION = 'target_orientation'
TARGET_ROTATION = 'target_rotation'
REPULSOR_ACTIVATION = 'repulsor_activation'
REPULSOR_RAY_ACTIVE = 'repulsor_ray_active'
REPULSOR_RAY_FRAC = 'repulsor_ray_frac'
REPULSOR_RAY_DIR = 'repulsor_ray_dir'
REPULSOR_RAY_POS = 'repulsor_ray_pos'
REPULSOR_RAY_CONTACT = 'repulsor_ray_contact'
REPULSOR_TURNING_ANGLE = 'repulsor_turning_angle'
REPULSOR_TARGET_ORIENTATIONS = 'repulsor_target_orientation'
REPULSOR_OLD_ORIENTATION = 'repulsor_old_orientation'
GYRO_ROTATION = 'gyro_rotation'
THRUST = 'thrust'
THRUSTER_ACTIVATION = 'thruster_activation'
TO_HORIZON = 'to_horizon'
TO_GROUND = 'to_ground'
PASSIVE = 'passive'
ACTIVE_STABILIZATION_ON_GROUND = 'active_stabilization_on_ground'
ACTIVE_STABILIZATION_CUTOFF_ANGLE = 'active_stabilization_cutoff_angle'
ACTIVE_STABILIZATION_IN_AIR = 'active_stabilization_in_air'
ACCELERATE = 'accelerate'
TURN = 'turn'
TURN_LEFT = 'turn_left'
TURN_RIGHT = 'turn_right'
STRAFE = 'strafe'
HOVER = 'hover'
X = '_x'
Y = '_y'
MASS = 'mass'
AIRBRAKE = 'airbrake'


# class RepulsorSensor:
#     def __init__(self, active, fraction, direction, position):
#         self.active = active
#         self.fraction = fraction
#         self.direction = direction
#         self.position = position


class Vehicle:
    def __init__(self, app, model_file):
        self.app = app

        self.model = Actor(model_file)
        puppet = self.app.loader.load_model(model_file)
        puppet.reparentTo(self.model)

        self.physics_node = BulletRigidBodyNode('vehicle')
        friction_node = self.model.find('**/={}'.format(FRICTION))
        friction_str = friction_node.get_tag('friction')
        friction = float(friction_str)
        self.physics_node.set_friction(friction)
        self.physics_node.set_linear_sleep_threshold(0)
        self.physics_node.set_angular_sleep_threshold(0)
        self.physics_node.setCcdMotionThreshold(1e-7)
        self.physics_node.setCcdSweptSphereRadius(0.5)
        mass_node = self.model.find('**/={}'.format(MASS))
        mass_str = mass_node.get_tag('mass')
        mass = float(mass_str)
        self.inertia = mass
        self.physics_node.setMass(mass)
        shape = BulletConvexHullShape()
        for geom_node in self.model.find_all_matches("**/+GeomNode"):
            for geom in geom_node.node().get_geoms():
                vertices = GeomVertexReader(geom.get_vertex_data(), 'vertex')
                while not vertices.is_at_end():
                    v_geom = vertices.getData3f()
                    v_model = self.model.get_relative_point(geom_node, v_geom)
                    shape.add_point(v_model)
        self.physics_node.add_shape(shape)
        self.vehicle = NodePath(self.physics_node)
        self.vehicle.set_collide_mask(CM_VEHICLE | CM_COLLIDE)

        self.model.reparent_to(self.vehicle)

        # Navigational aids
        self.target_node = self.app.loader.load_model('models/zup-axis')
        # Might make a nice GUI interface
        self.target_node.reparent_to(self.model)
        self.target_node.set_scale(1)
        self.target_node.set_render_mode_wireframe()
        self.target_node.hide()

        self.delta_node = self.app.loader.load_model('models/smiley')
        self.delta_node.set_pos(1,10,1)
        self.delta_node.reparent_to(base.cam)
        self.delta_node.hide()

        # Vehicle systems
        self.repulsor_nodes = []
        for repulsor in self.model.find_all_matches('**/{}*'.format(REPULSOR)):
            self.add_repulsor(repulsor)

        self.thruster_nodes = []
        for thruster in self.model.find_all_matches('**/{}*'.format(THRUSTER)):
            self.add_thruster(thruster)

        self.airbrake_state = 0.0
        self.airbrake_factor = 0.5
        # FIXME: This will be replaced by air drag.
        self.physics_node.set_linear_damping(self.airbrake_state)
        # FIXME: Make artist definable
        airbrake_duration = 0.2 # seconds
        self.airbrake_speed = 1 / airbrake_duration

        self.centroid = base.loader.load_model('models/smiley')
        self.centroid.reparent_to(self.vehicle)
        self.centroid.hide()

        self.inputs = {
            # Repulsors
            REPULSOR_ACTIVATION: 0.0,
            ACCELERATE: 0.0,
            TURN: 0.0,
            STRAFE: 0.0,
            HOVER: 0.0,
            # Gyro
            ACTIVE_STABILIZATION_ON_GROUND: PASSIVE,
            ACTIVE_STABILIZATION_CUTOFF_ANGLE: PASSIVE,
            ACTIVE_STABILIZATION_IN_AIR: PASSIVE,
            TARGET_ORIENTATION: Vec3(0, 0, 0),
            # Thrust
            THRUST: 0.0,
            # Airbrake
            AIRBRAKE: 0.0,
        }
        self.sensors = {}
        self.commands = {}

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
        self.repulsor_nodes.append(repulsor)

        # Transcribe tags
        force = float(repulsor.get_tag(FORCE))
        repulsor.set_python_tag(FORCE, force)
        activation_distance = float(repulsor.get_tag(ACTIVATION_DISTANCE))
        repulsor.set_python_tag(ACTIVATION_DISTANCE, activation_distance)

        animation_tags = [ACCELERATE, TURN_LEFT, TURN_RIGHT, STRAFE, HOVER]
        for tag in animation_tags:
            tag_x = repulsor.get_tag(tag+X)
            if tag_x == '':
                tag_x = 0.0
            else:
                tag_x = float(tag_x)
            tag_y = repulsor.get_tag(tag+Y)
            if tag_y == '':
                tag_y = 0.0
            else:
                tag_y = float(tag_y)
            angle = VBase3(tag_x, tag_y, 0)
            repulsor.set_python_tag(tag, angle)
        # FIXME: Make it artist-definable
        repulsor.set_python_tag(REPULSOR_TURNING_ANGLE, 540)
        repulsor.set_python_tag(REPULSOR_OLD_ORIENTATION, Vec3(0, 0, 0))

        ground_contact = self.app.loader.load_model("assets/repulsorhit.egg")
        ground_contact.set_scale(1)
        ground_contact.reparent_to(self.app.render)
        repulsor.set_python_tag('ray_end', ground_contact)

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
        self.apply_airbrake()

    def gather_sensors(self):
        # Gather ray data on collision with ground from each repulsor
        repulsor_ray_active = []
        repulsor_ray_frac = []
        repulsor_ray_dir = []
        repulsor_ray_pos = []
        repulsor_ray_contact = []
        for repulsor in self.repulsor_nodes:
            max_distance = repulsor.get_python_tag(ACTIVATION_DISTANCE)
            repulsor_pos = repulsor.get_pos(self.app.render)
            repulsor_ray_pos.append(repulsor.get_pos(self.vehicle))
            repulsor_dir = self.app.render.get_relative_vector(
                repulsor,
                Vec3(0, 0, -max_distance),
            )
            repulsor_ray_dir.append(repulsor_dir)
            # FIXME: `self.app.environment.physics_world` is ugly.
            feeler = self.app.environment.physics_world.ray_test_closest(
                repulsor_pos,
                repulsor_pos + repulsor_dir,
                CM_TERRAIN,
            )
            if feeler.has_hit():
                repulsor_ray_active.append(True)
                ray_frac = feeler.get_hit_fraction()
                repulsor_ray_frac.append(ray_frac)
                repulsor_ray_contact.append(
                    self.vehicle.get_relative_point(
                        self.app.render,
                        feeler.get_hit_pos(),
                    )
                )
            else:
                repulsor_ray_active.append(False)
                ray_frac = 1.0
                repulsor_ray_frac.append(ray_frac)
                repulsor_ray_contact.append(None)

        self.sensors = {
            CURRENT_ORIENTATION: self.vehicle.get_hpr(self.app.render),
            CURRENT_MOVEMENT: self.physics_node.get_linear_velocity(),
            CURRENT_ROTATION: self.physics_node.get_angular_velocity(),
            REPULSOR_RAY_ACTIVE: repulsor_ray_active,
            REPULSOR_RAY_POS: repulsor_ray_pos,
            REPULSOR_RAY_DIR: repulsor_ray_dir,
            REPULSOR_RAY_FRAC: repulsor_ray_frac,
            REPULSOR_RAY_CONTACT: repulsor_ray_contact,
        }

    def ecu(self):
        # Repulsors
        repulsor_activation = [self.inputs[REPULSOR_ACTIVATION]
                               for _ in self.repulsor_nodes]

        # Calculate effective repulsor motion blend values
        accelerate = self.inputs[ACCELERATE]
        turn = self.inputs[TURN]
        strafe = self.inputs[STRAFE]
        hover = self.inputs[HOVER]
        length = sum([abs(accelerate), abs(turn), abs(strafe), hover])
        if length > 1:
            accelerate /= length
            turn /= length
            strafe /= length
            hover /= length
        # Split the turn signal into animation blend factors
        if turn > 0.0:
            turn_left = 0.0
            turn_right = turn
        else:
            turn_left = -turn
            turn_right = 0.0
        # Blend the repulsor angle
        repulsor_target_angles = []
        for repulsor in self.repulsor_nodes:
            acc_angle = -(repulsor.get_python_tag(ACCELERATE)) * accelerate
            turn_left_angle = repulsor.get_python_tag(TURN_LEFT) * turn_left
            turn_right_angle = repulsor.get_python_tag(TURN_RIGHT) * turn_right
            strafe_angle = repulsor.get_python_tag(STRAFE) * strafe
            hover_angle = repulsor.get_python_tag(HOVER) * hover
            angle = acc_angle + turn_left_angle + turn_right_angle + \
                    strafe_angle + hover_angle
            repulsor_target_angles.append(angle)

        # Gyroscope:
        gyro_rotation = self.ecu_gyro_stabilization()

        # Thrusters
        thruster_activation = [self.inputs[THRUST]
                               for _ in self.thruster_nodes]

        # Airbrake
        airbrake = self.inputs[AIRBRAKE]

        self.commands = {
            REPULSOR_ACTIVATION: repulsor_activation,
            REPULSOR_TARGET_ORIENTATIONS: repulsor_target_angles,
            GYRO_ROTATION: gyro_rotation,
            THRUSTER_ACTIVATION: thruster_activation,
            AIRBRAKE: airbrake,
        }

    def ecu_gyro_stabilization(self):
        local_up = False
        # Find the local up
        contacts = [pos
                    for pos, active in zip(
                            self.sensors[REPULSOR_RAY_CONTACT],
                            self.sensors[REPULSOR_RAY_ACTIVE],
                    )
                    if active and self.inputs[REPULSOR_ACTIVATION]]
        if len(contacts) > 2:
            local_up = True
            target_mode = self.inputs[ACTIVE_STABILIZATION_ON_GROUND]
            # We can calculate a local up
            centroid = reduce(lambda a,b: a+b, contacts) / len(contacts)
            covariance = [[c.x, c.y, c.z]
                          for c in [contact - centroid for contact in contacts]
            ][0:3] # We need exactly 3, no PCA yet. :(
            eigenvalues, eigenvectors = numpy.linalg.eig(
                numpy.array(covariance)
            )
            # FIXME: These few lines look baaaad...
            indexed_eigenvalues = enumerate(eigenvalues)
            def get_magnitude(indexed_element):
                index, value = indexed_element
                return abs(value)
            sorted_indexed_eigenvalues = sorted(
                indexed_eigenvalues,
                key=get_magnitude,
            )
            # The smallest eigenvalue leads to the ground plane's normal
            up_vec_idx, _ = sorted_indexed_eigenvalues[0]
            up_vec = VBase3(*eigenvectors[:, up_vec_idx])
            # Point into the upper half-space
            if up_vec.z < 0:
                up_vec *= -1
            # Calculate the forward of the centroid
            centroid_forward = Vec3(0,1,0) - up_vec * (Vec3(0,1,0).dot(up_vec))
            # FIXME: Huh?
            forward_planar = centroid_forward - up_vec * (centroid_forward.dot(up_vec))
            # Now let's orient and place the centroid
            self.centroid.set_pos(self.vehicle, (0, 0, 0))
            self.centroid.heads_up(forward_planar, up_vec)
            self.centroid.set_pos(self.vehicle, centroid)
        else:
            local_up = False

        # Active stabilization and angular dampening
        tau = 0.2  # Seconds until target orientation is reached

        if local_up:
            target_mode = self.inputs[ACTIVE_STABILIZATION_ON_GROUND]
        else:
            target_mode = self.inputs[ACTIVE_STABILIZATION_IN_AIR]

        if target_mode == TO_HORIZON:
            # Stabilize to the current heading, but in a horizontal
            # orientation
            self.target_node.set_hpr(
                self.app.render,
                self.vehicle.get_h(),
                0,
                0,
            )
        elif target_mode == TO_GROUND:
            self.target_node.set_hpr(self.centroid, (0, 0, 0))

        if target_mode != PASSIVE:
            xyz_driver_modification = self.inputs[TARGET_ORIENTATION]
            hpr_driver_modification = VBase3(
                xyz_driver_modification.z,
                xyz_driver_modification.x,
                xyz_driver_modification.y,
            )
            self.target_node.set_hpr(
                self.target_node,
                hpr_driver_modification,
            )

            # Now comes the math.
            orientation = self.vehicle.get_quat(self.app.render)
            target_orientation = self.target_node.get_quat(self.app.render)
            delta_orientation = target_orientation * invert(orientation)
            self.delta_node.set_quat(invert(delta_orientation))

            delta_angle = delta_orientation.get_angle_rad()
            if abs(delta_angle) < (pi/360*0.1) or isnan(delta_angle):
                delta_angle = 0
                axis_of_torque = VBase3(0, 0, 0)
            else:
                axis_of_torque = delta_orientation.get_axis()
                axis_of_torque.normalize()
                axis_of_torque = self.app.render.get_relative_vector(
                    self.vehicle,
                    axis_of_torque,
                )
            if delta_angle > pi:
                delta_angle -= 2*pi

            # If the mass was standing still, this would be the velocity that
            # has to be reached to achieve the targeted orientation in tau
            # seconds.
            target_angular_velocity = axis_of_torque * delta_angle / tau
        else: # `elif target_mode == PASSIVE:`, since we moght want an OFF mode
            # Passive stabilization, so this is the pure commanded impulse
            target_angular_velocity = self.app.render.get_relative_vector(
                self.vehicle,
                self.inputs[TARGET_ORIENTATION] * tau / pi,
            )

        # But we also have to cancel out the current velocity for that.
        angular_velocity = self.physics_node.get_angular_velocity()
        countering_velocity = -angular_velocity

        # An impulse of 1 causes an angular velocity of 2.5 rad on a unit mass,
        # so we have to adjust accordingly.
        target_impulse = target_angular_velocity / 2.5 * self.inertia
        countering_impulse = countering_velocity / 2.5 * self.inertia

        # Now just sum those up, and we have the impulse that needs to be
        # applied to steer towards target.
        impulse = target_impulse + countering_impulse
        return impulse

    def apply_repulsors(self):
        dt = globalClock.dt
        repulsor_data = zip(
            self.repulsor_nodes,
            self.sensors[REPULSOR_RAY_ACTIVE],
            self.sensors[REPULSOR_RAY_FRAC],
            self.commands[REPULSOR_ACTIVATION],
            self.commands[REPULSOR_TARGET_ORIENTATIONS]
        )
        for node, active, frac, activation, angle in repulsor_data:
            # Repulse in current orientation
            if active and activation:
                # Repulsor power at zero distance
                base_strength = node.get_python_tag(FORCE)
                # Effective fraction of repulsors force
                transfer_frac = cos(0.5*pi * frac)
                # Effective repulsor force
                strength = base_strength * activation * transfer_frac
                # Resulting impulse
                impulse_dir = Vec3(0, 0, 1)
                impulse_dir_world = self.app.render.get_relative_vector(
                    node,
                    impulse_dir,
                )
                impulse = impulse_dir_world * strength
                # Apply
                repulsor_pos = node.get_pos(self.vehicle)
                # FIXME! The position at which an impulse is applied seems to be
                # centered around node it is applied to, but offset in the world
                # orientation. So, right distance, wrong angle. This is likely a
                # bug in Panda3D's Bullet wrapper. Or an idiosyncracy of Bullet.
                self.physics_node.apply_impulse(
                    impulse * dt,
                    self.app.render.get_relative_vector(
                        self.vehicle,
                        repulsor_pos,
                    ),
                )

                # Contact visualization node
                max_distance = node.get_python_tag(ACTIVATION_DISTANCE)
                contact_distance = -impulse_dir_world * max_distance * frac
                contact_node = node.get_python_tag('ray_end')
                contact_node.set_pos(
                    node.get_pos(self.app.render) + contact_distance,
                )
                #contact_node.set_hpr(node, 0, -90, 0) # Look towards repulsor
                #contact_node.set_hpr(0, -90, 0) # Look up
                contact_node.set_hpr(0, -90, contact_node.getR()+4) # Look up and rotate
                contact_node.set_scale(1-frac)

                contact_node.show()
            else:
                node.get_python_tag('ray_end').hide()
            # Reorient
            old_hpr = node.get_python_tag(REPULSOR_OLD_ORIENTATION)
            want_hpr = VBase3(angle.z, angle.x, angle.y)
            delta_hpr = want_hpr - old_hpr
            max_angle = node.get_python_tag(REPULSOR_TURNING_ANGLE) * dt
            if delta_hpr.length() > max_angle:
                delta_hpr = delta_hpr / delta_hpr.length() * max_angle
            new_hpr = old_hpr + delta_hpr
            node.set_hpr(new_hpr)
            node.set_python_tag(REPULSOR_OLD_ORIENTATION, new_hpr)

    def apply_gyroscope(self):
        impulse = self.commands[GYRO_ROTATION]
        # Clamp the impulse to what the "motor" can produce.
        max_impulse = 0.8 * 1000
        if impulse.length() > max_impulse:
            clamped_impulse = impulse / impulse.length() * max_impulse
        else:
            clamped_impulse = impulse

        self.physics_node.apply_torque_impulse(impulse)

    def apply_thrusters(self):
        dt = globalClock.dt
        thruster_data = zip(
            self.thruster_nodes,
            self.commands[THRUSTER_ACTIVATION],
        )
        for node, thrust in thruster_data:
            max_force = node.get_python_tag(FORCE)
            real_force = max_force * thrust
            # FIXME: See repulsors above for the shortcoming that this suffers
            thruster_pos = node.get_pos(self.vehicle)
            thrust_direction = self.app.render.get_relative_vector(
                node,
                Vec3(0, 0, 1)
            )
            self.physics_node.apply_impulse(
                thrust_direction * real_force * dt,
                thruster_pos,
            )

    def apply_airbrake(self):
        dt = globalClock.dt
        target = self.commands[AIRBRAKE]
        max_movement = self.airbrake_speed * dt
        # Clamp change to available speed
        delta = target - self.airbrake_state
        if abs(delta) > max_movement:
            self.airbrake_state += copysign(max_movement, delta)
        else:
            self.airbrake_state = target
        if self.airbrake_state > 1.0:
            self.airbrake_state = 1.0
        if self.airbrake_state < 0.0:
            self.airbrake_state = 0.0
        self.model.pose(AIRBRAKE, self.airbrake_state)
        # FIXME: This will be replaced by air drag.
        self.physics_node.set_linear_damping(self.airbrake_state * self.airbrake_factor)

    def shock(self, x=0, y=0, z=0):
        self.physics_node.apply_impulse(
            Vec3(0,0,0),
            Vec3(random(), random(), random()) * 10,
        )
        self.physics_node.apply_torque_impulse(Vec3(x, y, z))
