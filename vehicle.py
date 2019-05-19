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

from model_data import ModelData
from tools import adjust_within_limit


SPAWN_POINT_CONNECTOR = 'fz_spawn_point_connector'
VEHICLE = 'fz_vehicle'
MAX_GYRO_TORQUE = 'max_gyro_torque'
REPULSOR = 'fz_repulsor'
ACTIVATION_DISTANCE = 'activation_distance'
THRUSTER_NODE = 'fz_thruster'
THRUSTER_SOUND = 'thruster'
THRUSTER_POWER = 'thruster_power'  # Current power [0-1] of a thruster
FORCE = 'force' # Maximum force in N produced by a thruster
THRUSTER_HEATING = 'heating'
THRUSTER_COOLING = 'cooling'
THRUSTER_RAMP_TIME = 'ramp_time'
DRAG_COEFFICIENT_X = 'drag_coefficient_x'
DRAG_COEFFICIENT_Y = 'drag_coefficient_y'
DRAG_COEFFICIENT_Z = 'drag_coefficient_z'
DRAG_AREA_X = 'drag_area_x'
DRAG_AREA_Y = 'drag_area_y'
DRAG_AREA_Z = 'drag_area_z'
AIRBRAKE_AREA_X = 'airbrake_area_x'
AIRBRAKE_AREA_Y = 'airbrake_area_y'
AIRBRAKE_AREA_Z = 'airbrake_area_z'
STABILIZER_FINS_AREA_X = 'stabilizer_fins_area_x'
STABILIZER_FINS_AREA_Y = 'stabilizer_fins_area_y'
STABILIZER_FINS_AREA_Z = 'stabilizer_fins_area_z'
CURRENT_ORIENTATION = 'current_orientation'
CURRENT_MOVEMENT = 'current_movement'
CURRENT_ROTATION = 'current_rotation'
TARGET_ORIENTATION = 'target_orientation'
TARGET_ROTATION = 'target_rotation'
REPULSOR_ACTIVATION = 'repulsor_activation'
REPULSOR_DATA = 'repulsor_data'
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
LOCAL_UP = 'local_up'
FLIGHT_HEIGHT = 'flight_height'
CLIMB_SPEED = 'climb_speed'
TARGET_FLIGHT_HEIGHT = 'target_flight_height'
TARGET_FLIGHT_HEIGHT_TAU = 'target_flight_height_tau'
HEIGHT_OVER_TARGET = 'height_over_target'
HEIGHT_OVER_TARGET_PROJECTED = 'height_over_target_projected'
REPULSOR_POWER_FRACTION_NEEDED = 'power_frac_needed'
ACCELERATE = 'accelerate'
TURN = 'turn'
TURN_LEFT = 'turn_left'
TURN_RIGHT = 'turn_right'
STRAFE = 'strafe'
HOVER = 'hover'
FULL_REPULSORS = 'full_repulsors'
X = '_x'
Y = '_y'
MASS = 'mass'
LOCAL_DRAG_FORCE = 'local_drag_force'
AIRBRAKE = 'airbrake'
AIRBRAKE_DURATION = 'airbrake_duration'
STABILIZER_FINS = 'stabilizer_fins'
STABILIZER_FINS_DURATION = 'stabilizer_fins_duration'


class VehicleData(ModelData):
    def read_model(self, model, model_name, specs):
        # Body data
        if VEHICLE not in specs:
            specs[VEHICLE] = {}
        body_specs = specs[VEHICLE]
        body_node = model.find('**/{}'.format(VEHICLE))

        self.friction = self.get_value(FRICTION, body_node, body_specs)
        self.mass = self.get_value(MASS, body_node, body_specs)
        self.max_gyro_torque = self.get_value(
            MAX_GYRO_TORQUE,
            body_node,
            body_specs,
        )
        self.airbrake_duration = self.get_value(
            AIRBRAKE_DURATION,
            body_node,
            body_specs,
        )
        self.stabilizer_fins_duration = self.get_value(
            STABILIZER_FINS_DURATION,
            body_node,
            body_specs,
        )

        # Aerodynamic properties
        drag_coefficient_x = self.get_value(
            DRAG_COEFFICIENT_X,
            body_node,
            body_specs,
        )
        drag_coefficient_y = self.get_value(
            DRAG_COEFFICIENT_Y,
            body_node,
            body_specs,
        )
        drag_coefficient_z = self.get_value(
            DRAG_COEFFICIENT_Z,
            body_node,
            body_specs,
        )
        self.drag_coefficient = Vec3(
            drag_coefficient_x,
            drag_coefficient_y,
            drag_coefficient_z,
        )
        drag_area_x = self.get_value(
            DRAG_AREA_X,
            body_node,
            body_specs,
        )
        drag_area_y = self.get_value(
            DRAG_AREA_Y,
            body_node,
            body_specs,
        )
        drag_area_z = self.get_value(
            DRAG_AREA_Z,
            body_node,
            body_specs,
        )
        self.drag_area = Vec3(
            drag_area_x,
            drag_area_y,
            drag_area_z,
        )
        airbrake_area_x = self.get_value(
            AIRBRAKE_AREA_X,
            body_node,
            body_specs,
        )
        airbrake_area_y = self.get_value(
            AIRBRAKE_AREA_Y,
            body_node,
            body_specs,
        )
        airbrake_area_z = self.get_value(
            AIRBRAKE_AREA_Z,
            body_node,
            body_specs,
        )
        self.airbrake_area = Vec3(
            airbrake_area_x,
            airbrake_area_y,
            airbrake_area_z,
        )
        stabilizer_fins_area_x = self.get_value(
            STABILIZER_FINS_AREA_X,
            body_node,
            body_specs,
        )
        stabilizer_fins_area_y = self.get_value(
            STABILIZER_FINS_AREA_Y,
            body_node,
            body_specs,
        )
        stabilizer_fins_area_z = self.get_value(
            STABILIZER_FINS_AREA_Z,
            body_node,
            body_specs,
        )
        self.stabilizer_fins_area = Vec3(
            stabilizer_fins_area_x,
            stabilizer_fins_area_y,
            stabilizer_fins_area_z,
        )

        # Sub-nodes for vehicle systems
        self.repulsor_nodes = model.find_all_matches(
            '**/{}*'.format(REPULSOR),
        )
        for node in self.repulsor_nodes:
            node_name = node.get_name()
            if node_name not in specs:
                specs[node_name] = {}
            repulsor_specs = specs[node_name]
            self.transcribe_repulsor_tags(node, repulsor_specs)

        self.thruster_nodes = model.find_all_matches(
            '**/{}*'.format(THRUSTER_NODE),
        )
        for node in self.thruster_nodes:
            node_name = node.get_name()
            if node_name not in specs:
                specs[node_name] = {}
            thruster_specs = specs[node_name]
            self.transcribe_thruster_tags(node, thruster_specs)

    def transcribe_repulsor_tags(self, node, specs):
        force = self.get_value(FORCE, node, specs)
        node.set_python_tag(FORCE, force)

        activation_distance = self.get_value(ACTIVATION_DISTANCE, node, specs)
        node.set_python_tag(ACTIVATION_DISTANCE, activation_distance)

        animation_tags = [ACCELERATE, TURN_LEFT, TURN_RIGHT, STRAFE, HOVER]
        for tag in animation_tags:
            tag_x = self.get_value(tag+X, node, specs, default=0.0)
            tag_y = self.get_value(tag+Y, node, specs, default=0.0)
            angle = VBase3(tag_x, tag_y, 0)
            node.set_python_tag(tag, angle)
        # FIXME: Make it artist-definable
        repulsor_turning_angle = self.get_value(
            REPULSOR_TURNING_ANGLE,
            node,
            specs,
            default=540,
        )
        node.set_python_tag(REPULSOR_TURNING_ANGLE, repulsor_turning_angle)
        node.set_python_tag(REPULSOR_OLD_ORIENTATION, Vec3(0, 0, 0))

    def transcribe_thruster_tags(self, node, specs):
        force = self.get_value(FORCE, node, specs)
        node.set_python_tag(FORCE, force)
        heating = self.get_value(THRUSTER_HEATING, node, specs)
        node.set_python_tag(THRUSTER_HEATING, heating)
        cooling = self.get_value(THRUSTER_COOLING, node, specs)
        node.set_python_tag(THRUSTER_COOLING, cooling)
        ramp_time = self.get_value(THRUSTER_RAMP_TIME, node, specs)
        node.set_python_tag(THRUSTER_RAMP_TIME, ramp_time)


class RepulsorData:
    def __init__(self):
        self.active = None
        self.fraction = None
        self.direction = None
        self.position = None
        self.contact = None

    def __repr__(self):
        r = ("active {}, frac {}, dir {}, pos {}, contact {}"
             "".format(
                 self.active,
                 self.fraction,
                 self.direction,
                 self.position,
                 self.contact,
             )
        )
        return r


class Vehicle:
    def __init__(self, app, model_name):
        model_file_name = 'assets/cars/{}/{}.bam'.format(model_name, model_name)
        self.app = app

        def animation_path(model, animation):
            base_path = 'assets/cars/animations/{}/{}-{}.bam'
            return base_path.format(model, model, animation)

        self.model = Actor(
            model_file_name,
            {
            #    AIRBRAKE: 'assets/cars/animations/{}-{}.bam'.format(model_name, AIRBRAKE),
            #    AIRBRAKE: animation_path(model_name, AIRBRAKE),
            #    STABILIZER_FINS: animation_path(model_name, STABILIZER_FINS),
            }
        )
        self.model.enableBlend()
        self.model.setControlEffect(AIRBRAKE, 1)
        self.model.setControlEffect(STABILIZER_FINS, 1)
        # FIXME: This code fails due to a bug in Actor
        # airbrake_joints = [joint.name
        #                    for joint in self.model.getJoints()
        #                    if joint.name.startswith(AIRBRAKE)
        # ]
        # self.model.makeSubpart(AIRBRAKE, airbrake_joints)
        # stabilizer_joints = [joint.name
        #                      for joint in self.model.getJoints()
        #                      if joint.name.startswith(STABILIZER_FINS)
        # ]
        # self.model.makeSubpart(STABILIZER_FINS, stabilizer_joints)

        puppet = self.app.loader.load_model(model_file_name)
        puppet.find("**/armature").hide()
        puppet.reparentTo(self.model)

        # Get the vehicle data
        self.vehicle_data = VehicleData(puppet, model_name, 'cars')

        # Configure the physics node
        self.physics_node = BulletRigidBodyNode('vehicle')
        self.physics_node.set_friction(self.vehicle_data.friction)
        self.physics_node.set_linear_sleep_threshold(0)
        self.physics_node.set_angular_sleep_threshold(0)
        self.physics_node.setCcdMotionThreshold(1e-7)
        self.physics_node.setCcdSweptSphereRadius(0.5)
        self.physics_node.setMass(self.vehicle_data.mass)
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
        self.target_node.reparent_to(self.model)
        self.target_node.set_scale(1)
        self.target_node.set_render_mode_wireframe()
        self.target_node.hide()

        self.delta_node = self.app.loader.load_model('models/smiley')
        self.delta_node.set_pos(1,10,1)
        self.delta_node.reparent_to(base.cam)
        self.delta_node.hide()

        self.airbrake_state = 0.0
        self.airbrake_factor = 0.5
        self.airbrake_speed = 1 / self.vehicle_data.airbrake_duration
        self.stabilizer_fins_state = 0.0
        self.stabilizer_fins_speed = 1 / self.vehicle_data.stabilizer_fins_duration

        self.centroid = base.loader.load_model('models/smiley')
        self.centroid.reparent_to(self.vehicle)
        self.centroid.hide()

        # Thruster limiting
        self.thruster_state = 0.0
        self.thruster_heat = 0.0

        for repulsor in self.vehicle_data.repulsor_nodes:
            self.add_repulsor(repulsor)
        for thruster in self.vehicle_data.thruster_nodes:
            self.add_thruster(thruster, model_name)

        # ECU data storage from frame to frame
        self.last_flight_height = None

        # FIXME: Move into a default controller
        self.inputs = {
            # Repulsors
            REPULSOR_ACTIVATION: 0.0,
            ACCELERATE: 0.0,
            TURN: 0.0,
            STRAFE: 0.0,
            HOVER: 0.0,
            FULL_REPULSORS: False,
            # Gyro
            ACTIVE_STABILIZATION_ON_GROUND: PASSIVE,
            ACTIVE_STABILIZATION_CUTOFF_ANGLE: PASSIVE,
            ACTIVE_STABILIZATION_IN_AIR: PASSIVE,
            TARGET_ORIENTATION: Vec3(0, 0, 0),
            # Thrust
            THRUST: 0.0,
            # Air foils
            AIRBRAKE: 0.0,
            STABILIZER_FINS: 0.0,
        }
        self.sensors = {}
        self.commands = {}

    def np(self):
        return self.vehicle

    def place(self, spawn_point):
        # FIXME: Pass a root node to __init__ instead
        self.vehicle.reparent_to(self.app.environment.model)
        connector = self.model.find("**/"+SPAWN_POINT_CONNECTOR)
        self.vehicle.set_hpr(-connector.get_hpr(spawn_point))
        self.vehicle.set_pos(-connector.get_pos(spawn_point))
        self.app.environment.add_physics_node(self.physics_node)

    def set_inputs(self, inputs):
        self.inputs = inputs

    def add_repulsor(self, repulsor):
        ground_contact = self.app.loader.load_model("assets/effect/repulsorhit.egg")
        ground_contact.set_scale(1)
        ground_contact.reparent_to(self.app.render)
        repulsor.set_python_tag('ray_end', ground_contact)

    def add_thruster(self, thruster, model_name):
        thruster.set_python_tag(THRUSTER_POWER, 0.0)
        thruster_sound_file = 'assets/cars/{}/{}.wav'.format(
            model_name,
            THRUSTER_SOUND,
        )
        thruster_sound = base.audio3d.load_sfx(thruster_sound_file)
        thruster.set_python_tag(THRUSTER_SOUND, thruster_sound)
        base.audio3d.attach_sound_to_object(
            thruster_sound,
            thruster,
        )
        thruster_sound.set_volume(0)
        thruster_sound.set_play_rate(0)
        thruster_sound.set_loop(True)
        thruster_sound.play()

    def game_loop(self):
        self.gather_sensors()
        self.ecu()
        self.apply_air_drag()
        self.apply_repulsors()
        self.apply_gyroscope()
        self.apply_thrusters()
        self.apply_airbrake()
        self.apply_stabilizer_fins()

    def gather_sensors(self):
        # Gather data repulsor ray collisions with ground
        repulsor_data = []
        for node in self.vehicle_data.repulsor_nodes:
            data = RepulsorData()
            repulsor_data.append(data)
            max_distance = node.get_python_tag(ACTIVATION_DISTANCE)
            data.position = node.get_pos(self.vehicle)
            data.direction = self.app.render.get_relative_vector(
                node,
                Vec3(0, 0, -max_distance),
            )
            # FIXME: `self.app.environment.physics_world` is ugly.
            feeler = self.app.environment.physics_world.ray_test_closest(
                base.render.get_relative_point(self.vehicle, data.position),
                base.render.get_relative_point(self.vehicle, data.position + data.direction),
                CM_TERRAIN,
            )
            if feeler.has_hit():
                data.active = True
                data.fraction = feeler.get_hit_fraction()
                data.contact = self.vehicle.get_relative_point(
                    self.app.render,
                    feeler.get_hit_pos(),
                )
            else:
                data.active = False
                data.fraction = 1.0

        # Find the local ground's normal vector
        local_up = False
        contacts = [data.contact
                    for data in repulsor_data
                    if data.active and self.inputs[REPULSOR_ACTIVATION]]
        if len(contacts) > 2:
            local_up = True
            target_mode = self.inputs[ACTIVE_STABILIZATION_ON_GROUND]
            # We can calculate a local up as the smallest base vector of the
            # point cloud of contacts.
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

            # Flight height for repulsor attenuation
            flight_height = -self.centroid.get_z(self.vehicle)
            if self.last_flight_height is not None:
                climb_speed = (flight_height - self.last_flight_height) / globalClock.dt
            else:
                climb_speed = 0
            self.last_flight_height = flight_height
        else:
            local_up = False
            self.last_flight_height = None
            flight_height = 0.0
            climb_speed = 0.0

        # Air drag
        movement = self.physics_node.get_linear_velocity()

        drag_coefficient = self.vehicle_data.drag_coefficient
        drag_area = self.vehicle_data.drag_area + \
                    self.vehicle_data.airbrake_area * self.airbrake_state + \
                    self.vehicle_data.stabilizer_fins_area * self.stabilizer_fins_state
        air_density = self.app.environment.env_data.air_density
        air_speed = -self.vehicle.get_relative_vector(
            base.render,
            movement,
        )

        # drag_force = 1/2*drag_coefficient*mass_density*area*flow_speed**2
        result = Vec3(air_speed)
        result = (result**3) / result.length()
        result.componentwise_mult(drag_area)
        result.componentwise_mult(self.vehicle_data.drag_coefficient)
        drag_force = result * 1/2 * air_density

        self.sensors = {
            CURRENT_ORIENTATION: self.vehicle.get_hpr(self.app.render),
            CURRENT_MOVEMENT: movement,
            CURRENT_ROTATION: self.physics_node.get_angular_velocity(),
            LOCAL_DRAG_FORCE: drag_force,
            REPULSOR_DATA: repulsor_data,
            LOCAL_UP: local_up,
            FLIGHT_HEIGHT: flight_height,
            CLIMB_SPEED: climb_speed,
        }

    def ecu(self):
        repulsor_target_angles = self.ecu_repulsor_reorientation()
        repulsor_activation, delta_height, projected_delta_height, \
            power_frac_needed = self.ecu_repulsor_activation()
        gyro_rotation = self.ecu_gyro_stabilization()
        thruster_activation = [
            self.inputs[THRUST]
            for _ in self.vehicle_data.thruster_nodes
        ]
        airbrake = self.inputs[AIRBRAKE]
        stabilizer_fins = self.inputs[STABILIZER_FINS]

        self.commands = {
            # Steering commands
            REPULSOR_TARGET_ORIENTATIONS: repulsor_target_angles,
            REPULSOR_ACTIVATION: repulsor_activation,
            GYRO_ROTATION: gyro_rotation,
            THRUSTER_ACTIVATION: thruster_activation,
            AIRBRAKE: airbrake,
            STABILIZER_FINS: stabilizer_fins,
            # ECU data output; Interesting numbers we found along the way.
            HEIGHT_OVER_TARGET: delta_height,
            HEIGHT_OVER_TARGET_PROJECTED: projected_delta_height,
            REPULSOR_POWER_FRACTION_NEEDED: power_frac_needed,
        }

    def ecu_repulsor_reorientation(self):
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
        for node in self.vehicle_data.repulsor_nodes:
            acc_angle = -(node.get_python_tag(ACCELERATE)) * accelerate
            turn_left_angle = node.get_python_tag(TURN_LEFT) * turn_left
            turn_right_angle = node.get_python_tag(TURN_RIGHT) * turn_right
            strafe_angle = node.get_python_tag(STRAFE) * strafe
            hover_angle = node.get_python_tag(HOVER) * hover
            angle = acc_angle + turn_left_angle + turn_right_angle + \
                    strafe_angle + hover_angle
            repulsor_target_angles.append(angle)
        return repulsor_target_angles

    def ecu_repulsor_activation(self):
        # Do we know how high we are flying?
        if self.sensors[LOCAL_UP]:
            tau = self.inputs[TARGET_FLIGHT_HEIGHT_TAU]
            # What would we be at in tau seconds if we weren't using repulsors?
            flight_height = self.sensors[FLIGHT_HEIGHT]
            target_flight_height = self.inputs[TARGET_FLIGHT_HEIGHT]
            delta_height = flight_height - target_flight_height
            gravity_z = self.centroid.get_relative_vector(
                self.app.render,
                self.app.environment.physics_world.get_gravity(),
            ).get_z()
            # Since gravity is an acceleration
            gravity_h = 1/2 * gravity_z * tau ** 2
            climb_rate = self.sensors[CLIMB_SPEED] * tau
            projected_delta_height = delta_height + gravity_h + climb_rate
            # Are we sinking?
            if projected_delta_height <= 0:
                # Our projected height will be under our target height, so we
                # will need to apply the repulsors to make up the difference.
                # How much climb can each repulsor provide at 100% power right
                # now?
                max_powers = [
                    node.get_python_tag(FORCE)
                    for node in self.vehicle_data.repulsor_nodes
                ]
                transferrable_powers = [
                    max_power * cos(0.5*pi * data.fraction)
                    for max_power, data in zip(
                            max_powers, self.sensors[REPULSOR_DATA],
                    )
                ]
                angle_ratios = [
                    cos(node.get_quat(self.vehicle).get_angle_rad())
                    for node in self.vehicle_data.repulsor_nodes
                ]
                angled_powers = [
                    power * ratio
                    for power, ratio in zip(transferrable_powers, angle_ratios)
                ]
                # We don't want to activate the repulsors unevenly, so we'll
                # have to go by the weakest link.
                total_angled_power = min(angled_powers) * len(angled_powers)
                # How high can we climb under 100% repulsor power?
                max_climb = 1/2 * total_angled_power * tau**2 / self.vehicle_data.mass
                # The fraction of power needed to achieve the desired climb
                power_frac_needed = -projected_delta_height / max_climb
                # ...and store it.
                repulsor_activation = [
                    power_frac_needed
                    for _ in self.vehicle_data.repulsor_nodes
                ]
            else:
                # We're not sinking.
                repulsor_activation = [
                    0.0
                    for _ in self.vehicle_data.repulsor_nodes
                ]
                power_frac_needed = 0.0
        else: # We do not have ground contact.
            repulsor_activation = [
                0.0
                for _ in self.vehicle_data.repulsor_nodes
            ]
            delta_height = 0.0
            projected_delta_height = 0.0
            power_frac_needed = 0.0
        # The driver gives 100% repulsor power, no matter how high we are, or
        # whether we even have ground contact.
        if self.inputs[FULL_REPULSORS]:
            repulsor_activation = [
                self.inputs[REPULSOR_ACTIVATION]
                for _ in self.vehicle_data.repulsor_nodes
            ]
        return repulsor_activation, delta_height, projected_delta_height,\
            power_frac_needed

    def ecu_gyro_stabilization(self):
        # Active stabilization and angular dampening
        # FIXME: Get from self.inputs
        tau = 0.2  # Seconds until target orientation is reached

        if self.sensors[LOCAL_UP]:
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
            # Stabilize towards the local up
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
        else: # `elif target_mode == PASSIVE:`, since we might want an OFF mode
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
        target_impulse = target_angular_velocity / 2.5 * self.vehicle_data.mass
        countering_impulse = countering_velocity / 2.5 * self.vehicle_data.mass

        # Now just sum those up, and we have the impulse that needs to be
        # applied to steer towards target.
        impulse = target_impulse + countering_impulse
        return impulse

    def apply_air_drag(self):
        drag_force = self.sensors[LOCAL_DRAG_FORCE]
        global_drag_force = base.render.get_relative_vector(
            self.vehicle,
            drag_force,
        )
        if not isnan(global_drag_force.x):
            self.physics_node.apply_central_impulse(
                global_drag_force * globalClock.dt,
            )

    def apply_repulsors(self):
        dt = globalClock.dt
        repulsor_data = zip(
            self.vehicle_data.repulsor_nodes,
            self.sensors[REPULSOR_DATA],
            self.commands[REPULSOR_ACTIVATION],
            self.commands[REPULSOR_TARGET_ORIENTATIONS]
        )
        for node, data, activation, angle in repulsor_data:
            active = data.active
            frac = data.fraction
            # Repulse in current orientation
            if active and activation:
                if activation > 1.0:
                    activation = 1.0
                if activation < 0.0:
                    activation = 0.0
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
                # Sound
                #sound = node.get_python_tag(REPULSOR_SOUND)
                #sound.set_volume(activation)
                #sound.set_play_rate(1 + activation/100)
            else:
                node.get_python_tag('ray_end').hide()
                #sound = node.get_python_tag(REPULSOR_SOUND)
                #sound.set_volume(0.0)
                #sound.set_play_rate(0.0)
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
        max_impulse = self.vehicle_data.max_gyro_torque
        if impulse.length() > max_impulse:
            clamped_impulse = impulse / impulse.length() * max_impulse
        else:
            clamped_impulse = impulse

        self.physics_node.apply_torque_impulse(clamped_impulse)

    def apply_thrusters(self):
        dt = globalClock.dt
        thruster_data = zip(
            self.vehicle_data.thruster_nodes,
            self.commands[THRUSTER_ACTIVATION],
        )
        for node, thrust in thruster_data:
            if self.thruster_heat >= 1.0:
                thrust = 0.0
            current_power = node.get_python_tag(THRUSTER_POWER)
            # Adjust thrust to be within bounds of thruster's ability to adjust
            # thrust.
            ramp_time = node.get_python_tag(THRUSTER_RAMP_TIME)
            ramp_step = (1 / ramp_time) * globalClock.dt
            thrust = adjust_within_limit(thrust, current_power, ramp_step)
            node.set_python_tag(THRUSTER_POWER, thrust)
            max_force = node.get_python_tag(FORCE)
            real_force = max_force * thrust
            # FIXME: See repulsors above for the shortcoming that this suffers
            thruster_pos = base.render.get_relative_vector(
                self.vehicle,
                node.get_pos(self.vehicle),
            )
            thrust_direction = self.app.render.get_relative_vector(
                node,
                Vec3(0, 0, 1)
            )
            self.physics_node.apply_impulse(
                thrust_direction * real_force * dt,
                thruster_pos,
            )
            heating = node.get_python_tag(THRUSTER_HEATING)
            cooling = node.get_python_tag(THRUSTER_COOLING)
            effective_heating = -cooling + (heating - cooling) * thrust
            self.thruster_heat += effective_heating * dt
            if self.thruster_heat < 0.0:
                self.thruster_heat = 0.0

            # Sound
            sound = node.get_python_tag(THRUSTER_SOUND)
            sound.set_volume(thrust)
            sound.set_play_rate(1 + thrust/20)

    def apply_airbrake(self):
        # Animation and state update only, since the effect is in air drag
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
        #self.model.pose(AIRBRAKE, self.airbrake_state, partName=AIRBRAKE)
        self.model.pose(AIRBRAKE, self.airbrake_state)

    def apply_stabilizer_fins(self):
        # Animation and state update only, since the effect is in air drag
        dt = globalClock.dt
        target = self.commands[STABILIZER_FINS]
        max_movement = self.stabilizer_fins_speed * dt
        # Clamp change to available speed
        delta = target - self.stabilizer_fins_state
        if abs(delta) > max_movement:
            self.stabilizer_fins_state += copysign(max_movement, delta)
        else:
            self.stabilizer_fins_state = target
        if self.stabilizer_fins_state > 1.0:
            self.stabilizer_fins_state = 1.0
        if self.stabilizer_fins_state < 0.0:
            self.stabilizer_fins_state = 0.0
        self.model.pose(
            STABILIZER_FINS,
            self.stabilizer_fins_state,
            #partName=STABILIZER_FINS,
        )
        # FIXME: Implement stabilizing effect

    def shock(self, x=0, y=0, z=0):
        self.physics_node.apply_impulse(
            Vec3(0,0,0),
            Vec3(random(), random(), random()) * 10,
        )
        self.physics_node.apply_torque_impulse(Vec3(x, y, z))
