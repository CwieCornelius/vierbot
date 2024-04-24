import sys
import time
import numpy as np
import pybullet as p
import pybullet_data
from pychamp.champ.types import Pose, Twist
from pychamp.champ.base import Base
from pychamp.champ.controllers.cheetah_one import CheetahOne
from pychamp.champ.kinematics import Kinematics

from pychamp.champ.robots.profile import Vierbot as vierbot
from pychamp.champ.robots.profile import AlienGo as aliengo
from pychamp.champ.robots.profile import AnymalB as anymal_b
from pychamp.champ.robots.profile import AnymalC as anymal_c
from pychamp.champ.robots.profile import DKitty as dkitty
from pychamp.champ.robots.profile import LittleDog as littledog
from pychamp.champ.robots.profile import MiniCheetah as mini_cheetah
from pychamp.champ.robots.profile import OpenQuadruped as open_quadruped
from pychamp.champ.robots.profile import OpenDog as opendog
from pychamp.champ.robots.profile import Spot as spot
from pychamp.champ.robots.profile import SpotMicro as spotmicro
from pychamp.champ.robots.profile import StochLite as stochlite
from pychamp.champ.pybullet.fake_hardware import Sensors, Actuators
import pychamp.champ.pybullet.utils as pb_utils
import pychamp.champ.geometry as geom

from tinymovr_handler import Actor, Actors
class Champ:
    def __init__(self):
        robot_profile = mini_cheetah
        robot_profile = anymal_c
        robot_profile = vierbot
        #robot_profile = spot

        physics_client = p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        base_id = p.loadURDF(
            robot_profile.urdf,
            [0, 0, robot_profile.nominal_height * 2.5],
            p.getQuaternionFromEuler([0, 0, 0])
        )


        quadruped = Base(robot_profile)
        controller = CheetahOne(quadruped, robot_profile)
        ik = Kinematics(quadruped, robot_profile.knee_orientation)
        sensors = Sensors(plane_id, base_id, robot_profile.joint_names)
        actuators = Actuators(base_id, robot_profile.joint_names)
        print(f" LEG JOINTSSSS {quadruped.legs.joint_states}")
        req_pose = Pose()
        req_pose.position.z = robot_profile.nominal_height
        req_vel = Twist()

        pb_utils.print_teleop_instructions()

        actors = Actors()
        print(actors.get_voltages())
        actors.stand_up()
        time.sleep(5)

        while True:
            req_vel = pb_utils.get_req_vel(base_id, req_vel)

            foot_positions = controller.walk(req_pose, req_vel)
            target_joint_positions = ik.inverse(foot_positions)

            base_orientation = sensors.get_base_quat()
            base_rot_matrix = geom.quat_to_matrix(
                x=base_orientation[0],
                y=base_orientation[1],
                z=base_orientation[2],
                w=base_orientation[3]
            )

            base_orientation = sensors.get_base_rpy()
            base_rot_matrix = geom.rpy_to_matrix(
                roll=base_orientation[0],
                pitch=base_orientation[1],
                yaw=base_orientation[2]
            )

            foot_positions_from_base = quadruped.feet.position
            foot_positions_from_hip = quadruped.transform_to_hip(foot_positions_from_base)

            joints_pos, joints_vel = sensors.get_joint_states()
            quadruped.legs.joint_states = (joints_pos, joints_vel)

            world_x, world_y, world_z = sensors.get_base_position()
            quadruped.pose.position = (world_x, world_y, world_z)

            quadruped.pose.orientation = sensors.get_base_quat()
            quadruped.pose.rpy = sensors.get_base_rpy()
            quadruped.legs.contact_states = sensors.get_contact_states()

            linear, angular = sensors.get_base_velocity()
            quadruped.velocity.linear = linear
            quadruped.velocity.angular = angular

            j_pos, j_vel = quadruped.legs.joint_states
            actuators.position_control(target_joint_positions)
            print(target_joint_positions)
            target_joint_positions[0] = 0
            target_joint_positions[3] = 0
            target_joint_positions[6] = 0
            target_joint_positions[9] = 0
            actors.set_positions(target_joint_positions)

        p.disconnect()


if __name__ == '__main__':
    c = Champ()
