import pybullet as p
import pdb
import numpy as np
import brain2.bullet.problems as problems
from brain2.bullet.interface import BulletInterface
from brain2.bullet.ik import BulletIKSolver
from brain2.robot.domain import RobotDomainDefinition
import brain2.utils.axis as axis
from brain2.utils.pose import make_pose
from brain2.utils.info import logwarn


class ABBFkIkSolver():

    def __init__(self, assets_path, visualize=False, start_pos=[0,0,0], start_orientation=[0,0,0,1]):
        self.ik = BulletIKSolver()
        self.iface = BulletInterface(gui=visualize, add_ground_plane=False)
        self.robot = self.load_ABB(self.iface, assets_path, start_pos=start_pos, start_orientation=start_orientation)

    def load_ABB(self, iface, model_path, padding=0., start_pos=[0,0,0], start_orientation=[0,0,0,1]):
        """Simple helper function for loading the robot."""


        # Load URDF
        logwarn("Loading robot with path = " + str(model_path))

        robot = iface.load_urdf('robot',
                                model_path,
                                fixed_base=True,
                                padding=padding,
                                base_position=start_pos,
                                base_orientation=start_orientation)

        robot = iface.get_object('robot')

        ARM = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        robot.set_active_joints(ARM)
        robot.set_ee("joint_6")

        robot.verbose = 1

        # Return reference
        return iface.get_object('robot')
        
    def computeIK(self, target_position, target_orientation, q0=None, retrials=0):
        goal = make_pose(target_position, target_orientation)

        if q0 is None:
            res = self.ik(self.robot, goal, q0=self.robot.sample_uniform())
        else:
            res = self.ik(self.robot, goal, q0=q0)

        if res is None:
            # print("Original config failed. trying something else...")
            for _ in range(retrials):
                res = self.ik(self.robot, goal, q0=self.robot.sample_uniform())
                if res is not None:
                    break
        # if res is not None:
        #     print(">> ik found config =", res)
        # else:
        #     print("We failed to find a goal pose!")

        return res

    def computeFK(self, joints):
        self.robot.set_joint_positions(joints)
        p, o = self.robot.get_ee_pose(matrix=False)
        return np.array(p), np.array(o) 


