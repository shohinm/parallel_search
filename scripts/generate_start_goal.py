import os
import mujoco as mp
from mujoco import MjData, MjModel
# import mujoco_viewer
from time import sleep
import numpy as np
import pdb
from numpy import genfromtxt
from abb_fkik_solver import ABBFkIkSolver



#### Load mujoco model
model_dir = '../third_party/mujoco-2.3.2/model/abb/irb_1600'
urdf = os.path.join(model_dir, 'irb1600_6_12_generated.urdf')
mjcf_arm = 'irb1600_6_12_shield.xml'
arm_model = MjModel.from_xml_path(os.path.join(model_dir, mjcf_arm))
arm_data = MjData(arm_model)


urdf = os.path.join(model_dir, 'irb1600_6_12_generated.urdf')
fkik = ABBFkIkSolver(urdf)
print(fkik.computeFK([0,0,0,0,0,0]))
print(fkik.computeIK([0.815,0,0.9615], [0,0,0]))


starts = np.empty((0,arm_model.nq))
goals = np.empty((0,arm_model.nq))

### Volume to do IK
# vol = np.array([[[1.2, 1.8], [-1.2, 0], [0.2, 1.2]],
#                 [[0, 1.2], [0, 0.6], [0.2, 1.2]],
#                 [[-0.6, 0], [-1.2, 0], [0.2, 1.2]],
#                 [[0, 1.2], [-1.8, -1.2], [0.2, 1.2]]])
# vol = np.array([[[1.4, 2.0], [-1.2, 0], [1.0, 1.8]],
#                 [[0, 1.2], [0.2, 1.0], [1.0, 1.8]],
#                 [[-1.0, -0.4], [-1.2, 0], [1.0, 1.8]],
#                 [[0, 1.2], [-2.0, -1.4], [1.0, 1.8]]])
vol = np.array([[[1.4, 2.0], [-1.2, 0], [1.0, 1.4]],
                [[1.4, 2.0], [-1.2, 0], [1.4, 1.8]],
                [[0, 1.2], [0.2, 1.0], [1.0, 1.4]],
                [[0, 1.2], [0.2, 1.0], [1.4, 1.8]],
                [[-1.0, -0.4], [-1.2, 0], [1.0, 1.4]],
                [[-1.0, -0.4], [-1.2, 0], [1.4, 1.8]],
                [[0, 1.2], [-2.0, -1.4], [1.0, 1.4]],
                [[0, 1.2], [-2.0, -1.4], [1.4, 1.8]]])

data_size = 1000

fkik = ABBFkIkSolver(urdf, start_pos=[0.65, -0.7, 0.45])

def isCollisionFree(q):
    if not (np.all(arm_model.jnt_range[:,0] <= q) and np.all(arm_model.jnt_range[:,1] >= q)):
        return False

    arm_data.qpos[:] = q[:]
    mp.mj_forward(arm_model, arm_data)

    if arm_data.ncon > 0:
        return False
    else:
        return True


def ik(x,r):
    ### your code here
    q = fkik.computeIK(target_position=x, target_orientation=r)
    return q

num_regions = 8

s = 0
while s < data_size:
    sid = 0
    gid = 0
    while 1:
        id = np.random.choice(8, 2)
        if id[0] != id[1]:
            sid = id[0]
            gid = id[1]
            break

    st_seed = np.random.random_sample((3,))
    go_seed = np.random.random_sample((3,))
    start = np.array([(vol[sid,:,1]-vol[sid,:,0])*st_seed+vol[sid,:,0]])
    goal = np.array([(vol[gid,:,1]-vol[gid,:,0])*go_seed+vol[gid,:,0]])

    r = np.array([0,0,0,1])
    if sid==2:
        r = np.array([0.707, 0, 0, 0.707])
    elif sid==3:
        r = np.array([0.707, 0, 0, 0.707])
    elif sid == 4:
        r = np.array([0, 0, 1, 0])
    elif sid == 5:
        r = np.array([0, 0, 1, 0])
    elif sid==6:
        r = np.array([0.707, 0, 0, -0.707])
    elif sid==7:
        r = np.array([0.707, 0, 0, -0.707])
    st_q = ik(start, r)

    r = np.array([0,0,0,1])
    if gid==2:
        r = np.array([0.707, 0, 0, 0.707])
    if gid == 3:
        r = np.array([0.707, 0, 0, 0.707])
    elif gid==4:
        r = np.array([0, 0, 1, 0])
    elif gid == 5:
        r = np.array([0, 0, 1, 0])
    elif gid==6:
        r = np.array([0.707, 0, 0, -0.707])
    elif gid==7:
        r = np.array([0.707, 0, 0, -0.707])
    go_q = ik(goal, r)


    if st_q is not None and go_q is not None:
        # try:
        if isCollisionFree(st_q) and isCollisionFree(go_q):
            try:
                starts = np.append(starts, st_q[np.newaxis,:], axis=0)
                goals = np.append(goals, go_q[np.newaxis,:], axis=0)
                s = s+1
            except Exception as e: 
                print(e)
                pdb.set_trace()
        print("i: {} | total: {}".format(s, data_size))
    # else:
        # print("start: {}".format(start))
        # print("goal: {}".format(goal))

        # print("ik failed")
        # except Exception as e: 
        #     print(e)
        #     pdb.set_trace()


start_file = '../examples/manipulation/resources/shield/starts.txt'
goal_file = '../examples/manipulation/resources/shield/goals.txt'
np.savetxt(start_file, starts, delimiter=' ')
np.savetxt(goal_file, goals, delimiter=' ')
