import os
import mujoco as mp
from mujoco import MjData, MjModel
import mujoco_viewer
from time import sleep
import numpy as np
from numpy import genfromtxt

model_dir = '../third_party/mujoco-2.3.2/model/abb/irb_1600'
mjcf_arm = 'irb1600_6_12_shield.xml'
arm_model = MjModel.from_xml_path(os.path.join(model_dir, mjcf_arm))
arm_data = MjData(arm_model)

starts_file = '../examples/manipulation/resources/shield/starts.txt'
goals_file = '../examples/manipulation/resources/shield/goals.txt'
starts = genfromtxt(starts_file, delimiter=' ')
goals = genfromtxt(goals_file, delimiter=' ')

valid_starts_file = '../examples/manipulation/resources/shield/valid_starts.txt'
valid_goals_file = '../examples/manipulation/resources/shield/valid_goals.txt'
valid_starts = np.empty((0,arm_model.nq))
valid_goals = np.empty((0,arm_model.nq))

for i in range(np.shape(starts)[0]):
    arm_data.qpos[:] = starts[i,:]
    mp.mj_step(arm_model, arm_data)
    start_coll_free = False
    if arm_data.ncon == 0:
        start_coll_free = True

    arm_data.qpos[:] = goals[i,:]
    mp.mj_step(arm_model, arm_data)
    goal_coll_free = False
    if arm_data.ncon == 0:
        goal_coll_free = True

    if start_coll_free and goal_coll_free:
        valid_starts = np.append(valid_starts, starts[i,:][np.newaxis,:], axis=0)
        valid_goals = np.append(valid_goals, goals[i,:][np.newaxis,:], axis=0)

np.savetxt(valid_starts_file, valid_starts, delimiter=' ')
np.savetxt(valid_goals_file, valid_goals, delimiter=' ')


