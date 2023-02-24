
import os
import mujoco as mp
from mujoco import MjData, MjModel
import mujoco_viewer
from time import sleep
import numpy as np
from numpy import genfromtxt

model_dir = '/home/gaussian/cmu_ri_phd/phd_research/parallel_search/third_party/mujoco-2.3.2/model/abb/irb_1600'
mjcf_arm = 'irb1600_6_12_shield.xml'
starts_file = '../examples/manipulation/resources/shield/starts.txt'
goals_file = '../examples/manipulation/resources/shield/goals.txt'
starts = genfromtxt(starts_file, delimiter=' ')
goals = genfromtxt(goals_file, delimiter=' ')

num_samp = np.shape(starts)[0]

# just using arm model for calculating ee traj
arm_model = MjModel.from_xml_path(os.path.join(model_dir, mjcf_arm))
arm_data = MjData(arm_model)
viewer = mujoco_viewer.MujocoViewer(arm_model, arm_data)

dt = 0.5
for i in range(num_samp):
  if viewer.is_alive:
    arm_data.qpos[:] = starts[i,:]
    mp.mj_step(arm_model, arm_data)
    viewer.render()
    sleep(dt)
    arm_data.qpos[:] = goals[i,:]
    mp.mj_step(arm_model, arm_data)
    viewer.render()
    sleep(dt)
  else:
      break

# close
viewer.close()

