import os
import mujoco as mp
from mujoco import MjData, MjModel
import mujoco_viewer
from time import sleep
import numpy as np
from numpy import genfromtxt

dt = 6e-3

model_dir = '/home/gaussian/cmu_ri_phd/phd_research/parallel_search/third_party/mujoco-2.3.2/model/abb/irb_1600'
mjcf = 'irb1600_6_12.xml'
mjcf_arm = 'irb1600_6_12_shield.xml'

# just using arm model for calculating ee traj
arm_model = MjModel.from_xml_path(os.path.join(model_dir, mjcf_arm))
arm_data = MjData(arm_model)
viewer = mujoco_viewer.MujocoViewer(arm_model, arm_data)


# simulate and render
for _ in range(10000):
    if viewer.is_alive:
        mp.mj_step(arm_model, arm_data)
        print(arm_data.xpos[-1])
        viewer.render()
    else:
        break

# close
viewer.close()
