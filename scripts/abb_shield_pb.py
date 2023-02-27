
import os
import mujoco as mp
from mujoco import MjData, MjModel
import mujoco_viewer
from time import sleep
import numpy as np
from numpy import genfromtxt

# planner_name = 'insat'
planner_name = 'pinsat'
# planner_name = 'rrt'
# planner_name = 'epase'

static_planner = True if planner_name=='rrt' or planner_name=='epase' else False

if static_planner:
  # dt = 1e-2
  # dt = 0.05
  dt = 6e-3
else:
  dt = 6e-3
  # dt = 5e-2
  # dt = 0.05
  # dt = 1

model_dir = '/home/gaussian/cmu_ri_phd/phd_research/parallel_search/third_party/mujoco-2.3.2/model/abb/irb_1600'
mjcf = 'irb1600_6_12.xml'
mjcf_arm = 'irb1600_6_12_shield.xml'
traj_file = '/home/gaussian/cmu_ri_phd/phd_research/parallel_search/logs/' + planner_name + '_abb_traj.txt'
starts_file = '/home/gaussian/cmu_ri_phd/phd_research/parallel_search/logs/' + planner_name + '_abb_starts.txt'
goals_file = '/home/gaussian/cmu_ri_phd/phd_research/parallel_search/logs/' + planner_name + '_abb_goals.txt'
traj = genfromtxt(traj_file, delimiter=' ' if static_planner else ',')
starts = genfromtxt(starts_file, delimiter=',')
goals = genfromtxt(goals_file, delimiter=',')

# just using arm model for calculating ee traj
arm_model = MjModel.from_xml_path(os.path.join(model_dir, mjcf_arm))
arm_data = MjData(arm_model)
viewer = mujoco_viewer.MujocoViewer(arm_model, arm_data)

# num_traj = np.shape(starts)[0]
# ee_traj = np.zeros((np.shape(starts)[0],3))
# for i in range(num_traj):
#   arm_data.qpos[:] = goals[i,:]
#
#
#   mp.mj_step(arm_model, arm_data)
#   ee_traj[i,:] = arm_data.xpos[-1]

def balltraj(end_pos, len, max_len, dx):
  xy_end = np.array([end_pos[0], end_pos[1], 0])
  u = end_pos/np.linalg.norm(xy_end)
  ball_traj = np.tile(end_pos, [max_len, 1])
  for i in range(len):
    ball_traj[i] = end_pos + (len-i)*u*dx
  quat_traj = np.tile([1,0,0,0], [max_len, 1])
  ball_traj = np.hstack((ball_traj, quat_traj))
  return ball_traj


def upsampleTraj(traj, dx=0.1):
  col = np.shape(traj)[1]
  uptraj = np.empty((0, col))
  for i in range(np.shape(traj)[0]-1):
    if np.array_equal(traj[i,:], -1*np.ones((arm_model.nq,))):
      continue
    if np.array_equal(traj[i+1,:], -1*np.ones((arm_model.nq,))):
      continue

    x1 = traj[i,:]
    x2 = traj[i+1,:]
    dist = np.linalg.norm(x2-x1)
    N = int(dist/dx)
    samp = np.linspace(x1, x2, N)
    uptraj = np.append(uptraj, samp, axis=0)

  return uptraj

if static_planner:
  traj = upsampleTraj(traj, 0.1)

for i in range(np.shape(traj)[0]):
  if np.array_equal(traj[i,:], -1*np.ones((arm_model.nq,))):
    continue
  if viewer.is_alive:
    arm_data.qpos[:] = traj[i,:]
    print(traj[i,:])
    mp.mj_step(arm_model, arm_data)
    viewer.render()
    sleep(dt)
  else:
      break

# close
viewer.close()

