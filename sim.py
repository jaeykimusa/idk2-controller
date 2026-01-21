# sim.py
# this is the main simulation loop.
# author: Jaey Kim
# date: 01/19/2026


from mujoco_sim import MuJoCoSim

import numpy as np
import mujoco as mj


zero_torques = np.zeros(12)
# zero_torques = mj.mju_zero(0,12)
mj_sim = MuJoCoSim()

# mj_sim.launch_viewer()

mj_sim.run(zero_torques)

