# sim.py
# this is the main simulation loop.
# author: Jaey Kim
# date: 01/19/2026


from mujoco_sim import IdkSim, IdkSimConfig

import numpy as np
import mujoco as mj


def main():
    simConfig = IdkSimConfig(
        show_contact_pts=True, 
        show_joint_axes=True, 
        transparent_robot=True
        )
    
    mj_sim = IdkSim(simConfig=simConfig)
    
    # mj_sim.launch_viewer()
    zero_torques = np.zeros(12)
    mj_sim.run(zero_torques)


if __name__ == "__main__":
    main()
