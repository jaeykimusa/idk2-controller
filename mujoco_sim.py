# mujoco_sim.py
# author: Jaey Kim
# date: 01/19/2026


from robot_description.go2_description import go2_mujoco

import mujoco as mj
import mujoco.viewer


class MuJoCoSim:


    def __init__(this, dt=0.001):
        this.model = go2_mujoco.model
        this.data = go2_mujoco.data
        this.dt = dt
        this.model.opt.timestep = dt
        this.viewer = None


    def reset(this):
        mj.mj_resetData(this.model, this.data)


    def get_state(this):
        return {
            "qpos": this.data.qpos.copy(),
            "qvel": this.data.qvel.copy(),
        }


    def apply_control(this, tau):
        this.data.ctrl[:] = tau


    def step(this):
        mj.mj_step(this.model, this.data)


    def render(this):
        if this.viewer is not None:
            this.viewer.sync()


    def launch_viewer(this, contactPts=False, jointAxes=False, robotTransprt=False):
        this.viewer = mujoco.viewer.launch_passive(
            this.model, this.data
        )

        # Camera configuration
        this.viewer.cam.azimuth = 135
        this.viewer.cam.elevation = -20
        this.viewer.cam.distance = 3
        this.viewer.cam.lookat[:] = [0, 0, 0.3]

        # Show contact points
        this.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = contactPts
        
        # Show joint axes
        this.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = jointAxes

        # Make the robot slightly transparent to see internal joints
        this.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = robotTransprt


    def run(this, tau):
        this.launch_viewer(True, True, True)

        with this.viewer:
            while this.viewer.is_running():
                state = this.get_state()
                # tau = controller.compute_control(state)
                this.apply_control(tau)
                this.step()
                this.render()

    def close(this):
        this.viewer.close()


def main():
    # mj_sim = MuJoCoSim()
    # print(mj_sim.mj_model.nq)
    pass


if __name__ == "__main__":
    main()