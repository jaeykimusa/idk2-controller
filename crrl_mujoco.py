# mujoco_sim.py
# author: Jaey Kim
# date: 01/19/2026


from robot_description.go2_description import go2_mujoco

import mujoco as mj
import mujoco.viewer as mjv
import logging 
from dataclasses import dataclass, field


@dataclass
class CRRLMujocoConfig:
    """Configuration for the Go2 Simulation. """
    # simulation settings
    dt: float = 0.002
    showSimMsg: bool = True
    show_contact_pts: bool = False
    show_joint_axes: bool = False
    transparent_robot: bool = False    # xray_mode: show axes + transparent
    # show_contact_forces: bool = False   //TODO: NOT IN USE
    # show_com: bool = False              //TODO: NOT IN USE


@dataclass
class CRRLMujocoPropty:
    """Dynamic simulation properties. """
    # kd: float = 0.1             //TODO: NOT IN USE
    # kp: float = 0.1             //TODO: NOT IN USE
    # use_gravity: bool = True    //TODO: NOT IN USE


@dataclass
class CRRLMujocoCamConfig:
    """Camera configuration for the simulation viewer. """
    # camera settings
    cam_azimuth: float = 135.0    # Horizontal angle (deg)
    cam_elevation: float = -20.0  # Vertical angle (deg)
    cam_distance: float = 3.0     # Distance from robot (meters)
    cam_target: list = field(default_factory=lambda: [0, 0, 0.3]) # Focus point
    # enable_recorder: bool = False //TODO: NOT IN USE


class CRRLMujoco:


    def __init__(
            this,
            simName: str = "defaultSim",
            simConfig: CRRLMujocoConfig | None = None, 
            simPropty: CRRLMujocoPropty| None = None, 
            simCamConfig: CRRLMujocoCamConfig | None = None):
        
        this.simName = simName

        this.model = go2_mujoco.model
        this.data = go2_mujoco.data

        this.simConfig = simConfig or CRRLMujocoConfig()
        this.simPropty = simPropty or CRRLMujocoPropty()
        this.simCamConfig = simCamConfig or CRRLMujocoCamConfig()

        # values from config obj
        this.dt = this.simConfig.dt; this.model.opt.timestep = this.simConfig.dt    # defaut to 0.002s

        this.viewer = None
        this.showSimMsg = this.simConfig.showSimMsg

        # basic logging
        # Configure how the message looks
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
            datefmt='%H:%M:%S'
        )
        logger = logging.getLogger("Go2Controller")
        logger.info("MuJoCo model loaded successfully.")


    def reset(this):
        mj.mj_resetData(this.model, this.data)


    def get_state(this):
        return {
            "q": this.data.qpos.copy(),
            "qd": this.data.qvel.copy(),
            "qdd": this.data.qacc.copy(),
        }


    def apply_control(this, tau):
        this.data.ctrl[:] = tau


    def step(this):
        mj.mj_step(this.model, this.data)


    def render(this):
        if this.viewer is not None:
            this.viewer.sync()


    def launch_viewer(this, contactPts=False, jointAxes=False, robotTransprt=False):
        this.viewer = mjv.launch_passive(
            this.model, this.data
        )

        # camera configuration
        this.viewer.cam.azimuth = this.simCamConfig.cam_azimuth
        this.viewer.cam.elevation = this.simCamConfig.cam_elevation
        this.viewer.cam.distance = this.simCamConfig.cam_distance
        this.viewer.cam.lookat[:] = this.simCamConfig.cam_target

        this.viewer.opt.flags[mj.mjtVisFlag.mjVIS_JOINT] = this.simConfig.show_contact_pts
        this.viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = this.simConfig.show_joint_axes
        this.viewer.opt.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = this.simConfig.transparent_robot


    def run(this, tau):
        this.launch_viewer(False, False, False)

        with this.viewer:
            while this.viewer.is_running():
                state = this.get_state()
                # tau = controller.compute_control(state)
                this.apply_control(tau)
                this.step()
                this.render()


    def close(this):
        this.viewer.close()


