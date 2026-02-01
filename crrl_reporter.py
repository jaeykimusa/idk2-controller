# sim_reporter.py
# this generates the messages and reports the robot state during simulation.
# author: Jaey Kim
# date: 01/19/2026

import logging #as log
import colorlog


SIMULATION_LEVEL = 21
CONTROLLER_LEVEL = 22


class SimReporter:

    def __init__(this, simName, ):
        this.logging.addLevelName(SIMULATION_LEVEL, "Sim")


# Define custom levels (choosing numbers that don't clash)
SETTING_LEVEL = 21     # Just above INFO
CONTROLLER_LEVEL = 22  # Just above SETTING

logging.addLevelName(SETTING_LEVEL, "SETTING")
logging.addLevelName(CONTROLLER_LEVEL, "CONTROLLER")

# Add helper methods to the Logger class so you can call logger.setting()
def setting(self, message, *args, **kws):
    self.log(SETTING_LEVEL, message, *args, **kws)

def controller(self, message, *args, **kws):
    self.log(CONTROLLER_LEVEL, message, *args, **kws)

logging.Logger.setting = setting
logging.Logger.controller = controller

log_format = "%(asctime)s | %(levelname)-10s | %(name)s:%(funcName)s:%(lineno)d - %(message)s"

# Setup
logger = logging.getLogger("Go2Sim")
logging.basicConfig(level=logging.DEBUG, format=log_format)

# Usage
# logger.setting("Timestep set to 0.001s")
# logger.controller("PD gains updated: Kp=40, Kd=1")




handler = colorlog.StreamHandler()
handler.setFormatter(colorlog.ColoredFormatter(
    "%(log_color)s%(levelname)-8s%(reset)s %(blue)s%(message)s",
    log_colors={
        'DEBUG':    'cyan',
        'INFO':     'green',
        'WARNING':  'yellow',
        'ERROR':    'red',
        'CRITICAL': 'red,bg_white',
    }
))

logger = colorlog.getLogger("Go2Sim")
logger.addHandler(handler)
logger.setLevel(logging.DEBUG)

logger.info("Simulation started.")
logger.error("Inverse Kinematics failed!")

