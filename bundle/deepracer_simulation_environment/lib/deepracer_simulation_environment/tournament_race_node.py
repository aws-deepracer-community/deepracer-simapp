#!/usr/bin/env python3

import rospy

import logging
import subprocess
import sys

from markov.log_handler.logger import Logger
from markov.log_handler.exception_handler import log_and_exit
from markov.log_handler.constants import (SIMAPP_EVENT_ERROR_CODE_500,
                                          SIMAPP_SIMULATION_WORKER_EXCEPTION)

logger = Logger(__name__, logging.INFO).get_logger()


def main():
    try:
        # parse argument
        race_idx = sys.argv[1]
        race_yaml_path = sys.argv[2]
        racecars_with_stereo_cameras = sys.argv[3]
        racecars_with_lidars = sys.argv[4]
        race_car_colors = sys.argv[5]
        simapp_versions = sys.argv[6]

        launch_name = 'tournament_rl_agent.launch'

        cmd = ' '.join(["roslaunch",
                        "deepracer_simulation_environment",
                        "{}".format(launch_name),
                        "local_yaml_path:={}".format(race_yaml_path),
                        "racecars_with_stereo_cameras:={}".format(racecars_with_stereo_cameras),
                        "racecars_with_lidars:={}".format(racecars_with_lidars),
                        "multicar:={}".format(True),
                        "car_colors:={}".format(race_car_colors),
                        "simapp_versions:={}".format(simapp_versions)
                        ])
        logger.info("cmd: {}".format(cmd))
        subprocess.Popen(cmd, shell=True, executable="/bin/bash")
    except Exception as e:
        log_and_exit("Tournament race node failed: race_idx: {}, {}"
                         .format(race_idx, e),
                     SIMAPP_SIMULATION_WORKER_EXCEPTION,
                     SIMAPP_EVENT_ERROR_CODE_500)


if __name__ == '__main__':
    rospy.init_node('tournament_race_node', anonymous=True)
    main()
    rospy.spin()
