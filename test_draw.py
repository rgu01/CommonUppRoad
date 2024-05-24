import matplotlib.pyplot as plt
import warnings
import os
warnings.filterwarnings('ignore')

import numpy as np

from commonroad.geometry.shape import Rectangle
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer, MPDrawParams, DynamicObstacleParams
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import CustomState, InitialState
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction

root = os.path.dirname(__file__)
SCENARIO_PATH = root + '/scenarios/DEU_Ffb-1_3_T-1.xml'
SAMPLING_LOG_PATH = root + '/uppaal/sampling.log'
N_OBS_STATES = 5
N_NEW_OBS = 0


scenario, planning_problem_set = CommonRoadFileReader(SCENARIO_PATH).open()
rnd = MPRenderer(figsize=(8,4.5))
rnd.draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "yellow"
draw_params = DynamicObstacleParams()
draw_params.vehicle_shape.occupancy.shape.facecolor = "green"
scenario.draw(rnd)
scenario.dynamic_obstacles[0].draw(rnd, draw_params)
planning_problem_set.draw(rnd)
rnd.render()

draw_params = MPDrawParams()
draw_params.time_end = 25
draw_params.dynamic_obstacle.show_label = False
draw_params.dynamic_obstacle.draw_icon = True
draw_params.dynamic_obstacle.draw_shape = True
ego_params3.vehicle_shape.occupancy.shape.facecolor = "r"
ego_params3.vehicle_shape.occupancy.shape.edgecolor = "r"
rnd.create_video([scenario], str(scenario.scenario_id) + ".gif", draw_params=draw_params)

