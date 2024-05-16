import matplotlib.pyplot as plt
import warnings
import os
warnings.filterwarnings('ignore')

import numpy as np

from commonroad.geometry.shape import Rectangle
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.mp_renderer import MPRenderer, MPDrawParams
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import CustomState, InitialState
from commonroad.scenario.trajectory import Trajectory
from commonroad.prediction.prediction import TrajectoryPrediction


def read_sample_log(path):
    with open(path, 'r') as f:
        data = f.read().splitlines()

    data = [list(map(float, line.split(' '))) for line in data]
    return data


def make_dynamic_obstacle(obstacle_id, data, w=1.8, l=4.3):
    t, x, y, orientation, velocity, acceleration = data[0]
    inital_state = CustomState(
        position = np.array([x, y]),
        velocity = velocity,
        orientation = orientation,
        time_step = 0
    ).convert_state_to_state(InitialState())

    state_list = []
    for row in data[1:]:
        t, x, y, orientation, velocity, acceleration = row
        state = CustomState(
            position = np.array([x, y]),
            velocity = velocity,
            orientation = orientation,
            acceleration = acceleration,
            time_step = t
        )
        state_list.append(state)

    trajectory = Trajectory(1, state_list)
    shape = Rectangle(width=w, length=l)
    prediction = TrajectoryPrediction(trajectory, shape)

    obstacle = DynamicObstacle(
        obstacle_id, ObstacleType.CAR, shape, inital_state, prediction
    )
    return obstacle


# deifne variables for parsing
root = os.path.dirname(__file__)
SCENARIO_PATH = root + '/scenarios/ZAM_Tutorial-1_2_T-1.xml'
SAMPLING_LOG_PATH = root + '/uppaal/sampling.log'
N_OBS_STATES = 5
N_NEW_OBS = 0


scenario, planning_problem_set = CommonRoadFileReader(SCENARIO_PATH).open()

# read sample log
sample = read_sample_log(SAMPLING_LOG_PATH)

# divide data into obstacle states and ego states
ego_states = []
new_obsts = [[]] * N_NEW_OBS

for row in sample:
    t = int(row[0])
    obst = 0
    obst_state = []
    for idx in range(1, N_NEW_OBS * N_OBS_STATES, N_OBS_STATES):
        new_obsts[obst].append([t] + row[idx:idx + N_OBS_STATES])
        obst += 1

    ego_states.append([t] + row[1 + N_NEW_OBS * N_OBS_STATES:])

# create the obstacles
for obst_states in new_obsts:
    obst = make_dynamic_obstacle(scenario.generate_object_id(), obst_states)
    scenario.add_objects(obst)

ego = make_dynamic_obstacle(scenario.generate_object_id(), ego_states)
scenario.add_objects(ego)

# render and store as gif
rnd = MPRenderer()
dp = MPDrawParams()
dp.time_end = len(sample)
rnd.create_video([scenario], 'uppaal_generated.gif', draw_params=dp)
