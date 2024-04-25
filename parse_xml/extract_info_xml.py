from commonroad.common.file_reader import CommonRoadFileReader
import sys
import logging
import os
import numpy as np


# %% load the files and parameters
input_file_folder = os.getcwd() + "\\data_xml"
output_file_folder = os.getcwd() + "\\output_xml"

xml_file = "ZAM_Ramp-1_1-T-1.xml" # "ZAM_Tutorial-1_2_T-1.xml" or "DEU_Ffb-1_3_T-1.xml" or "ZAM_Ramp-1_1-T-1.xml"
xml_file_path = input_file_folder + "\\" + xml_file
file_name, _ = os.path.splitext(xml_file)

output_file = output_file_folder + "\\" + file_name + "_generated_lanelet.txt"
if not os.path.exists(output_file_folder):
        os.makedirs(output_file_folder)

SCALE = 100 # the scaling factor from double to int

# Parse the XML file
scenario, planning_problem_set = CommonRoadFileReader(xml_file_path).open()

# MAXP is to set the maximal number of points of a lane
MAXP = max(len(lane._left_vertices) for lane in scenario.lanelet_network.lanelets)
# MAXL is the number of lanes in the lanelet network
MAXL = len(scenario.lanelet_network.lanelets)
# MAXSO is the number of static obstacles
MAXSO = len(scenario.static_obstacles)


# %% construct the lanelet network declaration in c code
ST_BOUND_leftLane_str_set = []
ST_BOUND_rightLane_str_set = []
ST_LANE_lane_str_set = []
ST_LANE_laneNet_str = []
for i, lane in enumerate(scenario.lanelet_network.lanelets):
    # get lane information
    lane_ID = lane.lanelet_id
    lane_predecessor = None if lane.predecessor == [] else lane.predecessor # TODO: check predecessor and successor, they can be multiple
    lane_successor = None if lane.successor == [] else lane.successor
    lane_adjLeft = lane.adj_left
    lane_adjRight = lane.adj_right

    # lane_marking type: 'dashed', 'solid', 'unknown'
    lane_markingLeft = True if lane.line_marking_left_vertices.value == 'dashed' else False
    lane_markingRight = True if lane.line_marking_right_vertices.value == 'dashed' else False
    
    # if same_direction is True, return True, if False or None, return False
    lane_dirLeft = True if (lane.adj_left_same_direction == True) else False
    lane_dirRight = True if (lane.adj_right_same_direction == True) else False

    # parse the left and right lanes and scale the position
    leftLane = (lane._left_vertices*SCALE).astype(int)
    rightLane = (lane._right_vertices*SCALE).astype(int)
    # merge the lane points if they are on the same straight line 
    if np.all(leftLane[:, 1] == leftLane[0, 1]) or np.all(leftLane[:, 0] == leftLane[0, 0]):
        leftLane = leftLane[[0, -1]]
    if np.all(rightLane[:, 1] == rightLane[0, 1]) or np.all(rightLane[:, 0] == rightLane[0, 0]):
        rightLane = rightLane[[0, -1]]

    # construct the string declaration
    leftLane_str = "{" + ", ".join(["{" + ", ".join(map(str, point)) + "}" for point in leftLane]) + "}" # e.g., '{{-86, 7}, {-64, 6}}'
    ST_BOUND_left = f"const ST_BOUND leftLane{i + 1} = {{{leftLane_str}, {lane_markingLeft}}};" # e.g., 'const ST_BOUND leftLane1 = {{{-86, 7}, {-64, 6}}, True};'
    rightLane_str = "{" + ", ".join(["{" + ", ".join(map(str, point)) + "}" for point in rightLane]) + "}"
    ST_BOUND_right = f"const ST_BOUND rightLane{i + 1} = {{{rightLane_str}, {lane_markingRight}}};"

    # construct the lane string declaration, e.g., 'const ST_LANE lane1 ={1, leftLane1, rightLane1, None, None, 2, False, None, False'
    ST_LANE_lane = f"const ST_LANE lane{i + 1} = " + "{" + ", ".join([f"{lane_ID}", f"leftLane{i + 1}", f"rightLane{i + 1}", 
                                                                     f"{lane_predecessor}", f"{lane_successor}", f"{lane_adjLeft}",
                                                                     f"{lane_dirLeft}", f"{lane_adjRight}", f"{lane_dirRight}"]) + "};"
    # collect the declearations
    ST_BOUND_leftLane_str_set.append(ST_BOUND_left)
    ST_BOUND_rightLane_str_set.append(ST_BOUND_right)
    ST_LANE_lane_str_set.append(ST_LANE_lane)
    ST_LANE_laneNet_str.append(f"lane{i + 1}")

ST_LANE_laneNet_str = "const ST_LANE laneNet[MAXL] = {" + ", ".join(ST_LANE_laneNet_str) + "};"


# %% construct the static obstacles declaration in c code
ST_RECTANGLE_obs_str_set = []
for static_obs in scenario.static_obstacles:
    obs_pos = (static_obs._initial_state.position*SCALE).astype(int)
    obs_ori = int(static_obs._initial_state.orientation*SCALE)
    obs_width = int(static_obs._obstacle_shape._width*SCALE)
    obs_length = int(static_obs._obstacle_shape._length*SCALE)
    ST_RECTANGLE_obs_pos = "{" + ", ".join(map(str, obs_pos)) + "}"
    ST_RECTANGLE_obs_str_single = "{" + ", ".join([ST_RECTANGLE_obs_pos, f"{obs_width}", f"{obs_length}", f"{obs_ori}"]) + "}" # e.g., {{2000, 700}, 200, 450, 0}
    ST_RECTANGLE_obs_str_set.append(ST_RECTANGLE_obs_str_single)
ST_RECTANGLE_obs_str = "const ST_RECTANGLE staticObs[MAXSO] = {" + ", ".join(ST_RECTANGLE_obs_str_set) + "};" # e.g., const ST_RECTANGLE staticObs[MAXSO] = {{{2000, 700}, 200, 450, 0}};


# %% construct the goal declaration
planning_problem = list(planning_problem_set.planning_problem_dict.values())
if len(planning_problem) > 1:
    logging.error("More than one ego vehicles appear in the scenario. Exiting.")
    sys.exit(1)

ST_PLANNING_str_set = []
for planning_problem_veh in planning_problem:
    try:
        goal_pos = planning_problem_veh.goal.state_list[0].position.center
    except AttributeError:
        try:
            goal_pos = planning_problem_veh.goal.state_list[0].position.shapes[0].center
        except AttributeError:
            logging.error("Could not find goal position in the specified format. Exiting.")
            sys.exit(1)
    
    goal_pos = (goal_pos*SCALE).astype(int)
    ST_PLANNING_obs_str = "{" + ", ".join(map(str, goal_pos)) + "}"
    ST_PLANNING_str_set.append(ST_PLANNING_obs_str)
ST_PLANNING_str = "const ST_PLANNING planning = {" + ", ".join(ST_PLANNING_str_set) + "};" # e.g., const ST_PLANNING planning = {{3000, 0}};


# %% construct the definition and hyperparameter declaration in c code
MAXL_str = f"const int MAXL = {MAXL};"
MAXP_str = f"const int MAXP = {MAXP};"
MAXSO_str = f"const int MAXSO = {MAXSO};"


# %% write in the .txt file
# Open a file in write mode
with open(output_file, "w") as file:
    file.write(MAXP_str + "\n")
    file.write(MAXL_str + "\n")
    file.write(MAXSO_str + "\n")
    file.write("\n")

    for i in range(MAXL):
        file.write(ST_BOUND_leftLane_str_set[i] + "\n")
        file.write(ST_BOUND_rightLane_str_set[i] + "\n")
        file.write(ST_LANE_lane_str_set[i] + "\n")
        file.write("\n")
    
    file.write(ST_LANE_laneNet_str + "\n")
    file.write("\n")

    file.write(ST_RECTANGLE_obs_str + "\n")
    file.write("\n")

    file.write(ST_PLANNING_str + "\n")
    file.write("\n")
