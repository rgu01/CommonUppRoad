from commonroad.common.file_reader import CommonRoadFileReader
import sys
import logging
import os
import numpy as np
from parseCR.utils import write_large_block


# %% load the files and parameters
template_file = os.path.dirname(__file__) + "/uppaal/template.xml"
# read template files and store lines
with open(template_file, 'r') as template_file:
        lines = template_file.readlines()

input_file_folder = os.path.dirname(__file__) + "/scenarios"
output_file_folder = os.path.dirname(__file__) + "/uppaal/models"
if not os.path.exists(output_file_folder):
        os.makedirs(output_file_folder)

for xml_file in os.listdir(input_file_folder):

    # xml_file = "ZAM_Ramp-1_1-T-1.xml" # "ZAM_Tutorial-1_2_T-1.xml" or "DEU_Ffb-1_3_T-1.xml" or "ZAM_Ramp-1_1-T-1.xml"
    xml_file_path = input_file_folder + "/" + xml_file
    file_name, _ = os.path.splitext(xml_file)

    output_file = output_file_folder + "/" + file_name + "_generated_lanelet.xml"

    SCALE = 100 # the scaling factor from double to int
    MAXT = 10
    DEFAULT_VAL = 0

    # Parse the XML file
    scenario, planning_problem_set = CommonRoadFileReader(xml_file_path).open()

    # MAXP is to set the maximal number of points of a lane
    for lane in scenario.lanelet_network.lanelets:
        # merge the lane points if they are on the same straight line 
        if np.all(lane._left_vertices[:, 1] == lane._left_vertices[0, 1]) or np.all(lane._left_vertices[:, 0] == lane._left_vertices[0, 0]):
            lane._left_vertices = lane._left_vertices[[0, -1]]
        if np.all(lane._right_vertices[:, 1] == lane._right_vertices[0, 1]) or np.all(lane._right_vertices[:, 0] == lane._right_vertices[0, 0]):
            lane._right_vertices = lane._right_vertices[[0, -1]]

    MAXP = max(len(lane._left_vertices) for lane in scenario.lanelet_network.lanelets)
    # MAXL is the number of lanes in the lanelet network
    MAXL = len(scenario.lanelet_network.lanelets)
    # MAXSO is the number of static obstacles
    MAXSO = max(1, len(scenario.static_obstacles))
    # MAXDO is the number of dynamic obstacles
    MAXDO = max(1, len(scenario.dynamic_obstacles))
    # MAXTP is the number of max points in trajectories of moving obstacles
    MAXTP = 0 if scenario.dynamic_obstacles == [] else max(len(dyn_obs.prediction.trajectory.state_list) for dyn_obs in scenario.dynamic_obstacles)
    # MAXPRE is the maximal number of predecessor lanes 
    MAXPRE = max(1, max(len(lane.predecessor) for lane in scenario.lanelet_network.lanelets))
    # MAXSUC is the maximal number of successor lanes 
    MAXSUC = max(1, max(len(lane.successor) for lane in scenario.lanelet_network.lanelets))
    # TIMESTEPSIZE is the step time size
    TIMESTEPSIZE = scenario.dt


    # %% construct the lanelet network declaration in c code
    ST_BOUND_leftLane_str_set = []
    ST_BOUND_rightLane_str_set = []
    ST_LANE_lane_str_set = []
    ST_LANE_laneNet_str = []
    for i, lane in enumerate(scenario.lanelet_network.lanelets):
        # get lane information
        lane_ID = lane.lanelet_id
        lane_predecessor = [] if lane.predecessor == [] else lane.predecessor # TODO: check predecessor and successor, they can be multiple
        lane_successor = [] if lane.successor == [] else lane.successor
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
        # append {NONE, NONE} to meet the fixed-length array
        leftLane_str = "{" + ", ".join(["{" + ", ".join(map(str, point)) + "}" for point in leftLane]) + (MAXP - len(leftLane))*", {NONE, NONE}" + "}" # e.g., '{{-86, 7}, {-64, 6}, {NONE, NONE}}'
        ST_BOUND_left = f"const ST_BOUND leftLane{i + 1} = {{{leftLane_str}, {lane_markingLeft}}};" # e.g., 'const ST_BOUND leftLane1 = {{{-86, 7}, {-64, 6}}, True};'
        rightLane_str = "{" + ", ".join(["{" + ", ".join(map(str, point)) + "}" for point in rightLane]) + (MAXP - len(rightLane))*", {NONE, NONE}" + "}"
        ST_BOUND_right = f"const ST_BOUND rightLane{i + 1} = {{{rightLane_str}, {lane_markingRight}}};"

        # construct the lane string declaration, e.g., 'const ST_LANE lane1 ={1, leftLane1, rightLane1, None, None, 2, False, None, False'
        # extend the lane_predecessor and lane_successor to meet the fixed-length array
        lane_predecessor += (MAXPRE - len(lane_predecessor))*[None]
        lane_successor += (MAXSUC - len(lane_successor))*[None]
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

    # If static obstacles is zero, do not define “statisObs[MAXSO]”.
    if len(ST_RECTANGLE_obs_str_set) == 0:
        ST_RECTANGLE_obs_str1 = "const bool staticObsExists = false;"
        ST_RECTANGLE_obs_str2 = "const ST_RECTANGLE staticObs[MAXSO] = {{{NONE, NONE}, NONE, NONE, NONE}};"

    else:
        ST_RECTANGLE_obs_str1 = "const bool staticObsExists = true;"
        ST_RECTANGLE_obs_str2 = "const ST_RECTANGLE staticObs[MAXSO] = {" + ", ".join(ST_RECTANGLE_obs_str_set) + "};" # e.g., const ST_RECTANGLE staticObs[MAXSO] = {{{2000, 700}, 200, 450, 0}};

    # %% construct the dynamic obstacles declaration in c code
    initCS_str_set = []
    shapeObs_str_set = []
    trajectory_str_set = []
    MovingObs_str_set = []
    Obs_naming_set = []

    for dyn_obs in scenario.dynamic_obstacles:
        obs_width = int(dyn_obs._obstacle_shape._width*SCALE)
        obs_length = int(dyn_obs._obstacle_shape._length*SCALE)
        obs_id = dyn_obs.obstacle_id

        # const ST_CSTATE initCS1 = {{2.25, 3.50}, 2.30, 0.0, 0.0, 0.0};  // initial state of a moving obstacle
        obs_ini_pos = dyn_obs._initial_state.position
        obs_ini_vel = dyn_obs._initial_state.velocity
        obs_ini_ori = dyn_obs._initial_state.orientation
        obs_ini_acc = dyn_obs._initial_state.acceleration
        obs_ini_yaw = dyn_obs._initial_state.yaw_rate
        ST_RECTANGLE_obs_ini_pos = "{" + ", ".join(map(str, obs_ini_pos)) + "}"
        initCS_str = f"const ST_CSTATE initCS{obs_id} = {{" + ", ".join([ST_RECTANGLE_obs_ini_pos, f"{obs_ini_vel}", f"{obs_ini_ori}", f"{obs_ini_acc}", f"{obs_ini_yaw}"]) + "};"
        initCS_str_set.append(initCS_str)

        # const ST_RECTANGLE shapeObs1 = {{225, 350}, 200, 450, 0};       // shape of a moving obstacle 
        obs_ini_pos_int = "{" + ", ".join(map(str, (obs_ini_pos*SCALE).astype(int))) + "}"
        shapeObs_str = f"const ST_RECTANGLE shapeObs{obs_id} = {{" + ", ".join([obs_ini_pos_int, f"{int(dyn_obs._obstacle_shape._width*SCALE)}", f"{int(dyn_obs._obstacle_shape._length*SCALE)}", f"{int(dyn_obs._obstacle_shape._orientation*SCALE)}"]) + "};"
        shapeObs_str_set.append(shapeObs_str)

        # const ST_PAIR trajectory1[MAXTP] = {{0,{{-20.0,-0.4},3.5,0.0,0.0,0.0}},{1,{{-23.5,-0.4},3.5,0.0,0.0,0.0}},{MAXTIME,{{15.0,-0.4},3.5,0.0,0.0,0.0}},PHOLDER,PHOLDER}; // the trajectory
        trajectory_set = []
        initial_time_step = dyn_obs.prediction.initial_time_step
        len_traj = len(dyn_obs.prediction.trajectory.state_list)
        for i in range(MAXTP):
            time_step = i + initial_time_step
            if i+1 > len_traj:
                traj_str = "PHOLDER"
            else:
                dyn_obs_pos = dyn_obs.prediction.trajectory.state_list[i]
                center_pos = dyn_obs_pos.position
                ori = dyn_obs_pos.orientation
                # if not given, use the default value
                spd = dyn_obs_pos.velocity if hasattr(dyn_obs_pos, 'velocity') else DEFAULT_VAL
                acc = dyn_obs_pos.acceleration if hasattr(dyn_obs_pos, 'acceleration') else DEFAULT_VAL
                yaw = DEFAULT_VAL
                pos_str = "{" + ", ".join(map(str, center_pos)) + "}"
                veh_str = "{" + ", ".join([pos_str, f"{spd}", f"{ori}", f"{acc}", f"{yaw}"]) + "}"
                traj_str = "{" + str(int(time_step)) + ", " + veh_str + "}"
            trajectory_set.append(traj_str)
        trajectory_str = f"const ST_PAIR trajectory{obs_id}[MAXTP] = {{" + ", ".join(trajectory_set) + "};"
        trajectory_str_set.append(trajectory_str)

        # obs1 = MovingObs(1, initCS1, shapeObs1, tLen1, trajectory1);
        MovingObs_str = f"obs{obs_id} = MovingObs({obs_id}, initCS{obs_id}, shapeObs{obs_id}, trajectory{obs_id});"
        MovingObs_str_set.append(MovingObs_str)

        # Obs_naming
        Obs_naming_set.append(f"obs{obs_id}")

    # %% construct the goal declaration
    planning_problem = list(planning_problem_set.planning_problem_dict.values())

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
    P_str = "const int P = 1;"
    MAX_TIME_str = f"const uint8_t MAXTIME = {MAXT};"
    MAXL_str = f"const int MAXL = {MAXL};"
    NONE_str = "const int NONE = -1;"
    MAXP_str = f"const int MAXP = {MAXP};"
    MAXSO_str = f"const int MAXSO = {MAXSO};"
    MAXDO_str = f"const int MAXDO = {MAXDO};"
    MAXTP_str = f"const int MAXTP = {MAXTP};"
    MAXPRE_str = f"const int MAXPRE = {MAXPRE};"
    MAXSUC_str = f"const int MAXSUC = {MAXSUC};"
    TIMESTEPSIZE_str = f"const double TIMESTEPSIZE = {TIMESTEPSIZE};"
    SCALE_str = "const double SCALE = 100.0;"
    THRES_str = "const int THRESHOLD = 200;"
    PHOLDER_str = "const ST_PAIR PHOLDER = {NONE,{{NONE,NONE},NONE,NONE,NONE,NONE}};"
    
    if Obs_naming_set == []:
        system_str = "system Decisions, egoController, egoDynamics;"
    else:
        system_str = "system Decisions, egoController, egoDynamics, " + ", ".join(Obs_naming_set) + ";"

    # %% write in the xml templates
    scenario_prompt = "<declaration>// Generated scenario starts"
    moving_obs_prompt = "<system>// Generated moving obstacles starts"
    model_prompt = "// Generated model instances start"

    with open(output_file, 'w') as file:
        for line in lines:
            file.write(line)
            if line.strip() == scenario_prompt:
                file.write(P_str + "\n")
                file.write(MAX_TIME_str + "\n")
                file.write(MAXP_str + "\n")
                file.write(NONE_str + "\n")
                file.write(MAXL_str + "\n")
                file.write(MAXSO_str + "\n")
                file.write(MAXDO_str + "\n")
                file.write(MAXTP_str + "\n")
                file.write(MAXPRE_str + "\n")
                file.write(MAXSUC_str + "\n")
                file.write(SCALE_str + "\n")
                file.write(THRES_str + "\n")
                file.write(TIMESTEPSIZE_str + "\n")
                
                # Call the function to write the large static block
                write_large_block(file)

                for i in range(MAXL):
                    # replace 'False, True, None' in python to 'false, true, NONE' in c
                    ST_BOUND_leftLane = ST_BOUND_leftLane_str_set[i].replace('False', 'false').replace('True', 'true')
                    ST_BOUND_rightLane = ST_BOUND_rightLane_str_set[i].replace('False', 'false').replace('True', 'true')
                    ST_LANE_lane = ST_LANE_lane_str_set[i].replace('False', 'false').replace('True', 'true').replace('None', 'NONE').replace('[', '{').replace(']', '}')
                    file.write(ST_BOUND_leftLane + "\n")
                    file.write(ST_BOUND_rightLane + "\n")
                    file.write(ST_LANE_lane + "\n")
                    file.write("\n")
                
                file.write(ST_LANE_laneNet_str + "\n")
                file.write("\n")

                file.write(ST_RECTANGLE_obs_str1 + "\n")
                file.write(ST_RECTANGLE_obs_str2 + "\n")
                file.write("\n")

                file.write(ST_PLANNING_str + "\n")
                file.write("\n")
            
            if line.strip() == moving_obs_prompt:
                declear_flag = 0
                for i in range(len(scenario.dynamic_obstacles)):
                    file.write(initCS_str_set[i] + "\n")
                    file.write(shapeObs_str_set[i] + "\n")
                    if declear_flag == 0:
                        file.write(PHOLDER_str + "\n")
                        declear_flag = 1
                    file.write(trajectory_str_set[i] + "\n")
                    file.write(MovingObs_str_set[i] + "\n")

            if line.strip() == model_prompt:
                file.write(system_str + "\n")