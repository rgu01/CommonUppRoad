#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define MAXP 40
#define NONE -1
#define MAXL 11
#define MAXSO 1
#define MAXPRE 1
#define MAXSUC 1
#define SCALE 100.0 // scale from int to double: 100 = 1.0

typedef int id_t;

typedef struct {
    int32_t x;
    int32_t y;
}ST_DPOINT;

typedef struct {
    double x;
    double y;
}ST_CPOINT;

typedef struct {
    ST_DPOINT ends[2];
}ST_DLINE;

typedef struct {
    ST_DPOINT points[MAXP];
    bool dashLine;
}ST_BOUND;

typedef struct {
    id_t ID;
    ST_BOUND left;
    ST_BOUND right;
    id_t predecessor[MAXPRE];
    id_t successor[MAXSUC];
    id_t adjLeft;
    bool dirLeft;
    id_t adjRight;
    bool dirRight;
}ST_LANE;

typedef struct {
    bool collide; // collision
    bool outside; // outside road
    bool reach;   // reach the goal
}ST_DETECTION;

typedef struct {
    ST_CPOINT position;
    double velocity;
    double orientation;
    double acceleration;
    double yawRate;
}ST_CSTATE;

typedef struct {
    ST_DPOINT position;
    int16_t velocity;
    int16_t orientation;
    int16_t acceleration;
    int16_t yawRate;
    ST_DETECTION detection;
}ST_DSTATE;

typedef struct {
    ST_DPOINT center;
    int16_t width;
    int16_t length;
    int16_t orientation;
}ST_RECTANGLE;

typedef struct {
    int maxVelocity;
    int minVelocity;
    int maxOrientation;
    int minOrientation;
}ST_RULES;

typedef struct {
    ST_DPOINT goal;
}ST_PLANNING;

double i2d(const int i) {
    double v = i/SCALE;
    return v;
}

int d2i(const double i) {
    int v = rint(i*SCALE);
    return v;
}

// Function to calculate the corner points of a rectangle
void calculateCornerPoints(ST_RECTANGLE veh_state, ST_DPOINT corners[]) {
    double halfLength = i2d(veh_state.length)/2.0;
    double halfWidth = i2d(veh_state.width)/2.0;
    double angle = i2d(veh_state.orientation);

    // Calculate the coordinates of each corner point based on the center, length, width, and orientation
    corners[0].x = d2i(i2d(veh_state.center.x) + halfLength*cos(angle) - halfWidth*sin(angle));
    corners[0].y = d2i(i2d(veh_state.center.y) + halfLength*sin(angle) + halfWidth*cos(angle));

    corners[1].x = d2i(i2d(veh_state.center.x) + halfLength*cos(angle) + halfWidth*sin(angle));
    corners[1].y = d2i(i2d(veh_state.center.y) + halfLength*sin(angle) - halfWidth*cos(angle));

    corners[2].x = d2i(i2d(veh_state.center.x) - halfLength*cos(angle) + halfWidth*sin(angle));
    corners[2].y = d2i(i2d(veh_state.center.y) - halfLength*sin(angle) - halfWidth*cos(angle));

    corners[3].x = d2i(i2d(veh_state.center.x) - halfLength*cos(angle) - halfWidth*sin(angle));
    corners[3].y = d2i(i2d(veh_state.center.y) - halfLength*sin(angle) + halfWidth*cos(angle));
}

// Function to check if all elements in the vector (size=4) have the same sign
bool same_sign(int vec[]) {
    int i = 0;
    int sign = (vec[0] >= 0) ? 1 : -1; // Get the sign of the first element
    for (i = 1; i < 4; i++) {
        if ((vec[i] >= 0) != (sign >= 0)) {
            return false; // Different signs found
        }
    }
    return true; // All elements have the same sign
}

// check if pt1 is on the same line defined by pt2-pt3
int check_online(ST_DPOINT pt1, ST_DPOINT pt2, ST_DPOINT pt3) {
    int dis12 = sqrt(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2));
    int dis13 = sqrt(pow(pt1.x - pt3.x, 2) + pow(pt1.y - pt3.y, 2));
    int dis23 = sqrt(pow(pt2.x - pt3.x, 2) + pow(pt2.y - pt3.y, 2));
    if (dis12 + dis13 == dis23)
        return 1;
    else
        return 0; 
}

// Check if any corner of box2 is outside box1
int check_coverage(ST_DPOINT box1[], ST_DPOINT box2[]) {
    int i = 0, j = 0;
    int abx = 0, aby = 0, apx = 0, apy = 0;
    int cross_prod[4];
    int inside_sum = 0;
    int is_online = 0;
    // Check if all corners of box2 fall outside the bounding box of box1   
    for (i = 0; i < 4; i++) {
        // get the x y coordinate of the test points
        for (j = 0; j < 4; j++) {
            abx = box1[(j+1)%4].x - box1[j].x; // when j+1=4, back to the first one
            aby = box1[(j+1)%4].y - box1[j].y;
            apx = box2[i].x - box1[j].x;
            apy = box2[i].y - box1[j].y;
            // cross product of ab and ap
            cross_prod[j] = abx*apy - apx*aby;
            // check if on the line
            if (check_online(box2[i], box1[j], box1[(j+1)%4]) == 1)
                is_online = 1;
        }
        // if all the cross production have the same sign, then the test point is within the box1
        if (same_sign(cross_prod) && is_online == 0)
            inside_sum++;
    }
    return inside_sum;
}

// Function to count non-zero elements in a 2D array
int check_pts_num(ST_DPOINT lane_pts[]) {
    int count = 0;
    // NONE or 0?
    while (count < MAXP && (lane_pts[count].x != NONE || lane_pts[count].y != NONE)){
        count++;
    }
    return count;
}

// check if veh_state are not covered by a single lane, or if vehicle rectangle touches the edge of the lane
int check_inlane_lane_single(const ST_LANE lane, ST_RECTANGLE veh_state, ST_DPOINT veh_corners[], ST_DPOINT box_corners[]) {
    int num_box = 0;
    int i_box = 0;
    int inlane_pts_num = 0;

    //ST_DPOINT veh_corners[4];
    //ST_DPOINT box_corners[4];
    // Calculate the corner points
    calculateCornerPoints(veh_state, veh_corners);

    // check the number of points in the lane
    num_box = check_pts_num(lane.left.points) - 1;
    for (i_box = 0; i_box < num_box; i_box++){
        // define the corner of the road box
        box_corners[0] = lane.right.points[i_box];
        box_corners[1] = lane.right.points[i_box + 1];
        box_corners[2] = lane.left.points[i_box + 1];
        box_corners[3] = lane.left.points[i_box];
        // check if the inlane status of the vehicle box to the current box
        inlane_pts_num += check_coverage(box_corners, veh_corners);
        //if (if_inlane == 1)
            // this means the vehicle cross the lane
            //return false;
        //else if (if_inlane == 2)
            // this means the vehicle is within the lane
            //return true;            
    }
    // fully outside the lane network 
    //return false;
    if(inlane_pts_num < 4) {
        return false;
    }
    else {
        return true;
    }
}

// check if veh_state are not covered by laneNet, or if vehicle rectangle touches laneNet
// can this function call check_inlane_lane_single?
int check_inlane_laneNet(ST_LANE laneNet[], ST_RECTANGLE veh_state, int *lane, ST_DPOINT veh_corners[], ST_DPOINT box_corners[]) {
    int i_lane = 0;
    int num_box = 0;
    int i_box = 0;
    int inlane_pts_num = 0;
    //ST_DPOINT veh_corners[4];
    //ST_DPOINT box_corners[4];

    // Calculate the corner points
    calculateCornerPoints(veh_state, veh_corners);
    for(i_lane = 0; i_lane < MAXL; i_lane++){
        // check the number of points in each lane
        num_box = check_pts_num(laneNet[i_lane].left.points) - 1;
        for (i_box = 0; i_box < num_box; i_box++){
            // define the corner of the road box
            box_corners[0] = laneNet[i_lane].right.points[i_box];
            box_corners[1] = laneNet[i_lane].right.points[i_box + 1];
            box_corners[2] = laneNet[i_lane].left.points[i_box + 1];
            box_corners[3] = laneNet[i_lane].left.points[i_box];
            // check if the inlane status of the vehicle box to the current box
            inlane_pts_num += check_coverage(box_corners, veh_corners);     
        }
    }
    if(inlane_pts_num < 4) {
        return false;
    }
    else {
        return true;
    }
}

int main() {
    const ST_BOUND leftLane1 = {{{-3928, -215}, {-2081, -196}, {-2070, -196}, {-3, -174}, {7, -174}, {2138, -152}, {2149, -152}, {2672, -147}, {2674, -147}, {3202, -141}, {3218, -141}, {3734, -136}, {3754, -136}, {4291, -130}, {4303, -130}, {7189, -100}, {7201, -100}, {9511, -76}, {9522, -76}, {12009, -50}, {12020, -50}, {15467, -15}, {15478, -15}, {18208, 13}, {18219, 13}, {20736, 39}, {20747, 39}, {22816, 60}, {22828, 60}, {25679, 90}, {25691, 90}, {28619, 120}, {28630, 120}, {31057, 145}, {31068, 145}, {33414, 170}, {33426, 170}, {36391, 200}, {36402, 200}, {38727, 224}}, false};
    const ST_BOUND rightLane1 = {{{-3913, -588}, {-2077, -569}, {-2066, -569}, {0, -547}, {11, -547}, {2142, -525}, {2153, -525}, {2676, -520}, {2678, -520}, {3206, -514}, {3222, -514}, {3737, -509}, {3758, -508}, {4295, -503}, {4306, -503}, {7193, -473}, {7204, -473}, {9515, -449}, {9526, -449}, {12013, -423}, {12024, -423}, {15471, -388}, {15482, -388}, {18212, -359}, {18223, -359}, {20740, -333}, {20751, -333}, {22820, -312}, {22831, -312}, {25683, -282}, {25695, -282}, {28623, -252}, {28634, -252}, {31061, -227}, {31072, -227}, {33418, -202}, {33429, -202}, {36394, -172}, {36406, -171}, {38742, -147}}, false};
    const ST_LANE lane1 = {30622, leftLane1, rightLane1, {NONE}, {NONE}, 30624, true, NONE, false};

    const ST_BOUND leftLane2 = {{{-3927, 134}, {-2085, 153}, {-2080, 153}, {-7, 174}, {-1, 174}, {2134, 196}, {2139, 196}, {2670, 202}, {2683, 202}, {3198, 207}, {3222, 208}, {3730, 213}, {3735, 213}, {4288, 219}, {4293, 219}, {7186, 248}, {7191, 248}, {9507, 272}, {9513, 272}, {12005, 298}, {12010, 298}, {15464, 334}, {15469, 334}, {18204, 362}, {18209, 362}, {20733, 388}, {20738, 388}, {22813, 410}, {22818, 410}, {25676, 439}, {25681, 439}, {28615, 470}, {28621, 470}, {31053, 495}, {31058, 495}, {33411, 519}, {33416, 519}, {36387, 550}, {36392, 550}, {38728, 574}}, false};
    const ST_BOUND rightLane2 = {{{-3928, -215}, {-2081, -196}, {-2076, -196}, {-3, -174}, {1, -174}, {2138, -152}, {2143, -152}, {2674, -147}, {2687, -147}, {3202, -141}, {3226, -141}, {3734, -136}, {3739, -136}, {4291, -130}, {4297, -130}, {7189, -100}, {7195, -100}, {9511, -76}, {9516, -76}, {12009, -50}, {12014, -50}, {15467, -15}, {15472, -15}, {18208, 13}, {18213, 13}, {20736, 39}, {20741, 39}, {22816, 60}, {22822, 60}, {25679, 90}, {25685, 90}, {28619, 120}, {28624, 120}, {31057, 145}, {31062, 145}, {33414, 170}, {33420, 170}, {36391, 200}, {36396, 200}, {38727, 224}}, false};
    const ST_LANE lane2 = {30624, leftLane2, rightLane2, {NONE}, {NONE}, 30626, true, 30622, true};

    const ST_BOUND leftLane3 = {{{-3949, 487}, {-2102, 506}, {-2083, 506}, {-23, 527}, {-5, 528}, {2117, 549}, {2136, 550}, {2648, 555}, {2679, 555}, {3182, 560}, {3219, 561}, {3700, 566}, {3731, 566}, {4271, 572}, {4289, 572}, {7169, 602}, {7187, 602}, {9491, 626}, {9509, 626}, {11988, 651}, {12007, 651}, {15447, 687}, {15465, 687}, {18187, 715}, {18206, 715}, {20716, 741}, {20734, 742}, {22796, 763}, {22814, 763}, {25659, 792}, {25677, 793}, {28599, 823}, {28617, 823}, {31036, 848}, {31055, 848}, {33394, 872}, {33412, 872}, {36370, 903}, {36389, 903}, {38706, 927}}, false};
    const ST_BOUND rightLane3 = {{{-3927, 134}, {-2098, 153}, {-2080, 153}, {-20, 174}, {-1, 174}, {2121, 196}, {2139, 196}, {2652, 202}, {2683, 202}, {3186, 207}, {3222, 208}, {3704, 212}, {3735, 213}, {4275, 218}, {4293, 219}, {7173, 248}, {7191, 248}, {9494, 272}, {9513, 272}, {11992, 298}, {12010, 298}, {15450, 334}, {15469, 334}, {18191, 362}, {18209, 362}, {20719, 388}, {20738, 388}, {22800, 410}, {22818, 410}, {25663, 439}, {25681, 439}, {28602, 469}, {28621, 470}, {31040, 495}, {31058, 495}, {33398, 519}, {33416, 519}, {36374, 550}, {36392, 550}, {38728, 574}}, false};
    const ST_LANE lane3 = {30626, leftLane3, rightLane3, {NONE}, {NONE}, NONE, false, 30624, true};

    const ST_LANE laneNet[MAXL] = {lane1, lane2, lane3};

    const ST_RECTANGLE ego = {{-2102, -11}, 100, 450, 0};

    int lane;
    ST_DPOINT veh_corners[4], box_corners[4];

    //calculateCornerPoints(ego, veh_corners);

    bool is_inlane_net = check_inlane_laneNet(laneNet, ego, &lane, veh_corners, box_corners); 
    if (is_inlane_net)
        printf("The vehicle is in the lane network.\n");
    else
        printf("The vehicle is not in the lane network.\n");
        
    printf("vehicle corners: (%d, %d), (%d, %d), (%d, %d), (%d, %d)\n", veh_corners[0].x,  veh_corners[0].y, 
    veh_corners[1].x,  veh_corners[1].y, veh_corners[2].x,  veh_corners[2].y, veh_corners[3].x,  veh_corners[3].y);

    for(int i = 0; i < 3; i++) {
        bool is_inlane_id = check_inlane_lane_single(laneNet[i], ego, veh_corners, box_corners);
        if (is_inlane_id)
            printf("The vehicle is in the single lane: %d.\n", laneNet[i].ID);
        else
            printf("The vehicle is not in the single lane: %d.\n", laneNet[i].ID);
    }

    return 0;
}



