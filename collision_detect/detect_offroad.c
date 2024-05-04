#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#define MAXP 50
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

// Check if any corner of box2 is outside box1
int check_coverage(ST_DPOINT box1[], ST_DPOINT box2[]) {
    int i = 0, j = 0;
    int abx = 0, aby = 0, apx = 0, apy = 0;
    int cross_prod[4];
    int inside_sum = 0;
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
        }
        // if all the cross production have the same sign, then the test point is within the box1
        if (same_sign(cross_prod))
            inside_sum++;
    }
    // check the sum of the identifier
    if (inside_sum == 4)
        return 2; // the box2 is fully inside the box1
    else if (inside_sum == 0)
        return 0; // the box2 is fully outside the box1
    else
        return 1; // the box2 is partially outside the box1, which means cross the lane
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
    int if_inlane = 0;

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
        if_inlane = check_coverage(box_corners, veh_corners);
        if (if_inlane == 1)
            // this means the vehicle cross the lane
            return false;
        else if (if_inlane == 2)
            // this means the vehicle is within the lane
            return true;            
    }
    // fully outside the lane network 
    return false;
}

// check if veh_state are not covered by laneNet, or if vehicle rectangle touches laneNet
// can this function call check_inlane_lane_single?
int check_inlane_laneNet(ST_LANE laneNet[], ST_RECTANGLE veh_state, int *lane, ST_DPOINT veh_corners[], ST_DPOINT box_corners[]) {
    int i_lane = 0;
    int num_box = 0;
    int i_box = 0;
    int if_inlane = 0;
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
            if_inlane = check_coverage(box_corners, veh_corners);
            if (if_inlane == 1) {
                // this means the vehicle cross the lane
                *lane = laneNet[i_lane].ID;
                return false;
            }
            else if (if_inlane == 2) {
                // this means the vehicle is within the lane
                *lane = -1;
                return true;      
            }      
        }
    }
    // fully outside the lane network 
    lane = MAXL + 1;
    return false;
}

int main() {
    const ST_BOUND leftLane1 = {{{0, -150}, {40, -142}, {81, -134}, {122, -127}, {163, -120}, {204, -113}, {244, -107}, {285, -101}, {326, -95}, {367, -89}, {408, -84}, {448, -79}, {489, -74}, {530, -70}, {571, -66}, {612, -61}, {653, -58}, {693, -54}, {734, -50}, {775, -47}, {816, -44}, {857, -40}, {897, -38}, {938, -35}, {979, -32}, {1020, -29}, {1061, -27}, {1102, -25}, {1142, -23}, {1183, -20}, {1224, -18}, {1265, -17}, {1306, -15}, {1346, -13}, {1387, -12}, {1428, -10}, {1469, -9}, {1510, -7}, {1551, -6}, {1591, -5}, {1632, -4}, {1673, -3}, {1714, -2}, {1755, -2}, {1795, -1}, {1836, 0}, {1877, 0}, {1918, 0}, {1959, 0}, {2000, 0}}, false};
    const ST_BOUND rightLane1 = {{{0, -500}, {40, -493}, {81, -488}, {122, -482}, {163, -476}, {204, -470}, {244, -465}, {285, -460}, {326, -455}, {367, -449}, {408, -445}, {448, -440}, {489, -435}, {530, -430}, {571, -426}, {612, -422}, {653, -418}, {693, -413}, {734, -410}, {775, -406}, {816, -402}, {857, -398}, {897, -395}, {938, -392}, {979, -389}, {1020, -385}, {1061, -383}, {1102, -380}, {1142, -377}, {1183, -374}, {1224, -372}, {1265, -370}, {1306, -368}, {1346, -365}, {1387, -364}, {1428, -362}, {1469, -360}, {1510, -358}, {1551, -357}, {1591, -356}, {1632, -355}, {1673, -353}, {1714, -353}, {1755, -352}, {1795, -351}, {1836, -350}, {1877, -350}, {1918, -350}, {1959, -350}, {2000, -350}}, false};
    const ST_LANE lane1 = {2, leftLane1, rightLane1, {NONE}, {2}, NONE, false, NONE, false};

    const ST_BOUND leftLane2 = {{{2000, 0}, {16000, 0}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_BOUND rightLane2 = {{{2000, -350}, {16000, -350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_LANE lane2 = {3, leftLane2, rightLane2, {2}, {4}, 6, true, NONE, false};

    const ST_BOUND leftLane3 = {{{16000, 0}, {16105, 8}, {16210, 22}, {16315, 40}, {16421, 62}, {16526, 87}, {16631, 113}, {16736, 141}, {16842, 168}, {16947, 196}, {17052, 222}, {17157, 247}, {17263, 270}, {17368, 290}, {17473, 308}, {17578, 323}, {17684, 334}, {17789, 343}, {17894, 348}, {18000, 350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_BOUND rightLane3 = {{{16000, -350}, {16105, -341}, {16210, -327}, {16315, -309}, {16421, -287}, {16526, -262}, {16631, -236}, {16736, -208}, {16842, -181}, {16947, -153}, {17052, -127}, {17157, -102}, {17263, -79}, {17368, -59}, {17473, -41}, {17578, -26}, {17684, -15}, {17789, -6}, {17894, -1}, {18000, 0}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_LANE lane3 = {4, leftLane3, rightLane3, {3}, {3}, 8, true, NONE, false};

    const ST_BOUND leftLane4 = {{{0, 350}, {2000, 350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_BOUND rightLane4 = {{{0, 0}, {2000, 0}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_LANE lane4 = {5, leftLane4, rightLane4, {NONE}, {6}, 9, true, NONE, false};

    const ST_BOUND leftLane5 = {{{2000, 350}, {16000, 350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_BOUND rightLane5 = {{{2000, 0}, {16000, 0}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_LANE lane5 = {6, leftLane5, rightLane5, {6}, {7}, 10, true, 3, true};

    const ST_BOUND leftLane6 = {{{16000, 350}, {18000, 350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_BOUND rightLane6 = {{{16000, 0}, {18000, 0}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_LANE lane6 = {7, leftLane6, rightLane6, {6}, {8}, 11, true, 4, true};

    const ST_BOUND leftLane7 = {{{18000, 350}, {24000, 350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_BOUND rightLane7 = {{{18000, 0}, {24000, 0}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_LANE lane7 = {8, leftLane7, rightLane7, {7}, {NONE}, 12, true, NONE, false};

    const ST_BOUND leftLane8 = {{{0, 700}, {2000, 700}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_BOUND rightLane8 = {{{0, 350}, {2000, 350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_LANE lane8 = {9, leftLane8, rightLane8, {NONE}, {10}, NONE, false, 5, true};

    const ST_BOUND leftLane9 = {{{2000, 700}, {16000, 700}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_BOUND rightLane9 = {{{2000, 350}, {16000, 350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_LANE lane9 = {10, leftLane9, rightLane9, {9}, {11}, NONE, false, 6, true};

    const ST_BOUND leftLane10 = {{{16000, 700}, {18000, 700}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_BOUND rightLane10 = {{{16000, 350}, {18000, 350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_LANE lane10 = {11, leftLane10, rightLane10, {10}, {12}, NONE, false, 7, true};

    const ST_BOUND leftLane11 = {{{18000, 700}, {24000, 700}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, false};
    const ST_BOUND rightLane11 = {{{18000, 350}, {24000, 350}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}, {NONE, NONE}}, true};
    const ST_LANE lane11 = {12, leftLane11, rightLane11, {11}, {NONE}, NONE, false, 8, true};

    const ST_LANE laneNet[MAXL] = {lane1, lane2, lane3, lane4, lane5, lane6, lane7, lane8, lane9, lane10, lane11};

    const ST_RECTANGLE ego = {{276, 291}, 100, 450, 40};

    int lane;
    ST_DPOINT veh_corners[4], box_corners[4];

    calculateCornerPoints(ego, veh_corners);

    bool is_inlane_net = check_inlane_laneNet(laneNet, ego, &lane, veh_corners, box_corners);
    bool is_inlane_id = check_inlane_lane_single(laneNet[3], ego, veh_corners, box_corners);
    
    if (is_inlane_net)
        printf("The vehicle is in the lane network.\n");
    else
        printf("The vehicle is not in the lane network.\n");
    
    if (is_inlane_id)
        printf("The vehicle is in the single lane.\n");
    else
        printf("The vehicle is not in the single lane.\n");

    return 0;
}



