#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

# define MAXP 200
# define MAXL 3
# define MAXSO 1
# define SCALE 100

typedef struct {
    int32_t x;
    int32_t y;
}ST_DPOINT;

typedef struct {
    ST_DPOINT ends[2];
}ST_DLINE;

typedef struct {
    ST_DPOINT points[MAXP];
    bool dashLine;
}ST_BOUND;

typedef struct {
    int ID;
    ST_BOUND left;
    ST_BOUND right;
    int predecessor;
    int successor;
    int adjLeft;
    bool dirLeft;
    int adjRight;
    bool dirRight;
}ST_LANE;

typedef struct {
    bool collide; // collision
    bool outside; // outside road
    bool reach;   // reach the goal
}ST_DETECTION;

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


// Function to calculate the corner points of a rectangle
void calculateCornerPoints(ST_RECTANGLE veh_state, ST_DPOINT corners[]) {
    double halfLength = veh_state.length/2;
    double halfWidth = veh_state.width/SCALE/2.0;
    double angle = veh_state.orientation/SCALE;

    // Calculate the coordinates of each corner point based on the center, length, width, and orientation
    corners[0].x = (int)((veh_state.center.x/SCALE + halfLength*cos(angle) - halfWidth*sin(angle))*SCALE);
    corners[0].y = (int)((veh_state.center.y/SCALE + halfLength*sin(angle) + halfWidth*cos(angle))*SCALE);

    corners[1].x = (int)((veh_state.center.x/SCALE + halfLength*cos(angle) + halfWidth*sin(angle))*SCALE);
    corners[1].y = (int)((veh_state.center.y/SCALE + halfLength*sin(angle) - halfWidth*cos(angle))*SCALE);

    corners[2].x = (int)((veh_state.center.x/SCALE - halfLength*cos(angle) + halfWidth*sin(angle))*SCALE);
    corners[2].y = (int)((veh_state.center.y/SCALE - halfLength*sin(angle) - halfWidth*cos(angle))*SCALE);

    corners[3].x = (int)((veh_state.center.x/SCALE - halfLength*cos(angle) - halfWidth*sin(angle))*SCALE);
    corners[3].y = (int)((veh_state.center.y/SCALE - halfLength*sin(angle) + halfWidth*cos(angle))*SCALE);
}


// Function to check if all elements in the vector (size=4) have the same sign
int same_sign(int vec[]) {
    int sign = (vec[0] >= 0) ? 1 : -1; // Get the sign of the first element
    for (int i = 1; i < 4; i++) {
        if ((vec[i] >= 0) != (sign >= 0)) {
            return 0; // Different signs found
        }
    }
    return 1; // All elements have the same sign
}


// Check if any corner of box2 is outside box1
int check_coverage(ST_DPOINT box1[4], ST_DPOINT box2[4]) {

    int inside_sum = 0;
    // Check if all corners of box2 fall outside the bounding box of box1   
    for (int i = 0; i < 4; i++) {
        int cross_prod[4];
        // get the x y coordinate of the test points
        for (int j = 0; j < 4; j++) {
            int abx = box1[(j+1)%4].x - box1[j].x; // when j+1=4, back to the first one
            int aby = box1[(j+1)%4].y - box1[j].y;
            int apx = box2[i].x - box1[j].x;
            int apy = box2[i].y - box1[j].y;
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


int check_inlane(ST_LANE laneNet[], ST_RECTANGLE veh_state) {
    ST_DPOINT veh_corners[4];
    // Calculate the corner points
    calculateCornerPoints(veh_state, veh_corners);
    // check if veh_state are not covered by laneNet, or if vehicle rectangle touches laneNet
    for(int i_lane = 0; i_lane < MAXL; i_lane++){
        // check the number of points in each lane
        int num_box = sizeof(laneNet[i_lane].left.points)/sizeof(laneNet[i_lane].left.points[0]) - 1;
        for (int i_box = 0; i_box < num_box - 1; i_box++){
            // define the corner of the road box
            ST_DPOINT box_corners[4];
            box_corners[0] = laneNet[i_lane].right.points[i_box];
            box_corners[1] = laneNet[i_lane].right.points[i_box + 1];
            box_corners[2] = laneNet[i_lane].left.points[i_box + 1];
            box_corners[3] = laneNet[i_lane].left.points[i_box];
            // check if the inlane status of the vehicle box to the current box
            int if_inlane = check_coverage(box_corners, veh_corners);
            if (if_inlane == 1)
                // this means the vehicle cross the lane
                return false;
            else if (if_inlane == 2)
                // this means the vehicle is within the lane
                return true;            
        }
    }
    // fully outside the lane network 
    return false;
}

int main() {
    const ST_BOUND leftLane1 = {{{0, 175}, {19900, 175}}, false};
    const ST_BOUND rightLane1 = {{{0, -175}, {19900, -175}}, false};
    const ST_LANE lane1 = {1, leftLane1, rightLane1, NULL, NULL, 2, false, NULL, false};
    const ST_BOUND leftLane2 = {{{0, 525}, {19900, 525}}, false};
    const ST_BOUND rightLane2 = {{{0, 175}, {19900, 175}}, false};
    const ST_LANE lane2 = {2, leftLane2, rightLane2, NULL, NULL, 3, true, 1, false};
    const ST_BOUND leftLane3 = {{{0, 875}, {19900, 875}}, false};
    const ST_BOUND rightLane3 = {{{0, 525}, {19900, 525}}, false};
    const ST_LANE lane3 = {3, leftLane3, rightLane3, NULL, NULL, NULL, false, 2, true};
    const ST_LANE laneNet[MAXL] = {lane1, lane2, lane3};
    const ST_RECTANGLE staticObs[MAXSO] = {{{3000, 350}, 200, 450, 2}};
    
    bool is_inlane = check_inlane(laneNet, staticObs[0]);
    if (is_inlane)
        printf("The vehicle is in the lane.\n");
    else
        printf("The vehicle is not in the lane.\n");
    
    return 0;
}



