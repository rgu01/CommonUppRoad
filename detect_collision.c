#include <stdio.h>
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



typedef struct {
    int32_t x;
    int32_t y;
}ST_DPOINT;

typedef struct {
    double x;
    double y;
}ST_CPOINT;

typedef struct {
    ST_CPOINT position;
    double velocity;
    double orientation;
    double acceleration;
    double yawRate;
}ST_CSTATE;

typedef struct {
    ST_DPOINT center;
    int16_t width;
    int16_t length;
    int16_t orientation;
}ST_RECTANGLE;

typedef struct {
    double length;
    double width;
} VehicleDimensions;


double i2d(const int i) {
    const double SCALE = 100.0; // scale from int to double: 100 = 1.0
    double v = i/SCALE;
    return v;
}

int d2i(const double i) {
    const double SCALE = 100.0; // scale from int to double: 100 = 1.0
    int v = (int)(i*SCALE); // TODO: I update the function here, remember to change to fint during integration 
    return v;
}

double compute_approximating_circle_radius(double ego_length, double ego_width) {
    if (ego_length <= 0 || ego_width <= 0) {
        printf("Invalid vehicle dimensions\n");
        return -1.0;
    }

    if (fabs(ego_length) < 1e-6 && fabs(ego_width) < 1e-6)
        return 0.0;
    // Divide rectangle into 3 smaller rectangles
    double square_length = ego_length/3.0;
    // Calculate minimum radius
    double diagonal_square = sqrt(pow(square_length/2.0, 2) + pow(ego_width/2.0, 2));
    // Round up value
    double approx_radius = round(diagonal_square*10.0) / 10.0 + 0.1;

    return approx_radius;
}

void compute_centers_of_approximation_circles(ST_RECTANGLE veh_rect, ST_CPOINT *centers) {
    double disc_radius = compute_approximating_circle_radius((double)veh_rect.length, (double)veh_rect.width);
    double distance_centers = disc_radius/2.0;

    // Compute the center position of first circle (front)
    double veh_cent_x_double = i2d(veh_rect.center.x);
    double veh_cent_y_double = i2d(veh_rect.center.y);
    centers[0].x = veh_cent_x_double + (distance_centers/2.0)*cos(i2d(veh_rect.orientation));
    centers[0].y = veh_cent_y_double + (distance_centers/2.0)*sin(i2d(veh_rect.orientation));

    // Compute the center position of second circle (rear)
    centers[1].x = veh_cent_x_double - (distance_centers/2.0)*cos(i2d(veh_rect.orientation));
    centers[1].y = veh_cent_y_double - (distance_centers/2.0)*sin(i2d(veh_rect.orientation));
}


int check_collision(ST_RECTANGLE veh_st_rect1, ST_RECTANGLE veh_st_rect2, int dis_thres) {

    // define two vehicles' circle tuple (front and rear)
    ST_CPOINT veh_circle_tuple1[2];
    ST_CPOINT veh_circle_tuple2[2];
    
    // calculate the centers of the vehicles' circles
    compute_centers_of_approximation_circles(veh_st_rect1, veh_circle_tuple1);
    compute_centers_of_approximation_circles(veh_st_rect2, veh_circle_tuple2);

    double min_dis = INFINITY;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            double dis = sqrt(pow(veh_circle_tuple1[i].x - veh_circle_tuple2[j].x, 2) + pow(veh_circle_tuple1[i].y - veh_circle_tuple2[j].y, 2));
            min_dis = fmin(min_dis, dis);
        }
    }
    int min_dis_int = d2i(min_dis);
    if (dis_thres > min_dis_int)
        return 1;
    else
        return 0;
}

int main() {
    const int DIS_THRES = 10000;
    double state1[] = {0.0, 0.0, 5.0, 2.0, M_PI/4.0};  
    double state2[] = {5.0, 2.0, 5.0, 2.0, 3.0*M_PI/4.0};

    ST_CPOINT veh_circle_tuple1[2];
    ST_CPOINT veh_circle_tuple2[2];

    ST_CPOINT position1_double[2] = {state1[0], state1[1]};
    ST_CPOINT position2_double[2] = {state2[0], state2[1]};

    int32_t position1_x = d2i(state1[0]);
    int32_t position1_y = d2i(state1[1]);
    int32_t position2_x = d2i(state2[0]);
    int32_t position2_y = d2i(state2[1]);

    ST_DPOINT position1_int16t = {position1_x, position1_y};
    ST_DPOINT position2_int16t = {position2_x, position2_y};

    ST_RECTANGLE veh_st_rect1 = {position1_int16t, (int16_t)state1[3], (int16_t)state1[2], (int16_t)d2i(state1[4])};
    ST_RECTANGLE veh_st_rect2 = {position2_int16t, (int16_t)state2[3], (int16_t)state2[2], (int16_t)d2i(state2[4])};
    
    int is_collsion = check_collision(veh_st_rect1, veh_st_rect2, DIS_THRES);
    if(is_collsion == 1)
        printf("collision detected");
    
    return 0;
}
