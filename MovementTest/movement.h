#ifndef MOVEMENT_H
#define MOVEMENT_H

#define MOVEOFFSET 0
#define TURNOFFSET 10 // - for undershoot + for overshoot degrees off of correct turn

#define TWISTOFFSET -1 //positive is clockwise driving forward


#include "open_interface.h"
#include "button.h"
#include "Timer.h"
#include <math.h>

#define DEGREES_TO_RADS M_PI / 180.0

typedef struct {
    double x;
    double y;
    double heading;
} coords;

double moveCalibrate(oi_t *sensor_data, coords *robotCoords);
double turnCalibrate(oi_t *sensor_data, coords *robotCoords);

double move_forward(oi_t *sensor_data, coords *robotCoords, double distance_mm);
double move_to_point(oi_t *sensor_data, coords *robotCoords, double global_x, double global_y);
double move_backward(oi_t *sensor_data, coords *robotCoords, double distance_mm);
double turn_right(oi_t *sensor, coords *robotCoords, double degrees);
double turn_left(oi_t *sensor, coords *robotCoords, double degrees);

#endif
