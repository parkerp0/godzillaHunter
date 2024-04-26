#ifndef MOVEMENT_H
#define MOVEMENT_H

#define MOVEOFFSET 0
#define TURNOFFSET 14 // - for undershoot + for overshoot degrees off of correct tur

#define TWISTOFFSET -7 //positive is clockwise driving forward


#include "open_interface.h"
#include "button.h"
#include "Timer.h"
#include <math.h>

#define DEGREES_TO_RADS M_PI / 180.0
#define ROBOT_WIDTH 304.8 // I guessed this
#define AVOID_DISTANCE 200

typedef struct {
    float x;
    float y;
    float heading;
} coords;

typedef struct
{
    float x;
    float y;
    float linearWidth;
}object;


char toPutty[50];

float moveCalibrate(oi_t *sensor_data, coords *robotCoords);
float turnCalibrate(oi_t *sensor_data, coords *robotCoords);

float move_forward(oi_t *sensor_data, coords *robotCoords, float distance_mm);
float move_to_point(oi_t *sensor_data, coords *robotCoords, object *obs, float global_x, float global_y);
float checkObstacles(oi_t *sensor_data, coords *robotCoords, object *obs, float global_x, float global_y);
coords calculatePerpendicularPoint(coords *robotCoords, object targetCoords);
int compareDistances(const void *a, const void *b);
float calcDistToRobot(coords *robotCoords, object *obs);
float calcDistToPath(coords *robotCoords, object *obs, float global_x, float global_y);
float move_backward(oi_t *sensor_data, coords *robotCoords, float distance_mm);
float turn_right(oi_t *sensor, coords *robotCoords, float degrees);
float turn_left(oi_t *sensor, coords *robotCoords, float degrees);
float ram(oi_t *sensor, coords *robotCoords);

void manuever(oi_t *sensor_data, coords *robotCoords, float distance_mm);

#endif
