#ifndef MOVEMENT_H
#define MOVEMENT_H

#define MOVEOFFSET 0
#define TURNOFFSET 14 // - for undershoot + for overshoot degrees off of correct tur

#define TWISTOFFSET -7 //positive is clockwise driving forward


#include "open_interface.h"
#include "button.h"
#include "uart-interrupt.h"
#include "Timer.h"
#include <math.h>

#define DEGREES_TO_RADS M_PI / 180.0
#define ROBOT_WIDTH 304.8 // I guessed this
#define AVOID_DISTANCE 200

#define TILE_WIDTH 20 * 2.54 // my guess
#define FIELD_WIDTH 4 * TILEWIDTH
#define FIELD_LENGTH 7 * TILEWIDTH

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


char toPutty[55];

float moveCalibrate(oi_t *sensor_data);
float turnCalibrate(oi_t *sensor_data);

float move_forward(oi_t *sensor_data, float distance_mm);
float move_to_point(oi_t *sensor_data, object *obs, int numObs, int numAttempts, float global_x, float global_y);
float checkObstacles(oi_t *sensor_data, object *obs, int numObs, int numAttempts, float global_x, float global_y);
coords calculatePerpendicularPoint(object targetCoords);
int compareDistances(const void *a, const void *b);
float calcDistToRobot(object *obs);
float calcDistToPath(object *obs, float global_x, float global_y);
float move_backward(oi_t *sensor_data, float distance_mm);
float turn_right(oi_t *sensor, float degrees);
float turn_left(oi_t *sensor, float degrees);
float ram(oi_t *sensor);

void manuever(oi_t *sensor_data, float distance_mm);
bool cliff_detected(oi_t *sensor_data);

#endif
