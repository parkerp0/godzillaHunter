#ifndef MOVEMENT_H
#define MOVEMENT_H

#define MOVEOFFSET 0
#define TURNOFFSET 11 // - for undershoot + for overshoot degrees off of correct turn

#define TWISTOFFSET -1 //positive is clockwise driving forward


#include "open_interface.h"
#include "button.h"
#include "uart-interrupt.h"
#include "Timer.h"
#include "structs.h"
#include <math.h>

#define DEGREES_TO_RADS M_PI / 180.0
#define ROBOT_WIDTH 340
#define AVOID_DISTANCE 50.0
#define GODZILLA_RAM_DISTANCE (AVOID_DISTANCE*2.0) + ROBOT_WIDTH // can be adjusted


#define TILE_WIDTH 590 // mm
#define FIELD_WIDTH 2 * TILE_WIDTH
#define FIELD_LENGTH 3 * TILE_WIDTH

#define START_X ROBOT_WIDTH/2.0
#define START_Y ROBOT_WIDTH/2.0

#define BUMP_OBJECT_WIDTH 120
char toPutty[55];

//float moveCalibrate(oi_t *sensor_data);
//float turnCalibrate(oi_t *sensor_data);

float move_forward(oi_t *sensor_data, object **obs, int *numObs, float distance_mm, int dir);
float move_to_point(oi_t *sensor_data, object **obs, int *numObs, int numAttempts, float global_x, float global_y, int dir); // dir: -1 for left, +1 for right
float checkObstacles(oi_t *sensor_data, object **obs, int *numObs, int numAttempts, float global_x, float global_y, int dir);
coords calculatePerpendicularPoint(object targetCoords, int dir);
int compareDistances(const void *a, const void *b);
float calcDistToRobot(object *obs);
float calcDistToPath(object *obs, float global_x, float global_y);
float move_backward(oi_t *sensor_data, float distance_mm);
float turn_right(oi_t *sensor, float degrees);
float turn_left(oi_t *sensor, float degrees);
float ram(oi_t *sensor_data);
float calcDistToPathGodzilla(object *obs, object *godzilla, coords target, int *numObs);
coords get_target_for_godzilla(object *obs, object *godzilla, int *numObs);
float move_to_godzilla(oi_t *sensor_data, object *obs, int *numObs, object *godzilla, int dir); // global x and y should be godzilla's

void manuever(oi_t *sensor_data, float distance_mm);
int cliff_detected(oi_t *sensor_data, object **obs, int *numObs, int dir);

#endif
