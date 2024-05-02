#ifndef MOVEMENT_H
#define MOVEMENT_H

#define MOVEOFFSET 0
#define TURNOFFSET 14 // - for undershoot + for overshoot degrees off of correct turn

#define TWISTOFFSET -0 //positive is clockwise driving forward


#include "open_interface.h"
#include "button.h"
#include "uart-interrupt.h"
#include "Timer.h"
#include "structs.h"
#include "scan.h"
#include <math.h>

#define DEGREES_TO_RADS M_PI / 180.0
#define ROBOT_WIDTH 340
#define AVOID_DISTANCE 50.0
#define GODZILLA_RAM_DISTANCE (AVOID_DISTANCE*2.0) + ROBOT_WIDTH // can be adjusted
#define RESCAN_DIST 750 // mm that the robot should travel before rescanning


#define TILE_WIDTH 590 // mm
#define FIELD_WIDTH ((4 * TILE_WIDTH) - START_X) // 4
#define FIELD_LENGTH ((7 * TILE_WIDTH) - START_Y) // 7
//changed to be the maximum bounds that the robot can have

#define START_X ROBOT_WIDTH/2.0
#define START_Y ROBOT_WIDTH/2.0

#define BUMP_OBJECT_WIDTH 120
#define CLIFF_OBJECT_SIZE TILE_WIDTH

#define BOUNDRY_OVERSHOOT 30
char toPutty[55];

//float moveCalibrate(oi_t *sensor_data);
//float turnCalibrate(oi_t *sensor_data);

float move_forward(oi_t *sensor_data, object **obs, int *numObs, float distance_mm, int dir,int depth);
int move_to_point(oi_t *sensor_data, object **obs, int *numObs, int numAttempts, float global_x, float global_y, int dir); // dir: -1 for left, +1 for right
int checkObstacles(oi_t *sensor_data, object **obs, int *numObs, int numAttempts, float global_x, float global_y, int dir);
int checkPerpendicularPoint(object *obs, int numObs, float global_x, float global_y);
coords calculatePerpendicularPoint(object *obs, int obsCount, object targetCoords, int dir);
int compareDistances(const void *a, const void *b);
float calcDistToRobot(object *obs);
float calcDistToPath(object *obs, float global_x, float global_y);
float move_backward(oi_t *sensor_data, float distance_mm);

void set_heading(oi_t *sensor,float degrees);
float turn_right(oi_t *sensor, float degrees);
float turn_left(oi_t *sensor, float degrees);

float ram(oi_t *sensor_data);
float calcDistToPathGodzilla(object *obs, object *godzilla, coords target, int *numObs);
coords get_target_for_godzilla(object *obs, object *godzilla, int *numObs);
float move_to_godzilla(oi_t *sensor_data, object *obs, int *numObs, object *godzilla, int dir); 

int cliff_detected(oi_t *sensor_data, object **obs, int *numObs, int dir,int depth);

#endif
