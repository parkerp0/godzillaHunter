#ifndef MOVEMENT_H
#define MOVEMENT_H

#define TURNOFFSET 10

double move_forward(oi_t *sensor_data, double distance_mm);
double move_backward(oi_t *sensor_data, double distance_mm);
double turn_right(oi_t *sensor, double degrees);
double turn_left(oi_t *sensor, double degrees);

#endif
