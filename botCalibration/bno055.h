#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include <stdbool.h>

#define AXIS_MAP_CONFIG

// Define front of cybot as +y, with +x to the right
#define AXIS_MAP_AXES_DEFAULT 0x21
#define AXIS_MAP_SIGN_DEFAULT 0x04

// use m/s^2, and either degrees or radians. Farenheit is selected for both.
// Upper bit is which direction is positive rotation.
// Both of these are set so turning left is negative, and right is positive (Windows format).
#define UNIT_SEL_DEGREES 0x00
#define UNIT_SEL_RADIANS 0x06

// Max frq 400kHz
// ADDR 0x28 when ADDR low, 0x29 when high


typedef struct {
  short x;
  short y;
  short z;
} bno_xyz_t;

typedef struct {
  short heading;
  short roll;
  short pitch;
} bno_euler_t;

typedef struct {
  short w;
  short x;
  short y;
  short z;
} bno_quat_t;

typedef struct {
  bno_xyz_t accel;
  bno_xyz_t mag;
  bno_xyz_t gyro;
  bno_euler_t euler;
  bno_quat_t quat;
  bno_xyz_t linacl;
  bno_xyz_t grav;
  char temp;
} bno_t;

typedef struct {
  bno_xyz_t accel;
  bno_xyz_t mag;
  bno_xyz_t gyro;
  short accel_r;
  short mag_r;
} bno_calib_t;

void bno_init();
void bno_initCalib(bno_calib_t * bno);
void bno_readCalibration(bno_calib_t * bno);
void bno_writeCalibration(bno_calib_t * bno);
bool bno_calibrate();
void bno_calibrateInteractive();
void bno_stopCalibration();
void bno_update(bno_t * bno);
void bno_free(bno_t * bno);
bno_t * bno_alloc();


#endif
