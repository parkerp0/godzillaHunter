/**
 * @file imu.c
 *
 * Driver for BNO055 IMU.
 *
 * @author Braedon Giblin <bgiblin@iastate.edu>
 */

#include <stdlib.h>
#include <math.h>
#include "i2c.h"
#include "imu.h"
#include "timer.h"
#include "timer.h"

/******************************************************************************
            Static Function Prototypes
******************************************************************************/
static void addr_set(uint8_t addr);

static float vectorMag(float x, float y, float z);


/**
 * Sets the slave address of IMU
 *
 * @param addr either 0x28 or 0x29
 */
void addr_set(uint8_t addr) {
    switch(addr) {
        case(0x28):
            GPIO_PORTB_DATA_R &= ~0x80;
            break;
        case(0x29): // FALL THROUGH
        default:
            GPIO_PORTB_DATA_R |= 0x80;
    }
}

 void imu_init() {
     i2c_init();
     timer_init();
     // Enable clock to port B
     SYSCTL_RCGCGPIO_R |= 0x2;
     while (!(SYSCTL_PRGPIO_R) & 0x2);

     GPIO_PORTB_DEN_R |= 0xC0;
     GPIO_PORTB_AFSEL_R &= ~0xC0;
     GPIO_PORTB_DIR_R |= 0xC0;

     // Set PB6 to HIGH
     GPIO_PORTB_DATA_R |= 0x40;
     addr_set(BNO055_ADDRESS_B);
//     imu_reset();
 }

static float vectorMag(float x, float y, float z) {
    return sqrtf(powf(x, 2) + powf(y, 2) + powf(z, 2));
}

 void imu_setMode(imu_mode_t mode) {
     imu_writeReg(IMU_OPR_MODE, mode);
 }

 imu_mode_t imu_getMode() {
     return (imu_mode_t) imu_readRegByte(IMU_OPR_MODE);
 }

 mag_t* imu_getMag() {
     mag_t* mag = malloc(sizeof(mag_t));
     uint8_t* magReadings= imu_readRegBytes(IMU_MAG_DATAX_LSB, 6);

     mag->x = magReadings[0] + (magReadings[1] << 8);
     mag->y = magReadings[2] + (magReadings[3] << 8);
     mag->z = magReadings[4] + (magReadings[5] << 8);

     mag->heading = atan2f(mag->y, mag->x) * 180.0f/M_PI;

     return mag;
 }

acc_t* imu_getAcc() {
    acc_t* acc = malloc(sizeof(acc_t));
    uint8_t* accReadings = imu_readRegBytes(IMU_LIA_DATAX_LSB, 6);

    acc->x = (accReadings[0] + (accReadings[1] << 8)) * .01f;
    acc->y = (accReadings[2] + (accReadings[3] << 8)) * .01f;
    acc->z = (accReadings[4] + (accReadings[5] << 8)) * .01f;

    acc->magnitude = vectorMag(acc->x, acc->y, acc->z);

    return acc;
}


acc_t* imu_getLinAcc();

 void imu_setDefaultUnits() {
     imu_mode_t mode = imu_getMode();
     imu_setMode(CONFIG);
     uint8_t units = 0b00010000;
     imu_writeReg(IMU_UNIT_SEL, units);
     imu_setMode(mode);
 }

bool imu_connected() {
    return imu_readRegByte(IMU_CHIP_ID) == (BNO055_ID);
}

uint8_t imu_getStatus() {
    return imu_readRegByte(IMU_SYS_STATUS);
}

int imu_getTemp() {
   return imu_readRegByte(IMU_TEMP);
}

imu_info_t* imu_getChipInfo() {
    imu_info_t* chipInfo = malloc(sizeof(imu_info_t));

//    chipInfo->chipID = imu_readRegByte(IMU_CHIP_ID);
//    chipInfo->accID = imu_readRegByte(IMU_ACC_ID);
//    chipInfo->magID = imu_readRegByte(IMU_MAG_ID);
//    chipInfo->gyrID = imu_readRegByte(IMU_GYR_ID_R);
//    chipInfo->swRevID = imu_readRegByte(IMU_SW_REV_MSB);
//    chipInfo->swRevID += imu_readRegByte(IMU_SW_REV_LSB) << 8;
//    chipInfo->blRevID = imu_readRegByte(IMU_BL_VER_R);

    uint8_t* data = imu_readRegBytes(IMU_CHIP_ID, 7);
    chipInfo->chipID = data[0];
    chipInfo->accID = data[1];
    chipInfo->magID = data[2];
    chipInfo->gyrID = data[3];
    chipInfo->swRevID = data[4];
    chipInfo->swRevID += data[5] << 8;
    chipInfo->blRevID = data[6];

    return chipInfo;
}

void imu_reset() {
    GPIO_PORTB_DATA_R &= ~0x40;
    asm(" NOP");
    asm(" NOP");
    GPIO_PORTB_DATA_R |= 0x40;
}

void imu_writeReg(uint8_t regAddr, uint8_t val){
    uint8_t data[2];
    data[0] = regAddr;
    data[1] = val;
    i2c_sendBytes(BNO055_ADDRESS_B, data, 2);
}

uint8_t imu_readRegByte(uint8_t regAddr){
    i2c_requestByte(BNO055_ADDRESS_B, regAddr);
    return i2c_recByte(BNO055_ADDRESS_B);
}

uint8_t* imu_readRegBytes(uint8_t regAddr, size_t dataLen){
    uint8_t* ret = malloc(sizeof(uint8_t) * dataLen);
    i2c_requestByte(BNO055_ADDRESS_B, regAddr);
    ret = i2c_recBytes(BNO055_ADDRESS_B, dataLen);

    return ret;
}


