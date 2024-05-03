#include "imu.h"

void imu_init() {
     i2c_init();
     // Enable clock to port B
     SYSCTL_RCGCGPIO_R |= 0x2;
     while (!(SYSCTL_PRGPIO_R) & 0x2);

     GPIO_PORTB_DEN_R |= 0xC0;
     GPIO_PORTB_AFSEL_R &= ~0xC0;
     GPIO_PORTB_DIR_R |= 0xC0;

     // Set PB6 to HIGH
     GPIO_PORTB_DATA_R |= 0x40;

     //address A = 0x28 B = 0x29
     //if addr B switch to below line
     // GPIO_PORTB_DATA_R &= ~0x80;
     GPIO_PORTB_DATA_R |= 0x80;

     imu_writeReg(IMU_OPR_MODE, 12);//set to NDOF mode
//     imu_reset();
}

float imu_getHeading() {
    float heading;

            uint8_t* euler = imu_readRegBytes(IMU_EUL_HEAD_LSB,2);
            int16_t eulerHead = euler[0] + (euler[1] << 8);
            float Eheading = fmod((eulerHead/16.) + 360,360);
            lcd_printf("EHeading: %f",Eheading);
            

     return Eheading;
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

void imu_setDefaultUnits() {
     uint8_t mode = imu_readRegByte(IMU_OPR_MODE);
     imu_writeReg(IMU_OPR_MODE, 0x0);//set to config mode
     uint8_t units = 0b00010000;
     imu_writeReg(IMU_UNIT_SEL, units);
     imu_writeReg(IMU_OPR_MODE, mode);//switch config out of mode
}

void imu_reset() {
    GPIO_PORTB_DATA_R &= ~0x40;
    asm(" NOP");
    asm(" NOP");
    GPIO_PORTB_DATA_R |= 0x40;
}
