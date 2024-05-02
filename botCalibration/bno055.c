#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "bno055.h"
#include "lcd.h"
#include "uart-interrupt.h"

// Use fast mode --> TIMER_PRD = 1 from datasheet calculations
// uses I2C1

/** BNO CONFIG CONSTANTS -- See Datasheet **/
#define BNO_ADDR (0x29<<1)
#define BNO_MODE_NDOF 0x0C
#define BNO_MODE_CONFIG 0x00

#define BNO_PWR_NORMAL 0x00
#define BNO_MODE_R 0x3D
#define BNO_PWR_R 0x3E
#define BNO_AXIS_REMAP_R 0x41
#define BNO_AXIS_REMAP 0x21
#define BNO_AXIS_SIGN_R 0x42
#define BNO_AXIS_SIGN 0x04
#define BNO_CALIB_STAT_R 0x35

/**
 * Send the register to the BNO over I2C1
 * 1 Byte sent: REG
 */
void bno_sendReg(char reg)
{
    while (I2C1_MCS_R & I2C_MCS_BUSY)
        ;
    I2C1_MSA_R = BNO_ADDR | 0x0;
    I2C1_MDR_R = reg;
    I2C1_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
}

/**
 * Write a single register value on the BNO over I2C1
 * 2 Bytes sent: REG, DATA
 */
void bno_write(char reg, uint8_t value)
{
    bno_sendReg(reg);
    while (I2C1_MCS_R & I2C_MCS_BUSY)
        ;
    I2C1_MDR_R = value;
    I2C1_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;
}

/**
 * Reads a single register value on the BNO over I2C1
 * 1 Byte sent: REG
 * 1 Byte received: DATA
 */
char bno_read(char reg)
{
    bno_sendReg(reg);
    while (I2C1_MCS_R & I2C_MCS_BUSY)
        ;

    I2C1_MSA_R = BNO_ADDR | 0x1;
    I2C1_MCS_R = I2C_MCS_STOP | I2C_MCS_START | I2C_MCS_RUN;
    while (I2C1_MCS_R & I2C_MCS_BUSY)
        ;

    return I2C1_MDR_R;
}

/**
 * Read multiple registers in series (2 or more!). Assumes that you have already
 * sent the register number at some point in the past.
 *
 * Ex:
 * bno_sendReg(0x55);
 * bno_readNum(30, data);
 * bno_readNum(30, data);
 * ...
 * As per the datasheet, each call to readNum will start at register 0x55, the last register value sent.
 */
void bno_readNum(char num, char *data)
{
    while (I2C1_MCS_R & I2C_MCS_BUSY)
        ;

    I2C1_MSA_R = BNO_ADDR | 0x1;
    I2C1_MCS_R = I2C_MCS_START | I2C_MCS_ACK | I2C_MCS_RUN;

    for (; num > 2; num--)
    {
        while (I2C1_MCS_R & I2C_MCS_BUSY)
            ;
        *data++ = I2C1_MDR_R;
        I2C1_MCS_R = I2C_MCS_ACK | I2C_MCS_RUN;
    }
    while (I2C1_MCS_R & I2C_MCS_BUSY)
        ;
    *data++ = I2C1_MDR_R;
    I2C1_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;
    while (I2C1_MCS_R & I2C_MCS_BUSY)
        ;
    *data++ = I2C1_MDR_R;
}

/**
 * Initialize the BNO sensor with calibration parameters.
 * The BNO does a good job of self-calibrating, so only use this if you need to.
 * A ptr with an address of 0 (nullptr) will be assumed to mean there is no calibration data to set.
 */
void bno_initCalib(bno_calib_t *bno_c)
{

    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R1;   // Enable I2C1
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R1; // Enable PORTA and B
    timer_waitMillis(400); // 400ms off to config mode on POR

    // Enable I2C1 pins (PA6 and PA7)
    GPIO_PORTA_DEN_R |= 0xC0; // Digital Enable
    GPIO_PORTA_AFSEL_R |= 0xC0; // PA6 + PA7
    GPIO_PORTA_ODR_R |= 0x80; // SDA (PA7) Open Drain
    GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R & 0x00FFFFFF) | 0x33000000; // Configure PA6 + PA7 for I2C1

    // Enable the IMU (make reset pin HIGH)
    GPIO_PORTB_DEN_R |= 0xC0; // Digital Enable
    GPIO_PORTB_AFSEL_R &= ~0xC0; // PB6 (RSTn) and PB7 (ADDR)
    GPIO_PORTB_DIR_R |= 0xC0;
    GPIO_PORTB_DATA_R |= 0xC0;

    // Configure I2C1 parameters
    I2C1_MCR_R = I2C_MCR_MFE; // Set Master mode
    I2C1_MTPR_R = 1;   // Set SCL speed. See datasheet for calc.
                       // Assumes 16MHz sysclk and 400kHz SPI clk

    // Set the BNO's power state to NORMAL and put it in CONFIG mode.
    bno_write(BNO_PWR_R, BNO_PWR_NORMAL);
    bno_write(BNO_MODE_R, BNO_MODE_CONFIG);

    // Write the calibration data if any was set.
    if (bno_c != 0)
    {
        bno_writeCalibration(bno_c);
    }

    // Make sure the axes mappings are correct. The drawings on the baseboard are the
    // defaults. Makes more sense to have +Y be forwards, +X be to the right, and +Z to be up.
    // This also makes it so heading accurately provides the heading of the robot.
    bno_write(BNO_AXIS_REMAP_R, BNO_AXIS_REMAP);
    bno_write(BNO_AXIS_SIGN_R, BNO_AXIS_SIGN);

    // Set the mode to NDOF mode (full integration with all sensors).
    bno_write(BNO_MODE_R, BNO_MODE_NDOF);
}

/**
 * Initialize the BNO sensor without calibration parameters
 */
void bno_init()
{
    bno_initCalib((bno_calib_t*) 0);
}

/**
 * Allocate a new BNO sensor data struct.
 * The BNO_t struct contains all the sensor data you'll ever need.
 * One additional feature to add might be to configure which parts
 * of the sensor data are really necessary.
 */
bno_t* bno_alloc()
{
    bno_t *bno = malloc(sizeof(bno_t));
    memset(bno, 0, sizeof(bno_t));
    return bno;
}

/**
 * Free the BNO instance
 */
void bno_free(bno_t *bno)
{
    free(bno);
}

/**
 * Update the BNO data
 */
void bno_update(bno_t *bno)
{
    bno_sendReg(0x08);
    bno_readNum(sizeof(bno_t), (void*) bno);
}

// Flag to stop calibration if set.
volatile bool bno_stopCalibrationFlag = false;

// Tell the calibration routine to stop.
void bno_stopCalibration()
{
    bno_stopCalibrationFlag = true;
}

// Status messages for interactive calibration.
const char *statuses[] = { "Keep Stable", "6 Stable Positions",
                           "Random Movement", "Complete!" };

/**
 * Calibrate the BNO IMU using the LCD screen for output. Call bno_calibrateInteractive() instead and connect
 * over putty for easier use.
 */
bool bno_calibrate()
{
    bno_stopCalibrationFlag = false;
    uint8_t status;
    do
    {
        // Check the calibration status bits and display prompts on-screen
        status = (uint8_t) bno_read(BNO_CALIB_STAT_R);
        uint8_t system = (status >> 6) & 0x3;
        uint8_t gyro = (status >> 4) & 0x3;
        uint8_t accel = (status >> 2) & 0x3;
        uint8_t mag = (status) & 0x3;
        lcd_printf(
                "Calibrating: OK = 3\n%s\nSystem: %d, Gyro: %d\nAccel: %d, Mag: %d",
                gyro == 0x3 ?
                        (accel == 0x3 ?
                                (mag == 0x3 ? statuses[3] : statuses[2]) :
                                statuses[1]) :
                        statuses[0],
                system, gyro, accel, mag);
    }
    while (status != 0xFF && !bno_stopCalibrationFlag);
    return status == 0xFF;
}

/**
 * Read the current calibration data into a calibration pointer.
 * This is the same pointer you should pass to bno_initCalib.
 */
void bno_readCalibration(bno_calib_t *bno)
{
    bno_write(BNO_MODE_R, BNO_MODE_CONFIG);
    bno_sendReg(0x55);
    bno_readNum(sizeof(bno_calib_t), (void*) bno);
    bno_write(BNO_MODE_R, BNO_MODE_NDOF);
}

/**
 * Write calibration data. Internal function, do not call unless
 * you are in CONFIG mode.
 */
void bno_writeCalibration(bno_calib_t *bno)
{
    uint8_t *data = (void*) bno;
    char reg = 0x55;
    int i;
    for (i = 0; i < sizeof(bno_calib_t); i++)
    {
        bno_write(reg++, *data++);
    }
}

/**
 * Interactive calibration over PuTTY. Run this to generate the BNO_CALIB_t struct data, then pass the struct to init_calib
 * to reuse the calibration results.
 * Requires LCD, BNO, and UART to be initialized
 * and lcd_printf, uart_sendStr, and uart_waitForByte methods to exist.
 *
 */
void bno_calibrateInteractive()
{
    // Buffer string for printing to uart
    char bno_str[120];
    bno_calib_t *bno_c = malloc(sizeof(bno_calib_t));
    uart_sendStr("Press any key to continue...\r\n");
    while(command_byte != 's');
    bno_readCalibration(bno_c);
    uart_sendStr("Current Calibration Data:\r\n");
    sprintf(bno_str,
            "Accel x: %04hX, y: %04hX, z: %04hX, r: %04hX\r\nMag x: %04hX, y: %04hX, z: %04hX, r: %04hX\r\nGyro x: %04hX, y: %04hX, z: %04hX\r\n",
            bno_c->accel.x, bno_c->accel.y, bno_c->accel.z, bno_c->accel_r,
            bno_c->mag.x, bno_c->mag.y, bno_c->mag.z, bno_c->mag_r,
            bno_c->gyro.x, bno_c->gyro.y, bno_c->gyro.z);
    uart_sendStr(bno_str);
    sprintf(bno_str,
            "bno_calib_t bno_calibration = {0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX};\r\n",
            bno_c->accel.x, bno_c->accel.y, bno_c->accel.z, bno_c->mag.x,
            bno_c->mag.y, bno_c->mag.z, bno_c->gyro.x, bno_c->gyro.y,
            bno_c->gyro.z, bno_c->accel_r, bno_c->mag_r);
    uart_sendStr(bno_str);
    uart_sendStr("Calibrating...\r\n");
    bno_calibrate();
    uart_sendStr("Calibration Data:\r\n");
    bno_readCalibration(bno_c);
    sprintf(bno_str,
            "Accel x: %04hX, y: %04hX, z: %04hX, r: %04hX\r\nMag x: %04hX, y: %04hX, z: %04hX, r: %04hX\r\nGyro x: %04hX, y: %04hX, z: %04hX\r\n",
            bno_c->accel.x, bno_c->accel.y, bno_c->accel.z, bno_c->accel_r,
            bno_c->mag.x, bno_c->mag.y, bno_c->mag.z, bno_c->mag_r,
            bno_c->gyro.x, bno_c->gyro.y, bno_c->gyro.z);
    uart_sendStr(bno_str);
    sprintf(bno_str,
            "bno_calib_t bno_calibration = {0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX,0x%04hX};\r\n",
            bno_c->accel.x, bno_c->accel.y, bno_c->accel.z, bno_c->mag.x,
            bno_c->mag.y, bno_c->mag.z, bno_c->gyro.x, bno_c->gyro.y,
            bno_c->gyro.z, bno_c->accel_r, bno_c->mag_r);
    uart_sendStr(bno_str);
    free(bno_c);
}

