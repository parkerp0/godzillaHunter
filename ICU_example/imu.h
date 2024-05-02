/**
 * @file imu.h
 *
 * Driver for BNO055 IMU.
 *
 * @author Braedon Giblin <bgiblin@iastate.edu>
 */

#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <inc/tm4c123gh6pm.h>


/** BNO055 Address A **/
#define BNO055_ADDRESS_A (0x28)
/** BNO055 Address B **/
#define BNO055_ADDRESS_B (0x29)
/** BNO055 ID **/
#define BNO055_ID (0xA0)

// OPR_MODE 1100b

// Reset int y setting RST_INT in SYS_TRIGGER

// Start Page 0 Register defines
#define IMU_CHIP_ID         0x00
#define IMU_ACC_ID          0x01
#define IMU_MAG_ID          0x02
#define IMU_GYR_ID_R        0x03
#define IMU_SW_REV_MSB      0x04
#define IMU_SW_REV_LSB      0x05
#define IMU_BL_VER_R        0x06
#define IMU_PAGE_ID         0x07
#define IMU_ACC_DATAX_LSB   0x08
#define IMU_ACC_DATAX_MSB   0x09
#define IMU_ACC_DATAY_LSB   0x0A
#define IMU_ACC_DATAY_MSB   0x0B
#define IMU_ACC_DATAZ_LSB   0x0C
#define IMU_ACC_DATAZ_MSB   0x0D
#define IMU_MAG_DATAX_LSB   0x0E
#define IMU_MAG_DATAX_MSB   0x0F
#define IMU_MAG_DATAY_LSB   0x10
#define IMU_MAG_DATAY_MSB   0x11
#define IMU_MAG_DATAZ_LSB   0x12
#define IMU_MAG_DATAZ_MSB   0x13
#define IMU_GYR_DATAX_LSB   0x14
#define IMU_GYR_DATAX_MSB   0x15
#define IMU_GYR_DATAY_LSB   0x16
#define IMU_GYR_DATAY_MSB   0x17
#define IMU_GYR_DATAZ_LSB   0x18
#define IMU_GYR_DATAZ_MSB   0x19
#define IMU_EUL_HEAD_LSB    0x1A
#define IMU_EUL_HEAD_MSB    0x1B
#define IMU_EUL_ROLL_LSB    0x1C
#define IMU_EUL_ROLL_MSB    0x1D
#define IMU_EUL_PTCH_LSB    0x1E
#define IMU_EUL_PTCH_MSB    0x1F
#define IMU_QUA_DATAW_LSB   0x20
#define IMU_QUA_DATAW_MSB   0x21
#define IMU_QUA_DATAX_LSB   0x22
#define IMU_QUA_DATAX_MSB   0x23
#define IMU_QUA_DATAY_LSB   0x24
#define IMU_QUA_DATAY_MSB   0x25
#define IMU_QUA_DATAZ_LSB   0x26
#define IMU_QUA_DATAZ_MSB   0x27
#define IMU_LIA_DATAX_LSB   0x28
#define IMU_LIA_DATAX_MSB   0x29
#define IMU_LIA_DATAY_LSB   0x2A
#define IMU_LIA_DATAY_MSB   0x2B
#define IMU_LIA_DATAZ_LSB   0x2C
#define IMU_LIA_DATAZ_MSB   0x2D
#define IMU_GRV_DATAX_LSB   0x2E
#define IMU_GRV_DATAX_MSB   0x2F
#define IMU_GRV_DATAY_LSB   0x30
#define IMU_GRV_DATAY_MSB   0x31
#define IMU_GRV_DATAZ_LSB   0x32
#define IMU_GRV_DATAZ_MSB   0x33
#define IMU_TEMP            0x34
#define IMU_CALIB_STAT      0x35
#define IMU_SYS_STATUS      0x39
#define IMU_UNIT_SEL        0x3B
#define IMU_OPR_MODE        0x3D

/**
 * @brief IMU Operating Modes.
 */
typedef enum {
    CONFIG,     /**< Config mode */
    ACCONLY,    /**< Non-fusion Accelerometer only mode */
    MAGONLY,    /**< Non-fusion Magnetometer only mode */
    GYROONLY,   /**< Non-fusion Gyroscope only mode */
    ACCMAG,     /**< Non-fusion Accelerometer and Magnetometer mode */
    ACCGYRO,    /**< Non-fusion Accelerometer and Gyrosocpe mode */
    MAGGYRO,    /**< Non-fusion Gyroscope and Magnetometer mode */
    AMG,        /**< Non-fusion Accelerometer, Gyroscope, and Magnetometer mode */
    IMU,        /**< Fusion Accelerometer and Gyrosocope -- relative*/
    COMPASS,    /**< Fusion Accelerometer and Magnetometer -- absolute*/
    M4G,        /**< Fusion Accelerometer and magnetometer -- relative */
    NDOF_FMC_OFF, /**< Fusion, all devices, no fast calibration */
    NDOF        /**< Fusion, all devices */
} imu_mode_t;

/**
 * @brief Calibration status.
 *
 * a 3 on any of the feilds indicates full calibration of that device.
 */
typedef struct {
    uint8_t mag;    /**< Magnetometer calibration status */
    uint8_t acc;    /**< Accelerometer calibration status */
    uint8_t gyr;    /**< Gyroscope calibration status */
    uint8_t sys;    /**< System calibration status */
} calib_t;

/**
 * @brief Magnetometer data.
 *
 */
typedef struct {
    int16_t x;      /**< Magnetometer X component */
    int16_t y;      /**< Magnetometer Y component */
    int16_t z;      /**< Magnetometer Z component */
    float heading;  /**< Magnetometer heading, given in degrees. 0 Degrees represents north */
} mag_t;

/**
 * @brief Accelerometer data.
 *
 */
typedef struct {
    float x;        /**< Accelerometer X component */
    float y;        /**< Accelerometer Y component */
    float z;        /**< Accelerometer X component */
    float magnitude;/**< Accelerometer vector magnitude */
} acc_t;

/**
 * @brief IMU Info packet.
 */
typedef struct {
    uint8_t chipID;     /**< Chip ID */
    uint8_t accID;      /**< Accelerometer ID */
    uint8_t magID;      /**< Magnetometer ID */
    uint8_t gyrID;      /**< Gyroscope ID */
    uint16_t swRevID;   /**< Software Revision ID */
    uint8_t blRevID;    /**< Bootloader Revision */
} imu_info_t;

/**
 * @brief Initialize IMU.
 */
void imu_init();

/**
 * @brief Check if IMU is connected.
 *
 * @return True if connected, else false
 */
bool imu_connected();

/**
 * @brief Resets the IMU. Poll connected to wait until connected.
 */
void imu_reset();

/**
 * @brief Gets the current temperature reading from IMU.
 *
 * @return Temperature in specified units (default fahrenheit)
 */
int imu_getTemp();

/**
 * @brief Gets the current magnetometer data from the IMU.
 *
 * @return Magnetometer data packet
 */
mag_t* imu_getMag();

/**
 * @brief Gets the current acceleration data from the IMU.
 *
 * @return Accelerometer data packet
 */
acc_t* imu_getAcc();

/**
 * @brief Gets the linear acceleration data from IMU.
 *
 * IMU must be in Fusion mode to read this value.
 *
 * @return linear acceration data.
 */
acc_t* imu_getLinAcc();

/**
 * @brief Gets the current status from the IMU.
 *
 * @return status. See data sheet.
 */
uint8_t imu_getStatus();

/**
 * @brief Writes to a specified register.
 *
 * @param regAddr Register address to write
 * @param val Value to write
 */
void imu_writeReg(uint8_t regAddr, uint8_t val);

/**
 * @brief Reads a byte from the IMU.
 *
 * @param regAddr Address to read from
 *
 * @return Byte read
 */
uint8_t imu_readRegByte(uint8_t regAddr);

/**
 * @brief Reads a multiple bytes from the IMU.
 *
 * **NOTE** This allocates memory. Allocated array **MUST** be freed.
 *
 * @param regAddr Address to begin read from
 * @param dataLen Number of bytes to read
 *
 * @return Byte array of bytes read.
 */
uint8_t* imu_readRegBytes(uint8_t regAddr, size_t dataLen);

/**
 * @brief gets the current Chip Info.
 *
 * @return Chip info packet.
 */
imu_info_t* imu_getChipInfo();

/**
 * @brief Sets the default units.
 *
 * Default units are - M/s^2, degrees, fahrenheit
 */
void imu_setDefaultUnits();

/**
 * @brief sets the mode of the IMU. See imu_mode_t struct for different modes.
 */
void imu_setMode(imu_mode_t mode);


#endif /* IMU_H_ */
