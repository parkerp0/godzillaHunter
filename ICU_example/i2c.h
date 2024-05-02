/**
 * @file i2c.h
 *
 * Library for I2C on TM4C123GH6PM Microcontroller.
 *
 * @author Braedon Giblin <bgiblin@iastate.edu>
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <inc/tm4c123gh6pm.h>

/**
 * @brief Initialize I2C1 module.
 *
 * Initializes I2C1 module as a master at standard speed.
 */
void i2c_init();

/**
 * @brief Sends one byte of data.
 *
 * Automatically sends start and stop bits.
 *
 * @param addr
 *        7 bit address to send data to
 * @param data
 *        Byte of data to transmit
 * @param sendStop
 *        Whether or not to send a stop sequence
 * @return Number of bytes sent (1 if successful, else 0)
 *
 */
size_t i2c_sendByte(uint8_t addr, uint8_t data);

/**
 * @brief Sends multiple bytes of data.
 *
 * Automatically sends start and stop bits.
 *
 * @param addr
 *        7 bit address to send data to
 * @param byte
 *        Byte of data to transmit
 * @param dataLen
 *        Number of data bits to send
 * @return Number of bytes sent (if successful, should match dataLen
 *
 */
size_t i2c_sendBytes(uint8_t addr, uint8_t* data, size_t dataLen);

/**
 * @brief Receives a byte of data from a slave.
 *
 * @param addr
 *        Address to read from
 * @return byte received
 */
uint8_t i2c_recByte(uint8_t addr);

/**
 * @brief Receives multiple bytes of data from a slave.
 *
 * @param addr
 *        Address to read from
 * @param dataLen
 *        number of bytes to read
 * @return bytes received
 */
uint8_t* i2c_recBytes(uint8_t addr, size_t dataLen);

/**
 * @brief Transmits a request, then receives with a repeated START.
 *
 * @param addr
 *        Address to request from
 * @param request
 *        Request byte
 */
uint8_t i2c_requestByte(uint8_t addr, uint8_t request);

#endif /* I2C_H_ */
