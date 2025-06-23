/*
  This is an Arduino library written for the TCA9548A/PCA9548A 8-bit multiplexer.
  By Nathan Seidle @ SparkFun Electronics, May 16th, 2020
  Converted to C by Dylan Taft, 2025-06-21

  The TCA9548A/PCA9548A allows for up to 8 devices to be attached to a single
  I2C bus. This is helpful for I2C devices that have a single I2C address.

  https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14685
*/

#ifndef TCA954X_H
#define TCA954X_H
#include <stdint.h>
#include <stdbool.h>
#include "I2Chaldev.h" // Include the HAL definition for TCA9548A
#define QWIIC_MUX_DEFAULT_ADDRESS 0x70


bool QWIICMUX_begin(struct i2c_haldev_t hal); //Check communication and initialize device
bool QWIICMUX_isConnected();                                                                      //Returns true if device acks at the I2C address
bool QWIICMUX_setPort(uint8_t portNumber);                                                        //Enable a single port. All other ports disabled.
bool QWIICMUX_setPortState(uint8_t portBits);                                                     //Overwrite port register with all 8 bits. Allows multiple bit writing in one call.
uint8_t QWIICMUX_getPort();                                                                       //Returns the bit position of the first enabled port. Useful for IDing which port number is enabled.
uint8_t QWIICMUX_getPortState();                                                                  //Returns current 8-bit wide state. May have multiple bits set in 8-bit field.
bool QWIICMUX_enablePort(uint8_t portNumber);                                                     //Enable a single port without affecting other bits
bool QWIICMUX_disablePort(uint8_t portNumber);                                                    //Disable a single port without affecting other bits


#endif
