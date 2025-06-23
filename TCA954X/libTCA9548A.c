/*
  This is an Arduino library written for the TCA9548A/PCA9548A 8-bit multiplexer.
  By Nathan Seidle @ SparkFun Electronics, May 16th, 2020
  Converted to C by Dylan Taft, add HAL, 2025-06-21

  The TCA9548A/PCA9548A allows for up to 8 devices to be attached to a single
  I2C bus. This is helpful for I2C devices that have a single I2C address.

  https://github.com/sparkfun/SparkFun_I2C_Mux_Arduino_Library

  SparkFun labored with love to create this code. Feel like supporting open
  source? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14685
*/

#include "libTCA9548A.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include "I2Chaldev.h"
uint8_t _deviceAddress = QWIIC_MUX_DEFAULT_ADDRESS; //Default unshifted 7-bit address
//Sets up the Mux for basic function
//Returns true if device responded correctly. All ports will be disabled.

struct i2c_haldev_t _hal = {0};
bool QWIICMUX_begin(struct i2c_haldev_t hal)
{
  //Get user's options
  _hal = hal;
  assert(_hal.i2c_start != NULL);
  assert(_hal.i2c_end != NULL);
  assert(_hal.i2c_writeBytes != NULL); 
  assert(_hal.i2c_readBytes != NULL);
  assert(_hal.i2c_readBytes2 != NULL);
  _deviceAddress = QWIIC_MUX_DEFAULT_ADDRESS;

  //Valid addresses for the TCA9548 are 0x70 to 0x77.
  //We don't check the validity of the address. There may be other I2C mux's outside of this range.

  //Check if the device ack's over I2C
  if (QWIICMUX_isConnected() == false)
    return (false);
  return (true);
}

//Returns true if device is present
//Tests for device ack to I2C address
//Then tests if device behaves as we expect
//Leaves with all ports disabled
bool QWIICMUX_isConnected()
{
  _hal.i2c_start(_deviceAddress); //Begin I2C transmission to the device
  int ack = _hal.i2c_writeBytes(NULL, 0); //Write nothing to the device, just check for ACK
  _hal.i2c_end(); //End I2C transmission
  if (ack != 0)
    return (false); //Device did not ACK

  //Write to device, expect a return

  //These next tweo commented lines are unnecessary and actually wrong in the sparkfun implementation, 
  //if you're using this for TC95486a the first for bits will be 0000
  //Furthermore both the 8A and 86 device datasheets do not say you should do a continuous read after write
  //the device has no registers, and the datasheet examples actually show stop condition after both read and write

  //QWIICMUX_setPortState(0xA4); //Set port register to a known value
  //uint8_t response = QWIICMUX_getPortState();

  return(QWIICMUX_setPortState(0x00));   //Disable all ports
}

//Enables one port. Disables all others.
//If port number if out of range, disable all ports
bool QWIICMUX_setPort(uint8_t portNumber)
{
  uint8_t portValue = 0;

  if (portNumber > 7)
    portValue = 0; //If port number is out of range, turn off all ports
  else
    portValue = 1 << portNumber;

  _hal.i2c_start(_deviceAddress); //Begin I2C transmission to the device
  int ack = _hal.i2c_writeBytes(&portValue, 1); //Write the port value to the mux
  _hal.i2c_end(); //End I2C transmission
  if (ack != 0)
    return (false); //Device did not ACK
  return (true);
}

//Returns the first port number bit that is set
//Returns 255 if no port is enabled
//Return 254 if there is an I2C error
uint8_t QWIICMUX_getPort()
{
  //Read the current mux settings
  //_i2cPort->beginTransmission(_deviceAddress); <- Don't do this!
   _hal.i2c_start(_deviceAddress);
   uint8_t portBits;
  int ack = _hal.i2c_readBytes(&portBits, 1); //Request 1 byte from the device
  _hal.i2c_end(); //End I2C transmission
  if (!ack)
    return (254); //Error

  //Search for the first set bit, then return its location
  for (uint8_t x = 0; x < 8; x++)
  {
    if (portBits & (1 << x))
      return (x);
  }
  return (255); //Return no port set
}

//Writes a 8-bit value to mux
//Overwrites any other bits
//This allows us to enable/disable multiple ports at same time
bool QWIICMUX_setPortState(uint8_t portBits)
{
 _hal.i2c_start(_deviceAddress); //Begin I2C transmission to the device
  int ack = _hal.i2c_writeBytes(&portBits, 1); //Write the port value to the mux
  _hal.i2c_end(); //End I2C transmission
  if (ack != 0)
    return (false); //Device did not ACK
  return (true);
}

//Gets the current port state
//Returns byte that may have multiple bits set
uint8_t QWIICMUX_getPortState()
{
  //Read the current mux settings
  //_i2cPort->beginTransmission(_deviceAddress); <- Don't do this!
  _hal.i2c_start(_deviceAddress);
  uint8_t data;
  int ack = _hal.i2c_readBytes(&data, 1); //Request 1 byte from the device
  _hal.i2c_end(); //End I2C transmission
  return data;
}

//Enables a specific port number
//This allows for multiple ports to be 'turned on' at the same time. Use with caution.
bool QWIICMUX_enablePort(uint8_t portNumber)
{
  if (portNumber > 7)
    portNumber = 7; //Error check

  //Read the current mux settings
  uint8_t settings = QWIICMUX_getPortState();

  //Set the wanted bit to enable the port
  settings |= (1 << portNumber);

  return (QWIICMUX_setPortState(settings));
}

//Disables a specific port number
bool QWIICMUX_disablePort(uint8_t portNumber)
{
  if (portNumber > 7)
    portNumber = 7; //Error check

  uint8_t settings = QWIICMUX_getPortState();

  //Clear the wanted bit to disable the port
  settings &= ~(1 << portNumber);

  return (QWIICMUX_setPortState(settings));
}
