#ifndef I2CHALDEV_H
#define I2CHALDEV_H
/*
A note on HAL
The HAL (Hardware Abstraction Layer) is a set of function pointers that allow the library to interact with the hardware without being tied to a specific implementation. 
This allows for flexibility in how the library can be used, such as with different microcontrollers and their SDKs
Some SDKs may, for example, send the START or STOP condition automatically with their TX or RX functions.  In which case, your HAL implementation should not send those conditions in the i2c_start() and i2c_end() functions
Make sure to implement the HAL functions in a way that matches your specific hardware and SDK requirements.
*/

struct i2c_haldev_t {
    void (*digitalWrite)(uint16_t pin, uint8_t value);
    void (*i2c_start)(uint8_t address); // SDA sent low, SCL HI, buffers address
    void (*i2c_end)(); // SDA sent high, SCL HI, release bus
    int (*i2c_writeBytes)(uint8_t *data, uint8_t length); // address + 0 write bit + data sent, returns ack
    int (*i2c_readBytes)(uint8_t *data, uint8_t length); // address + 1 read bit sent, wait for data, returns ack
    int (*i2c_readBytes2)(uint8_t *out_data, uint8_t out_len, uint8_t *in_data, uint8_t in_len); // no stop between write and read, address + 1 read bit sent, wait for data, returns ack.  Often used for reading registers
    uint32_t (*micros)();
} ;
#endif