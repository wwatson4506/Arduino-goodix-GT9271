/*
 * License: Refer to LICENSE file in this directory.
 * Modified: For use with Teensy 4.x and MicroMod
 *       By: Warren Watson 2024.
*/

#ifndef _GOODIX_H_
#define _GOODIX_H_

#include <Arduino.h>
#include "GoodixStructs.h"
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
#include <i2c_t3.h>
#else // T4.0/T4.1/MicroMod
#include <Wire.h>
//#include <i2c_driver.h>
//#include <i2c_driver_wire.h>
#endif

#define LCD_WIDTH 1024
#define LCD_HEIGHT 600

#define GOODIX_OK   0

// 0x28/0x29 (0x14 7bit)
#define GOODIX_I2C_ADDR_28  0x14
// 0xBA/0xBB (0x5D 7bit)
#define GOODIX_I2C_ADDR_BA  0x5D

#define GOODIX_MAX_HEIGHT   4096
#define GOODIX_MAX_WIDTH    4096
#define GOODIX_INT_TRIGGER    1
#define GOODIX_CONTACT_SIZE   8
#define GOODIX_MAX_CONTACTS   10

#define GOODIX_CONFIG_MAX_LENGTH  240
#define GOODIX_CONFIG_911_LENGTH  186
#define GOODIX_CONFIG_967_LENGTH  228

/* Register defines */
#define GT_REG_CMD  0x8040

#define GT_REG_CFG  0x8047
#define GT_REG_DATA 0x8140


// Write only registers
#define GOODIX_REG_COMMAND        0x8040
#define GOODIX_REG_LED_CONTROL    0x8041
#define GOODIX_REG_PROXIMITY_EN   0x8042

// Read/write registers
// The version number of the configuration file
#define GOODIX_REG_CONFIG_DATA  0x8047
// X output maximum value (LSB 2 bytes)
#define GOODIX_REG_MAX_X        0x8048
// Y output maximum value (LSB 2 bytes)
#define GOODIX_REG_MAX_Y        0x804A
// Maximum number of output contacts: 1~5 (4 bit value 3:0, 7:4 is reserved)
#define GOODIX_REG_MAX_TOUCH    0x804C

// Module switch 1
// 7:6 Reserved, 5:4 Stretch rank, 3 X2Y, 2 SITO (Single sided ITO touch screen), 1:0 INT Trigger mode */
#define GOODIX_REG_MOD_SW1      0x804D
// Module switch 2
// 7:1 Reserved, 0 Touch key */
#define GOODIX_REG_MOD_SW2      0x804E

// Number of debuffs fingers press/release
#define GOODIX_REG_SHAKE_CNT    0x804F

// ReadOnly registers (device and coordinates info)
// Product ID (LSB 4 bytes, GT9110: 0x06 0x00 0x00 0x09)
#define GOODIX_REG_ID           0x8140
// Firmware version (LSB 2 bytes)
#define GOODIX_REG_FW_VER       0x8144

// Current output X resolution (LSB 2 bytes)
#define GOODIX_READ_X_RES       0x8146
// Current output Y resolution (LSB 2 bytes)
#define GOODIX_READ_Y_RES       0x8148
// Module vendor ID
#define GOODIX_READ_VENDOR_ID   0x814A

#define GOODIX_READ_COORD_ADDR  0x814E

/* Commands for REG_COMMAND */
//0: read coordinate state
#define GOODIX_CMD_READ         0x00
// 1: difference value original value
#define GOODIX_CMD_DIFFVAL      0x01
// 2: software reset
#define GOODIX_CMD_SOFTRESET    0x02
// 3: Baseline update
#define GOODIX_CMD_BASEUPDATE   0x03
// 4: Benchmark calibration
#define GOODIX_CMD_CALIBRATE    0x04
// 5: Off screen (send other invalid)
#define GOODIX_CMD_SCREEN_OFF   0x05

/* When data needs to be sent, the host sends command 0x21 to GT9x,
 * enabling GT911 to enter "Approach mode" and work as a transmitting terminal */
#define GOODIX_CMD_HOTKNOT_TX   0x21

#define RESOLUTION_LOC    1
#define MAX_CONTACTS_LOC  5
#define TRIGGER_LOC 6

class Goodix {
  public:
    uint8_t i2cAddr;
    struct GTConfig config;
    struct GTInfo info;
    uint8_t points[GOODIX_MAX_CONTACTS*GOODIX_CONTACT_SIZE]; //points buffer

    Goodix(uint8_t interruptPin, uint8_t resetPin, uint8_t addr);

    void setHandler(void (*handler)(int8_t, GTPoint*));

    bool begin();
    bool reset();
    uint8_t test();
    void loop();

    uint8_t write(uint16_t reg, uint8_t *buf, size_t len);
    uint8_t write(uint16_t reg, uint8_t value);
    uint8_t read(uint16_t reg, uint8_t *buf, size_t len);

    uint8_t calcChecksum(uint8_t* buf, uint8_t len);
    uint8_t readChecksum();

    void fwResolution(uint16_t maxX, uint16_t maxY);
    
    GTConfig* readConfig();
    GTInfo* readInfo();
    uint8_t readConfigVersion();
    
    uint8_t productID(char *buf);

    int16_t readInput(uint8_t *data);

	void setWireObject(TwoWire *wire) {_wire = wire; }
    void armIRQ();
    void disArmIRQ();
    void onIRQ();

  //--- Private routines ---
  private:
    uint8_t intPin, rstPin;
    void (*touchHandler)(int8_t, GTPoint*);

	TwoWire *_wire=&Wire; // Default wire object.

    void debugPin(uint8_t level);
//    void armIRQ();
//    void onIRQ();

    //--- utils ---
    void i2cStart(uint16_t reg);
    void i2cRestart();
    uint8_t i2cStop();
    
    uint8_t previousPoints[GOODIX_MAX_CONTACTS*GOODIX_CONTACT_SIZE];
    uint8_t previousContacts;
};

#endif
