#include <cstdint>

#ifndef BMP388_H
#define BMP388_H
// define BMP388 Device I2C address

#define I2C_ADD_BMP388_AD0_LOW 0x76
#define I2C_ADD_BMP388_AD0_HIGH 0x77
#define I2C_ADD_BMP388  I2C_ADD_BMP388_AD0_HIGH

#define BMP388_REG_ADD_WIA 0x00
#define BMP388_REG_VAL_WIA 0x50

#define BMP388_REG_ADD_ERR 0x02
#define BMP388_REG_VAL_FATAL_ERR 0x01
#define BMP388_REG_VAL_CMD_ERR 0x02
#define BMP388_REG_VAL_CONF_ERR 0x04

#define BMP388_REG_ADD_STATUS 0x03
#define BMP388_REG_VAL_CMD_RDY 0x10
#define BMP388_REG_VAL_DRDY_PRESS 0x20
#define BMP388_REG_VAL_DRDY_TEMP 0x40

#define BMP388_REG_ADD_CMD 0x7E
#define BMP388_REG_VAL_EXTMODE_EN 0x34
#define BMP388_REG_VAL_FIFI_FLUSH 0xB0
#define BMP388_REG_VAL_SOFT_RESET 0xB6

#define BMP388_REG_ADD_PWR_CTRL 0x1B
#define BMP388_REG_VAL_PRESS_EN 0x01
#define BMP388_REG_VAL_TEMP_EN 0x02
#define BMP388_REG_VAL_NORMAL_MODE 0x30

#define BMP388_REG_ADD_PRESS_XLSB 0x04
#define BMP388_REG_ADD_PRESS_LSB 0x05
#define BMP388_REG_ADD_PRESS_MSB 0x06
#define BMP388_REG_ADD_TEMP_XLSB 0x07
#define BMP388_REG_ADD_TEMP_LSB 0x08
#define BMP388_REG_ADD_TEMP_MSB 0x09

#define BMP388_REG_ADD_T1_LSB 0x31
#define BMP388_REG_ADD_T1_MSB 0x32
#define BMP388_REG_ADD_T2_LSB 0x33
#define BMP388_REG_ADD_T2_MSB 0x34
#define BMP388_REG_ADD_T3 0x35
#define BMP388_REG_ADD_P1_LSB 0x36
#define BMP388_REG_ADD_P1_MSB 0x37
#define BMP388_REG_ADD_P2_LSB 0x38
#define BMP388_REG_ADD_P2_MSB 0x39
#define BMP388_REG_ADD_P3 0x3A
#define BMP388_REG_ADD_P4 0x3B
#define BMP388_REG_ADD_P5_LSB 0x3C
#define BMP388_REG_ADD_P5_MSB 0x3D
#define BMP388_REG_ADD_P6_LSB 0x3E
#define BMP388_REG_ADD_P6_MSB 0x3F
#define BMP388_REG_ADD_P7 0x40
#define BMP388_REG_ADD_P8 0x41
#define BMP388_REG_ADD_P9_LSB 0x42
#define BMP388_REG_ADD_P9_MSB 0x43
#define BMP388_REG_ADD_P10 0x44
#define BMP388_REG_ADD_P11 0x45

class BMP388
{
 public:
  BMP388();
  double getTemperature();
  double getPressure();
  double getAltitude();
  void getTempPressAlt();
  
 private:

  int file = -1;
  double tFine;
  double temperature;
  double pressure;
  double altitude;

  uint16_t t1 = 1;
  uint16_t t2 = 2;
  int8_t t3 = 3;
  int16_t p1 = 4;
  int16_t p2 = 5 ;
  int8_t p3 = 6;
  int8_t p4 = 7;
  uint16_t p5 = 8;
  uint16_t p6 = 9;
  int8_t p7 = 10;
  int8_t p8 = 11 ;
  int16_t p9 = 12;
  int8_t p10 = 13;
  int8_t p11 = 14;

  
  void selectDevice(int file, int addr);
  void init();
  void writeByte();
  void readBlock(uint8_t command, uint8_t size, uint8_t *data);
  int readByte(uint8_t command);
  int readWord(uint8_t command);
  void loadCalibration();
  void dumpCalibration();
  double compensateTemperature(int adc_T);
  double compensatePressure(int adc_P);
  
}
; 
#endif
