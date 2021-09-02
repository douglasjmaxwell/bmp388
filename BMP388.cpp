#include <stdint.h>
#include  "BMP388.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fcntl.h>
#include <cmath>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>

extern "C"
{
#include "smbus.h"
#include "linux/i2c-dev.h"
}

using namespace std;

BMP388::BMP388()
{
  init();
}
 double BMP388::getTemperature()
{
  return temperature;
}
double   BMP388::getPressure()
{
  return pressure;
}
double BMP388::getAltitude()
{
  return altitude;
}
  
void BMP388::init()
{
  char filename[20];
  sprintf(filename, "/dev/i2c-%d", 1);
  file = open(filename, O_RDWR);
  if (file<0)
  {
    printf("Unable to open I2C bus!");
    exit(1);
  }

  selectDevice(file,I2C_ADD_BMP388);
  
  int u8RegData;
  if (i2c_smbus_read_byte_data(file,BMP388_REG_ADD_WIA) == BMP388_REG_VAL_WIA)
  {
    u8RegData = i2c_smbus_read_byte_data(file,BMP388_REG_ADD_STATUS);
    cout << "u8RegData: " << u8RegData << endl;
    if (u8RegData & BMP388_REG_VAL_CMD_RDY)
    {
      i2c_smbus_write_byte_data(file,BMP388_REG_ADD_CMD, BMP388_REG_VAL_SOFT_RESET);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

    }    
  }
  else
  {
    cout << "Pressure Sensor is null" << endl;
  }
  i2c_smbus_write_byte_data(file,BMP388_REG_ADD_PWR_CTRL, BMP388_REG_VAL_PRESS_EN
                         | BMP388_REG_VAL_TEMP_EN
			    | BMP388_REG_VAL_NORMAL_MODE);

  loadCalibration();
    
}

void BMP388::loadCalibration()
{
  //dumpCalibration();
  readBlock((uint8_t)BMP388_REG_ADD_T1_LSB,sizeof(t1),(uint8_t*)&t1);
  readBlock(BMP388_REG_ADD_T2_LSB,sizeof(t2),(uint8_t*)&t2);
  readBlock(BMP388_REG_ADD_T3,sizeof(t3),(uint8_t*)&t3);
  readBlock(BMP388_REG_ADD_P1_LSB,sizeof(p1),(uint8_t*)&p1);
  readBlock(BMP388_REG_ADD_P2_LSB,sizeof(p2),(uint8_t*)&p2);
  readBlock(BMP388_REG_ADD_P3,sizeof(p3),(uint8_t*)&p3);
  readBlock(BMP388_REG_ADD_P4,sizeof(p4),(uint8_t*)&p4);
  readBlock(BMP388_REG_ADD_P5_LSB,sizeof(p5),(uint8_t*)&p5);
  readBlock(BMP388_REG_ADD_P6_LSB,sizeof(p6),(uint8_t*)&p6);
  readBlock(BMP388_REG_ADD_P7,sizeof(p7),(uint8_t*)&p7);
  readBlock(BMP388_REG_ADD_P8,sizeof(p8),(uint8_t*)&p8);
  readBlock(BMP388_REG_ADD_P9_LSB,sizeof(p9),(uint8_t*)&p9);
  readBlock(BMP388_REG_ADD_P10,sizeof(p10),(uint8_t*)&p10);
  readBlock(BMP388_REG_ADD_P11,sizeof(p11),(uint8_t*)&p11);
  dumpCalibration();

}

void BMP388::dumpCalibration()
{
  cout << "t1: " << (int)t1 << endl;
  cout << "t2: " << (int)t2 << endl;
  cout << "t3: " << (int)t3 << endl;
  cout << "p1: " << (int)p1 << endl;
  cout << "p2: " << (int)p2 << endl;
  cout << "p3: " << (int)p3 << endl;
  cout << "p4: " << (int)p4 << endl;
  cout << "p5: " << (int)p5 << endl;
  cout << "p6: " << (int)p6 << endl;
  cout << "p7: " << (int)p7 << endl;
  cout << "p8: " << (int)p8 << endl;
  cout << "p9: " << (int)p9 << endl;
  cout << "p10: " << (int)p10 << endl;
  cout << "p11: " << (int)p11 << endl;
  cout << "***************" << endl;
}


// int u8RegData;
// readByte();
// readS8();
// readU16();
// readS16();
// writeByte();

double BMP388::compensateTemperature(int adc_T)
{
  double partial_data1 = adc_T - 256 * t1;
  double partial_data2 = t2 * partial_data1;
  double partial_data3 = partial_data1 * partial_data1;
  double partial_data4 = partial_data3 * t3;
  double partial_data5 = partial_data2 * 262144 + partial_data4;
  double partial_data6 = partial_data5 / 4294967296.0;
  tFine = partial_data6;
  double comp_temp = partial_data6 * 25 / 16384;
  /*  cout << "adc_T: " << adc_T
  << " data1: " << partial_data1 
  << " data2: " << partial_data2 
  << " data3: " << partial_data3 
  << " data4: " << partial_data4 
  << " data5: " << partial_data5 
  << " data6: " << partial_data6
  << " tFine: " << tFine << " comp_temp: " << comp_temp     << endl;
  */
  return comp_temp;
}

double BMP388:: compensatePressure(int adc_P)
{
  double partial_data1 = tFine * tFine;
  double partial_data2 = partial_data1 / 0x40;
  double partial_data3 = partial_data2 * tFine / 256;
  double partial_data4 = p8 * partial_data3 / 0x20;
  double partial_data5 = p7 * partial_data1 * 0x10;
  double partial_data6 = p6 * tFine * 4194304;
  double offset = (p5*1.0) * 140737488355328 + partial_data4 + partial_data5 + partial_data6;

    cout << "adc_P: " << adc_P
  << " data1: " << partial_data1 
  << " data2: " << partial_data2 
  << " data3: " << partial_data3 
  << " data4: " << partial_data4 
  << " data5: " << partial_data5 
  << " data6: " << partial_data6
  << " tFine: " << tFine << " offset: " << offset     << endl;
  

  
  partial_data2 = p4 * partial_data3 / 0x20;
  partial_data4 = p3 * partial_data1 * 0x04;
  partial_data5 = (p2 - 16384) * tFine * 2097152;
  double sensitivity = (p1 - 16384) * 70368744177664 + partial_data2 + partial_data4 + partial_data5;

  cout << " data2: " << partial_data2 
  << " data4: " << partial_data4 
  << " data5: " << partial_data5 
  << " sensitivity: " << sensitivity     << endl;

  partial_data1 = sensitivity / 16777216 * adc_P;
  partial_data2 = p10 * tFine;
  partial_data3 = partial_data2 + 65536 * p9;
  partial_data4 = partial_data3 * adc_P / 8192;
  partial_data5 = partial_data4 * adc_P / 512;
  partial_data6 = (adc_P * 1.0) * (adc_P * 1.0);

  cout << " data1: " << partial_data1 
  << " data2: " << partial_data2 
  << " data3: " << partial_data3 
  << " data4: " << partial_data4 
  << " data5: " << partial_data5 
  << " data6: " << partial_data6
       << " adc_P: " << adc_P
  << endl;


  partial_data2 = p11 * partial_data6 / 65536;
  partial_data3 = partial_data2 * (adc_P*1.0) / 128;
  partial_data4 = offset / 0x04 + partial_data1 + partial_data5 + partial_data3;
  double comp_press = partial_data4 * 25.0 / 1099511627776.0;

  cout<< " data1: " << partial_data1 
  << " data2: " << partial_data2 
  << " data3: " << partial_data3 
  << " data4: " << partial_data4 
  << " data5: " << partial_data5 
  << " comp_press: " << comp_press
  << endl;
  return comp_press;

}

void BMP388::getTempPressAlt()
{
  int xlsb;
  int lsb;
  int msb;

  selectDevice(file,I2C_ADD_BMP388);

  xlsb = readByte(BMP388_REG_ADD_TEMP_XLSB);
  lsb = readByte(BMP388_REG_ADD_TEMP_LSB);
  msb = readByte(BMP388_REG_ADD_TEMP_MSB);

  //cout << "Temperature: "  << "xlsb: " << (int) xlsb << " lsb: " << (int)lsb << " msb: " <<  (int)msb << endl;
  
  int adc_T = (msb << 0x10) | (lsb << 0x08) | xlsb;

  temperature = compensateTemperature(adc_T);
    
  selectDevice(file,I2C_ADD_BMP388);

  xlsb = readByte(BMP388_REG_ADD_PRESS_XLSB);
  lsb = readByte(BMP388_REG_ADD_PRESS_LSB);
  msb = readByte(BMP388_REG_ADD_PRESS_MSB);
  
  cout << "Pressure: " << "xlsb: " <<(int) xlsb << " lsb: " <<(int) lsb << " msb: " << (int) msb << endl;

  int adc_P = (msb << 0x10) + (lsb << 0x08) + xlsb;

  pressure = compensatePressure(adc_P);

  altitude = 4433000 * (0x01 - pow(pressure / 100.0 / 101325.0,
				   0.1903));
  
}

void  BMP388::readBlock(uint8_t command, uint8_t size, uint8_t *data)
{
    int result = i2c_smbus_read_i2c_block_data(file, command, size, data);
    //    cout << "size: " << (int) size << " data: " << (int) &data << endl;
    if (result != size){
		printf("Failed to read block from I2C.");
		exit(1);
	}
}

int BMP388::readByte(uint8_t command)
{
  uint8_t ret = i2c_smbus_read_byte_data(file, command);
  // cout << " ret: " << (int)ret << endl;
  return ret;
}

int BMP388::readWord(uint8_t command)
{
  return i2c_smbus_read_word_data(file, command);
}

void BMP388::selectDevice(int file, int addr)
{
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		 printf("Failed to select I2C device.");
	}
}
