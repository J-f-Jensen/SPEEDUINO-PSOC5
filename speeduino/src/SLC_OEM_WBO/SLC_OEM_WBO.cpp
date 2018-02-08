#include <Arduino.h>
#include <WIRE.h>
#include "SLC_OEM_WBO.h"


void SLC_OEM_WBO_init( void )
{
  Wire.begin();
}

/********************************************************************************
*  function for reading data from 14POINT7 SLC_OEM WBO controller using I2C
*  returning AFR*10 between 100 and 200 (AFR 10.0 to 20.0)
*  or 0xFF on error or when the sensor is outside recommended temperature range
********************************************************************************/
uint8_t SLC_OEM_WBO_readAFR( uint8_t I2C_Address )
{
  float            WBOData_float    = 0;
  unsigned int     WBORi_Min_uint   = 0;
  unsigned int     WBORi_Max_uint   = 0;
  unsigned int     WBOTemp_uint     = 0;
  unsigned int     WBODataAfr_uint  = 0;

  Wire.beginTransmission(I2C_Address); // I2C address
  Wire.write(0x0B); // point to Ri_Max register
  Wire.endTransmission(); // send to I2C device
  Wire.requestFrom(0x02,4); // request 4 bites of data from WBO, Ri_Max and Ri_Min

  if (Wire.available() == 4)
  {
    // Read the recived bytes
    ((byte*)&WBORi_Max_uint)[1]= Wire.read();
    ((byte*)&WBORi_Max_uint)[0]= Wire.read();
    ((byte*)&WBORi_Min_uint)[1]= Wire.read();
    ((byte*)&WBORi_Min_uint)[0]= Wire.read();

    WBOTemp_uint = WBORi_Max_uint - WBORi_Min_uint;
  }

  // Check if WBO is within recommended temp range, if not return FF
  if (WBOTemp_uint > 65 && WBOTemp_uint < 105)
  {
    Wire.beginTransmission(I2C_Address); // I2C address
    Wire.write(0x03); // point SLC to 32bit Lambda register
    Wire.endTransmission(); // send to I2C device
    Wire.requestFrom(0x02,4); // request 4 bites of data from WBO

    if (Wire.available() == 4)
    {
      // Read the recived bytes into the float
      ((byte*)&WBOData_float)[3]= Wire.read();
      ((byte*)&WBOData_float)[2]= Wire.read();
      ((byte*)&WBOData_float)[1]= Wire.read();
      ((byte*)&WBOData_float)[0]= Wire.read();
    }

    WBODataAfr_uint = (unsigned int) (WBOData_float * 147); // Convert from lambda 32 bit float value to AFR*10 uint8 value

    // Limit the data range returned between 0.68 Lambda to 1.36 Lambda equal AFR*10 between 100 to 200
    if (WBODataAfr_uint < 100) { WBODataAfr_uint = 100; }
    if (WBODataAfr_uint > 200) { WBODataAfr_uint = 200; }
  }
  else
  {
    WBODataAfr_uint = 0xFF;
  }

  return (uint8_t) WBODataAfr_uint;
}
