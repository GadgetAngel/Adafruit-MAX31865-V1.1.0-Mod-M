/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef ADAFRUIT_MAX31865_H
#define ADAFRUIT_MAX31865_H

#define MAX31856_CONFIG_REG 0x00
#define MAX31856_CONFIG_BIAS 0x80
#define MAX31856_CONFIG_MODEAUTO 0x40
#define MAX31856_CONFIG_MODEOFF 0x00
#define MAX31856_CONFIG_1SHOT 0x20
#define MAX31856_CONFIG_3WIRE 0x10
#define MAX31856_CONFIG_24WIRE 0x00
#define MAX31856_CONFIG_FAULTSTAT 0x02
#define MAX31856_CONFIG_FILT50HZ 0x01
#define MAX31856_CONFIG_FILT60HZ 0x00

#define MAX31856_RTDMSB_REG 0x01
#define MAX31856_RTDLSB_REG 0x02
#define MAX31856_HFAULTMSB_REG 0x03
#define MAX31856_HFAULTLSB_REG 0x04
#define MAX31856_LFAULTMSB_REG 0x05
#define MAX31856_LFAULTLSB_REG 0x06
#define MAX31856_FAULTSTAT_REG 0x07

#define MAX31865_FAULT_HIGHTHRESH 0x80
#define MAX31865_FAULT_LOWTHRESH 0x40
#define MAX31865_FAULT_REFINLOW 0x20
#define MAX31865_FAULT_REFINHIGH 0x10
#define MAX31865_FAULT_RTDINLOW 0x08
#define MAX31865_FAULT_OVUV 0x04

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7


#if (ARDUINO >= 100 || ARDUINOLPC)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

typedef enum max31865_numwires {
  MAX31865_2WIRE = 0,
  MAX31865_3WIRE = 1,
  MAX31865_4WIRE = 0
} max31865_numwires_t;

/*! Interface class for the MAX31865 RTD Sensor reader */
class Adafruit_MAX31865 {
public:
  Adafruit_MAX31865(uint32_t spi_cs, uint32_t spi_mosi, uint32_t spi_miso,
                    uint32_t spi_clk, uint8_t pin_mapping);

  Adafruit_MAX31865(uint32_t spi_cs, uint8_t pin_mapping);

  Adafruit_MAX31865(int8_t spi_cs, int8_t spi_mosi, int8_t spi_miso,
                    int8_t spi_clk);

  Adafruit_MAX31865(int8_t spi_cs);


  bool begin(max31865_numwires_t x = MAX31865_2WIRE);

  uint8_t readFault(void);
  void clearFault(void);
  uint16_t readRTD();

  void setWires(max31865_numwires_t wires);
  void autoConvert(bool b);
  void enable50Hz(bool b);
  void enableBias(bool b);

  float temperature(float RTDnominal, float refResistor);
  uint16_t readRTD_Resistance(uint32_t refResistor);
  uint16_t readRTD_with_Fault();

private:
  int8_t _sclk, _miso, _mosi, _cs;
  uint32_t __sclk, __miso, __mosi, __cs;
  uint8_t __pin_mapping = 0x00;
  bool first_reading = true;

  void readRegisterN(uint8_t addr, uint8_t buffer[], uint8_t n);

  uint8_t readRegister8(uint8_t addr);
  uint16_t readRegister16(uint8_t addr);

  void writeRegister8(uint8_t addr, uint8_t reg);
  uint8_t spixfer(uint8_t addr);
};

#endif
