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

//#define DEBUG_STM32
//#define DEBUG_STM32_SPI
//#define DEBUG_LPC_SPI
//#define DEBUG_LPC

#if defined(ARDUINO_ARCH_STM32) && defined(DEBUG_STM32)
  #define HAS_STM32_DEBUG 1
#endif

#if defined(ARDUINO_ARCH_STM32) && defined(DEBUG_STM32_SPI)
  #define HAS_STM32_DEBUG_SPI 1
#endif

#if defined(TARGET_LPC1768) && defined(DEBUG_LPC)
  #define HAS_LPC1768_DEBUG 1
#endif

#if defined(TARGET_LPC1768) && defined(DEBUG_LPC_SPI)
  #define HAS_LPC1768_DEBUG_SPI 1
#endif

#if HAS_LPC1768_DEBUG || HAS_LPC1768_DEBUG_SPI
  #include "../../../../Marlin/src/inc/MarlinConfig.h"
#endif

#include "Adafruit_MAX31865.h"

#include "../../../../Marlin/src/HAL/shared/Delay.h"

#ifdef __AVR__
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif

#include <stdlib.h>
#include <SPI.h>

#if defined(TARGET_LPC1768)
  #if defined(TEMP_MODE3)
    static SPISettings max31865_spisettings =
      SPISettings(SPI_QUARTER_SPEED, MSBFIRST, SPI_MODE3);
  #else
    static SPISettings max31865_spisettings =
      SPISettings(SPI_QUARTER_SPEED, MSBFIRST, SPI_MODE1);
  #endif
#elif defined(ARDUINO_ARCH_STM32)
  #if defined(TEMP_MODE3)
    static SPISettings max31865_spisettings =
      SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE3);
  #else
    static SPISettings max31865_spisettings =
      SPISettings(SPI_CLOCK_DIV4, MSBFIRST, SPI_MODE1);
  #endif
#else
  #if defined(TEMP_MODE3)
    static SPISettings max31865_spisettings =
      SPISettings(500000, MSBFIRST, SPI_MODE3);
  #else
    static SPISettings max31865_spisettings =
      SPISettings(500000, MSBFIRST, SPI_MODE1);
  #endif
#endif

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI for
    PIN values which are larger than 127. If you have PIN values less than
    or equal to 127 use the other call for SW SPI.
    @param spi_cs the SPI CS pin to use
    @param spi_mosi the SPI MOSI pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_clk the SPI clock pin to use
    @param pin_mapping set to 1 for positive pin values
*/
/**************************************************************************/
Adafruit_MAX31865::Adafruit_MAX31865(uint32_t spi_cs, uint32_t spi_mosi,
                                     uint32_t spi_miso, uint32_t spi_clk,
                                     uint8_t pin_mapping) {
  __cs = spi_cs;
  __mosi = spi_mosi;
  __miso = spi_miso;
  __sclk = spi_clk;
  __pin_mapping = pin_mapping;

  if (__pin_mapping == 0)  {
    _cs = __cs;
    _mosi = __mosi;
    _miso = __miso;
    _sclk = __sclk;
  }

}

/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI for PIN values
    which are larger than 127. If you have PIN values less than
    or equal to 127 use the other call for HW SPI
    @param spi_cs the SPI CS pin to use along with the default SPI device
    @param pin_mapping set to 1 for positive pin values
*/
/**************************************************************************/
Adafruit_MAX31865::Adafruit_MAX31865(uint32_t spi_cs, uint8_t pin_mapping) {
  __cs = spi_cs;
  __sclk = __miso = __mosi = -1UL;  //-1UL or 0xFFFFFFFF
  __pin_mapping = pin_mapping;

  if (__pin_mapping == 0) {
    _cs = __cs;
    _mosi = -1;
    _miso = -1;
    _sclk = -1;
  }
}

/**************************************************************************/
/*!
    @brief Create the interface object using software (bitbang) SPI for PIN
    values less than or equal to 127.
    @param spi_cs the SPI CS pin to use
    @param spi_mosi the SPI MOSI pin to use
    @param spi_miso the SPI MISO pin to use
    @param spi_clk the SPI clock pin to use
*/
/**************************************************************************/
Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs, int8_t spi_mosi,
                                     int8_t spi_miso, int8_t spi_clk) {
  _cs = spi_cs;
  _mosi = spi_mosi;
  _miso = spi_miso;
  _sclk = spi_clk;
}

/**************************************************************************/
/*!
    @brief Create the interface object using hardware SPI for PIN for PIN
    values less than or equal to 127.
    @param spi_cs the SPI CS pin to use along with the default SPI device
*/
/**************************************************************************/
Adafruit_MAX31865::Adafruit_MAX31865(int8_t spi_cs) {
  _cs = spi_cs;
  _sclk = _miso = _mosi = -1;
}

/**************************************************************************/
/*!
    @brief Initialize the SPI interface and set the number of RTD wires used
    @param wires The number of wires in enum format. Can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @return True
*/
/**************************************************************************/
bool Adafruit_MAX31865::begin(max31865_numwires_t wires) {

  if (!__pin_mapping) {
    pinMode(_cs, OUTPUT);
    digitalWrite(_cs, HIGH);
  }
  else {
    pinMode(__cs, OUTPUT);
    digitalWrite(__cs, HIGH);
  }

  // define pin modes
  if (_sclk != -1 || __sclk != -1UL) {
    if (!__pin_mapping) {
      pinMode(_sclk, OUTPUT);
      digitalWrite(_sclk, LOW);
      pinMode(_mosi, OUTPUT);
      pinMode(_miso, INPUT);
    }
    else {
      pinMode(__sclk, OUTPUT);
      digitalWrite(__sclk, LOW);
      pinMode(__mosi, OUTPUT);
      pinMode(__miso, INPUT);
    }
  }
  else {
    // start and configure hardware SPI
    SPI.begin();
  }

  setWires(wires);
  enableBias(false);
  autoConvert(false);
  clearFault();

  #if HAS_STM32_DEBUG
    //Serial.print("config: ");
    //Serial.println(readRegister8(MAX31856_CONFIG_REG), HEX);
  #endif

  #if HAS_STM32_DEBUG_SPI
    if (!__pin_mapping) {
      Serial.print("\n\n_cs: ");
      Serial.print(_cs);
      Serial.print(" _cs: 0x");
      Serial.print(_cs, HEX);
      uint32_t cs2 = 0;
      cs2 = _cs;
      Serial.print("\n uint32_t cs2: ");
      Serial.print(cs2);
      Serial.print("  cs2: 0x");
      Serial.print(cs2, HEX);
      Serial.print("\n _miso: ");
      Serial.print(_miso);
      Serial.print(" _miso: 0x");
      Serial.print(_miso, HEX);
      Serial.print("\n _sclk: ");
      Serial.print(_sclk);
      Serial.print(" _sclk: 0x");
      Serial.print(_sclk, HEX);
      Serial.print("\n _mosi: ");
      Serial.print(_mosi);
      Serial.print(" _mosi: 0x");
      Serial.print(_mosi, HEX);
      Serial.print("\n\n");
    }
    else {
      Serial.print("\n\n__cs: ");
      Serial.print(__cs);
      Serial.print(" __cs: 0x");
      Serial.print(__cs, HEX);
      Serial.print("\n __miso: ");
      Serial.print(__miso);
      Serial.print(" __miso: 0x");
      Serial.print(__miso, HEX);
      Serial.print("\n __sclk: ");
      Serial.print(__sclk);
      Serial.print(" __sclk: 0x");
      Serial.print(__sclk, HEX);
      Serial.print("\n __mosi: ");
      Serial.print(__mosi);
      Serial.print(" __mosi: 0x");
      Serial.print(__mosi, HEX);
      Serial.print("\n __pin_mapping: ");
      Serial.print(__pin_mapping);
      Serial.print("\n\n");
    }
  #endif

  #if HAS_LPC1768_DEBUG_SPI
    // for testing
    if (!__pin_mapping) {
      SERIAL_ECHOLN();
      SERIAL_ECHOLNPAIR("Regular call for _cs: ", _cs ," _miso: ", _miso ," _sclk: ", _sclk," _mosi: ",_mosi);
      SERIAL_PRINTF("Regular call for _cs: 0x%X  _miso: 0x%X  _sclk: 0x%X  _mosi: 0x%X  ", _cs, _miso, _sclk, _mosi);
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
    }
    else {
      SERIAL_ECHOLN();
      SERIAL_ECHOLNPAIR("PIN_MAPPING call for __cs: ", __cs ," __miso: ", __miso ," __sclk: ", __sclk," __mosi: ",__mosi);
      SERIAL_PRINTF("PIN_MAPPING call for __cs: 0x%X  __miso: 0x%X  __sclk: 0x%X  __mosi: 0x%X  ", __cs, __miso, __sclk, __mosi);
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
    }
  #endif

  return true;
}

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t Adafruit_MAX31865::readFault(void) {
  return readRegister8(MAX31856_FAULTSTAT_REG);
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void Adafruit_MAX31865::clearFault(void) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t &= ~0x2C;
  t |= MAX31856_CONFIG_FAULTSTAT;
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void Adafruit_MAX31865::enableBias(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_BIAS; // enable bias
  } else {
    t &= ~MAX31856_CONFIG_BIAS; // disable bias
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want to have continuous conversions (50/60 Hz)
    @param b If true, auto conversion is enabled
*/
/**************************************************************************/
void Adafruit_MAX31865::autoConvert(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_MODEAUTO; // enable autoconvert
  } else {
    t &= ~MAX31856_CONFIG_MODEAUTO; // disable autoconvert
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Whether we want filter out 50Hz noise or 60Hz noise
    @param b If true, 50Hz noise is filtered, else 60Hz(default)
*/
/**************************************************************************/

void Adafruit_MAX31865::enable50Hz(bool b) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (b) {
    t |= MAX31856_CONFIG_FILT50HZ;
  } else {
    t &= ~MAX31856_CONFIG_FILT50HZ;
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief How many wires we have in our RTD setup, can be MAX31865_2WIRE,
    MAX31865_3WIRE, or MAX31865_4WIRE
    @param wires The number of wires in enum format
*/
/**************************************************************************/
void Adafruit_MAX31865::setWires(max31865_numwires_t wires) {
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  if (wires == MAX31865_3WIRE) {
    t |= MAX31856_CONFIG_3WIRE;
  } else {
    // 2 or 4 wire
    t &= ~MAX31856_CONFIG_3WIRE;
  }
  writeRegister8(MAX31856_CONFIG_REG, t);
}

/**************************************************************************/
/*!
    @brief Read the temperature in C from the RTD through calculation of the
    resistance. Uses
   http://www.analog.com/media/en/technical-documentation/application-notes/AN709_0.pdf
   technique
    @param RTDnominal The 'nominal' resistance of the RTD sensor, usually 100
    or 1000
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @returns Temperature in C
*/
/**************************************************************************/
float Adafruit_MAX31865::temperature(float RTDnominal, float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  // prime the SPI communication channel
  if (!first_reading)
    Rt = readRTD();
  else {
    first_reading = false;
    Rt = readRTD();
    #if HAS_STM32_DEBUG
      Serial.print("\n\n1st Reading: 0x");
      Serial.println(d, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("1st Reading:");
      SERIAL_PRINTF("   0x%X  ", d);
      SERIAL_ECHOLN();
    #endif
    Rt = readRTD();
    #if HAS_STM32_DEBUG
      Serial.print("\n\n2nd Reading: 0x");
      Serial.println(d, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("2nd Reading:");
      SERIAL_PRINTF("   0x%X  ", d);
      SERIAL_ECHOLN();
    #endif
  }
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865::readRTD(void) {
  clearFault();
  enableBias(true);
  DELAY_US(10000);
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_1SHOT;
  writeRegister8(MAX31856_CONFIG_REG, t);
  DELAY_US(65000);

  uint16_t rtd = readRegister16(MAX31856_RTDMSB_REG);

  // remove fault
  rtd >>= 1;

  return rtd;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode and
    calucalte the RTD resistance value
    @param refResistor The value of the matching reference resistor, usually
    430 or 4300
    @return The raw unsigned 16-bit RTD resistance value, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865::readRTD_Resistance(uint32_t refResistor) {
  uint32_t Rt;
  uint16_t rtd;

  // prime the SPI communication channel
  if (!first_reading)
    rtd = readRTD();
  else {
    first_reading = false;
    rtd = readRTD();
    #if HAS_STM32_DEBUG
      Serial.print("\n\n1st Reading: 0x");
      Serial.println(rtd, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("1st Reading:");
      SERIAL_PRINTF("   0x%X  ", rtd);
      SERIAL_ECHOLN();
    #endif
    rtd = readRTD();
    #if HAS_STM32_DEBUG
      Serial.print("\n\n2nd Reading: 0x");
      Serial.println(rtd, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("2nd Reading:");
      SERIAL_PRINTF("   0x%X  ", rtd);
      SERIAL_ECHOLN();
    #endif
  }

  Rt = rtd;
  Rt *= refResistor;
  Rt >>= 16;
  rtd = 0;

  rtd = Rt;

  return rtd;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit RTD value with ERROR bit attached, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865::readRTD_with_Fault(void) {
  uint16_t rtd;

  clearFault();
  enableBias(true);
  DELAY_US(10000);
  uint8_t t = readRegister8(MAX31856_CONFIG_REG);
  t |= MAX31856_CONFIG_1SHOT;
  writeRegister8(MAX31856_CONFIG_REG, t);
  DELAY_US(65000);

  // prime the SPI communication channel
  if (!first_reading)
    rtd = readRegister16(MAX31856_RTDMSB_REG);
  else {
    first_reading = false;
    rtd = readRegister16(MAX31856_RTDMSB_REG);
    #if HAS_STM32_DEBUG
      Serial.print("\n\n1st Reading: 0x");
      Serial.println(rtd, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("1st Reading:");
      SERIAL_PRINTF("   0x%X  ", rtd);
      SERIAL_ECHOLN();
    #endif
    rtd = readRegister16(MAX31856_RTDMSB_REG);
    #if HAS_STM32_DEBUG
      Serial.print("\n\n2nd Reading: 0x");
      Serial.println(rtd, HEX);
    #endif
    #if HAS_LPC1768_DEBUG
      SERIAL_ECHOLN();
      SERIAL_ECHOLN();
      SERIAL_ECHO("2nd Reading:");
      SERIAL_PRINTF("   0x%X  ", rtd);
      SERIAL_ECHOLN();
    #endif
  }

  #if HAS_STM32_DEBUG || HAS_LPC1768_DEBUG
    uint16_t rtd_MSB = rtd >> 8;
    uint16_t rtd_LSB = rtd & 0x00FF;
  #endif
  #if HAS_STM32_DEBUG
    Serial.println(" ");
    Serial.print("RTD MSB : 0x");
    Serial.print(rtd_MSB, HEX);
    Serial.print("  : ");
    Serial.print(rtd_MSB);
    Serial.print("  RTD LSB : 0x");
    Serial.print(rtd_LSB, HEX);
    Serial.print("  : ");
    Serial.println(rtd_LSB);
    Serial.println(" ");
  #endif
  #if HAS_LPC1768_DEBUG
    SERIAL_ECHOLN();
    SERIAL_ECHO("RTD MSB : ");
    SERIAL_PRINTF("   0x%X  ", rtd_MSB);
    SERIAL_ECHOPAIR(" : ", rtd_MSB);
    SERIAL_ECHO("   RTD LSB : ");
    SERIAL_PRINTF("   0x%X  ", rtd_LSB);
    SERIAL_ECHOLNPAIR(" : ", rtd_LSB,"   ");
    SERIAL_ECHOLN();
  #endif

  return rtd;
}

/**********************************************/

uint8_t Adafruit_MAX31865::readRegister8(uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(addr, &ret, 1);

  return ret;
}

uint16_t Adafruit_MAX31865::readRegister16(uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |= buffer[1];

  return ret;
}

void Adafruit_MAX31865::readRegisterN(uint8_t addr, uint8_t buffer[],
                                      uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set

  if (_sclk == -1 || __sclk == -1UL)
    SPI.beginTransaction(max31865_spisettings);
  else
    if (!__pin_mapping)
      digitalWrite(_sclk, LOW);
    else
      digitalWrite(__sclk, LOW);

  if (!__pin_mapping)
    digitalWrite(_cs, LOW);
  else
    digitalWrite(__cs, LOW);


  spixfer(addr);

  // Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
  while (n--) {
    buffer[0] = spixfer(0xFF);
    // Serial.print(" 0x"); Serial.print(buffer[0], HEX);
    buffer++;
  }
  // Serial.println();

  if (_sclk == -1 || __sclk == -1UL)
    SPI.endTransaction();

  if (!__pin_mapping)
    digitalWrite(_cs, HIGH);
  else
    digitalWrite(__cs, HIGH);
}

void Adafruit_MAX31865::writeRegister8(uint8_t addr, uint8_t data) {
  if (_sclk == -1 || __sclk == -1UL)
    SPI.beginTransaction(max31865_spisettings);
  else
    if (!__pin_mapping)
      digitalWrite(_sclk, LOW);
    else
      digitalWrite(__sclk, LOW);

  if (!__pin_mapping)
    digitalWrite(_cs, LOW);
  else
    digitalWrite(__cs, LOW);

  spixfer(addr | 0x80); // make sure top bit is set
  spixfer(data);

  // Serial.print("$"); Serial.print(addr, HEX); Serial.print(" = 0x");
  // Serial.println(data, HEX);

  if (_sclk == -1 || __sclk == -1UL)
    SPI.endTransaction();

  if (!__pin_mapping)
    digitalWrite(_cs, HIGH);
  else
    digitalWrite(__cs, HIGH);
}

uint8_t Adafruit_MAX31865::spixfer(uint8_t x) {
  if (_sclk == -1 || __sclk == -1UL)
    return SPI.transfer(x);

  // software spi
  // Serial.println("Software SPI");
  uint8_t reply = 0;

  for (int i = 7; i >= 0; i--) {
    reply <<= 1;
    if (!__pin_mapping) {
      digitalWrite(_sclk, HIGH);
      digitalWrite(_mosi, x & (1 << i));
      digitalWrite(_sclk, LOW);
      if (digitalRead(_miso))
        reply |= 1;
    }
    else {
      digitalWrite(__sclk, HIGH);
      digitalWrite(__mosi, x & (1 << i));
      digitalWrite(__sclk, LOW);
      if (digitalRead(__miso))
        reply |= 1;
    }
  }

  return reply;
}
