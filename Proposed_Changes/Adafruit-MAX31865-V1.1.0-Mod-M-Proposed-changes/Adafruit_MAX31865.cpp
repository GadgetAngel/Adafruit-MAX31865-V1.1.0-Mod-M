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

#if !defined(__AVR__)
  #include "../../../../Marlin/src/HAL/shared/Delay.h"
#endif

#ifdef __AVR__
  #define AVR_FLAG 1
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include <stdlib.h>
#include <SPI.h>

#if defined(TEMP_MODE)
  #if TEMP_MODE == 3
    #define MAX31865_SPI_MODE SPI_MODE3
  #elif TEMP_MODE == 2
    #define MAX31865_SPI_MODE SPI_MODE2
  #elif TEMP_MODE == 1
    #define MAX31865_SPI_MODE SPI_MODE1
  #else
    #define MAX31865_SPI_MODE SPI_MODE0
  #endif
#else
  // default to origial settings
  #define MAX31865_SPI_MODE SPI_MODE1
#endif

#if defined(TARGET_LPC1768)
  static SPISettings max31865_spisettings =
    SPISettings(SPI_QUARTER_SPEED, MSBFIRST, MAX31865_SPI_MODE);
#elif defined(ARDUINO_ARCH_STM32)
  static SPISettings max31865_spisettings =
    SPISettings(SPI_CLOCK_DIV4, MSBFIRST, MAX31865_SPI_MODE);
#else
  static SPISettings max31865_spisettings =
    SPISettings(500000, MSBFIRST, MAX31865_SPI_MODE);
#endif

#ifdef MAX31865_USE_AUTO_MODE
  #define RTD_READ_MODE MAX31865_CONFIG_MODEAUTO | MAX31865_CONFIG_BIAS
#else
  #define RTD_READ_MODE MAX31865_CONFIG_MODEOFF
#endif

#ifdef MAX31865_USE_60HZ
  #define HZ_FILTER_SETTING MAX31865_CONFIG_FILT60HZ
#else
  #define HZ_FILTER_SETTING MAX31865_CONFIG_FILT50HZ
#endif

#ifdef MAX31865_USE_3WIRE
  #define WIRE_SETTING MAX31865_CONFIG_3WIRE
#else
  #define WIRE_SETTING MAX31865_CONFIG_24WIRE
#endif

#define MAX31865_DEFAULT_CONFIG WIRE_SETTING | HZ_FILTER_SETTING | RTD_READ_MODE

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
  __sclk = __miso = __mosi = -1UL;  //-1UL or 0xFFFFFFFF or 4294967295
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

  //initalize __pin_mapping variables
  __cs = 0;
  __mosi = 0;
  __miso = 0;
  __sclk = 0;
  __pin_mapping = 0;

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

  //initialzie __pin_mapping variables
  __cs = 0;
  __mosi = 0;
  __miso = 0;
  __sclk = 0;
  __pin_mapping = 0;

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

#if AVR_FLAG

  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  if (_sclk != -1) {
    // define pin modes
    pinMode(_sclk, OUTPUT);
    digitalWrite(_sclk, LOW);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
  } else {
    // start and configure hardware SPI
    SPI.begin();
  }

  for (uint8_t i = 0; i < 16; i++) {
    // readRegister8(i);
  }

  setWires(wires);
  enableBias(false);
  autoConvert(false);
  clearFault();

  // Serial.print("config: ");
  // Serial.println(readRegister8(MAX31865_CONFIG_REG), HEX);
  return true;

#else  // AVR_FLAG

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

   for (uint8_t i = 0; i < 16; i++) {
    // readRegister8(i);
  }

  #ifdef MAX31865_USE_AUTO_MODE
    setFlags(MAX31865_CONFIG_FAULTSTAT | MAX31865_CONFIG_MODEAUTO);
  #else
    setFlags(MAX31865_CONFIG_FAULTSTAT);

    _lastStamp = 0;
    _lastStep = 0;
  #endif

  _lastRead = 0;
  _lastReadStamp = 0;

  #if HAS_STM32_DEBUG
    //Serial.print("config: ");
    //Serial.println(readRegister8(MAX31865_CONFIG_REG), HEX);
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

#endif //end AVR_FLAG

}

/**************************************************************************/
/*!
    @brief Read the raw 8-bit FAULTSTAT register
    @return The raw unsigned 8-bit FAULT status register
*/
/**************************************************************************/
uint8_t Adafruit_MAX31865::readFault(void) {

  _lastRead = 0xFFFF; // readFault is called when THERMOCOUPLE_MAX_ERRORS is reached, so set lastRead to a killing value
  return _lastFault; // fault is read and kept right after occurring
}

/**************************************************************************/
/*!
    @brief Clear all faults in FAULTSTAT
*/
/**************************************************************************/
void Adafruit_MAX31865::clearFault(void) {
  setFlags(MAX31865_CONFIG_FAULTSTAT);
}

/**************************************************************************/
/*!
    @brief Enable the bias voltage on the RTD sensor
    @param b If true bias is enabled, else disabled
*/
/**************************************************************************/
void Adafruit_MAX31865::enableBias(bool b) {
  setFlags(b ? MAX31865_CONFIG_BIAS : MAX31865_DEFAULT_CONFIG);
}

void Adafruit_MAX31865::setFlags(uint8_t flags) {
  writeRegister8(MAX31865_CONFIG_REG, MAX31865_DEFAULT_CONFIG | flags);
}

void Adafruit_MAX31865::resetFlags() {
  writeRegister8(MAX31865_CONFIG_REG, MAX31865_DEFAULT_CONFIG);
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

  Rt = _lastRead;

  Rt /= 65536.0f;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4.0 * RTD_B);
  Z3 = (4.0f * RTD_B) / RTDnominal;
  Z4 = 2.0f * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0.0f)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100.0f; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02f;
  temp += 2.2228f * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3f * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6f * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8f * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10f * rpoly;

  return temp;
}

/**************************************************************************/
/*!
    @brief Read the raw 16-bit value from the RTD_REG in one shot mode
    @return The raw unsigned 16-bit value, NOT temperature!
*/
/**************************************************************************/
uint16_t Adafruit_MAX31865::readRTD(void) {
  return  readRTD_with_Fault() >> 1;
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

  rtd = readRTD();

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

#ifndef MAX31865_USE_AUTO_MODE

  switch (_lastStep) {
  case 0:
    setFlags(MAX31865_CONFIG_BIAS);

    _lastStamp = millis();
    _lastStep = 1;
    break;

  case 1:
    if (millis() - _lastStamp < 10)
      return _lastRead;

    setFlags(MAX31865_CONFIG_BIAS | MAX31865_CONFIG_1SHOT);

    _lastStamp = millis();
    _lastStep = 2;
    break;

  case 2:
    if (millis() - _lastStamp < 65)
      return _lastRead;
#endif

    _rtd = readRegister16(MAX31865_RTDMSB_REG);

    if (_rtd & 1) {
      _lastFault = readRegister8(MAX31865_FAULTSTAT_REG); // keep the fault in a variable and reset flag
      clearFault();

#if HAS_STM32_DEBUG || HAS_LPC1768_DEBUG
      SERIAL_ECHOLNPAIR("MAX31865 read fault: ", _rtd);
#endif
    }
#ifdef MAX31865_USE_READ_ERROR_DETECTION
    else if (detectSpike()) { // not reported as error by the sensor
      _lastFault = 0x01;
      _rtd |= 1; // make it an error

#if HAS_STM32_DEBUG || HAS_LPC1768_DEBUG
      SERIAL_ECHOLNPAIR("MAX31865 read error: ", _rtd);
#endif
    }
#endif
    else {
      _lastRead = _rtd;
      _lastReadStamp = millis();

    }

#ifndef MAX31865_USE_AUTO_MODE
    resetFlags();

    _lastStep = 0;

    break;
  }
#endif

#if HAS_STM32_DEBUG || HAS_LPC1768_DEBUG
  #ifndef MAX31865_USE_AUTO_MODE
    if (_lastStep == 0) {
  #endif
      uint16_t rtd_MSB = _rtd >> 8;
      uint16_t rtd_LSB = _rtd & 0x00FF;

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
  #else
    SERIAL_ECHOLNPAIR("MAX31865 MSB: ", rtd_MSB, " LSB: ", rtd_LSB);
  #endif

  #ifndef MAX31865_USE_AUTO_MODE
    }
  #endif

#endif

  return _rtd;
}

bool Adafruit_MAX31865::detectSpike() {
  return abs(_lastRead - _rtd) > 500 && millis() - _lastReadStamp < 1000; // if two readings within a second differ too much (~20°C), consider it a read error.
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
  #if AVR_FLAG

    addr &= 0x7F; // make sure top bit is not set

    if (_sclk == -1)
      SPI.beginTransaction(max31865_spisettings);
    else
      digitalWrite(_sclk, LOW);

    digitalWrite(_cs, LOW);

    spixfer(addr); //ga4

    // Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
    while (n--) {
      buffer[0] = spixfer(0xFF);  //ga4
      // Serial.print(" 0x"); Serial.print(buffer[0], HEX);
      buffer++;
    }
    // Serial.println();

    if (_sclk == -1)
      SPI.endTransaction();

    digitalWrite(_cs, HIGH);

  #else //AVR_FLAG

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

  #endif  //end AVR_FLAG
}

void Adafruit_MAX31865::writeRegister8(uint8_t addr, uint8_t data) {
  #if AVR_FLAG

    if (_sclk == -1)
      SPI.beginTransaction(max31865_spisettings);
    else
      digitalWrite(_sclk, LOW);

    digitalWrite(_cs, LOW);

    spixfer(addr | 0x80); // make sure top bit is set
    spixfer(data);

    // Serial.print("$"); Serial.print(addr, HEX); Serial.print(" = 0x");
    // Serial.println(data, HEX);

    if (_sclk == -1)
      SPI.endTransaction();

    digitalWrite(_cs, HIGH);

  #else //AVR_FLAG

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

  #endif //end AVR_FLAG
}

uint8_t Adafruit_MAX31865::spixfer(uint8_t x) {
  #if AVR_FLAG

    if (_sclk == -1)
      return SPI.transfer(x);

    // software spi
    // Serial.println("Software SPI");
    uint8_t reply = 0;

    for (int i = 7; i >= 0; i--) {
      reply <<= 1;
      digitalWrite(_sclk, HIGH);
      digitalWrite(_mosi, x & (1 << i));
      digitalWrite(_sclk, LOW);
      if (digitalRead(_miso))
        reply |= 1;
    }

    return reply;

  #else //AVR_FLAG

    if (_sclk == -1 || __sclk == -1UL)
      return SPI.transfer(x);

    // software spi
    // Serial.println("Software SPI");
    uint8_t reply = 0;

    if (!__pin_mapping) {
      for (int i = 7; i >= 0; i--) {
        reply <<= 1;
        digitalWrite(_sclk, HIGH);
        digitalWrite(_mosi, x & (1 << i));
        digitalWrite(_sclk, LOW);
        if (digitalRead(_miso))
          reply |= 1;
      }
    }
    else {
      for (int i = 7; i >= 0; i--) {
        reply <<= 1;
        digitalWrite(__sclk, HIGH);
        digitalWrite(__mosi, x & (1 << i));
        digitalWrite(__sclk, LOW);
        if (digitalRead(__miso))
          reply |= 1;
      }
    }

    return reply;

  #endif //end AVR_FLAG
}
