#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "driver/i2c.h"
#include "driver/gpio.h"

#ifdef __cplusplus   // everything below is only visible to C++ code

// Debug macro (no-op by default)
#define DBG(...)


class DFRobot_TMF8x01 {
public:
  // Constants
  static constexpr uint8_t  SENSOR_MTF8x01_CALIBRATION_SIZE = 14;
  static constexpr uint16_t MODEL_TMF8701 = 0x5e10;

  // Status helpers
  bool isCpuReady()     { return getCPUState() == 0x41; }
  bool isStandby()      { return getCPUState() == 0x00; }
  bool isApp0()         { return getAppId()     == 0xC0; }
  bool isBootloader()   { return getAppId()     == 0x80; }
  bool isDataReadyFlag()  { return getRegContents() == 0x55; }

  // Bit / index definitions
  static constexpr uint8_t CMDSET_INDEX_CMD7       = 0;
  static constexpr uint8_t CMDSET_BIT_CALIB        = 0;
  static constexpr uint8_t CMDSET_BIT_ALGO         = 1;

  static constexpr uint8_t CMDSET_INDEX_CMD6       = 1;
  static constexpr uint8_t CMDSET_BIT_PROXIMITY    = 0;
  static constexpr uint8_t CMDSET_BIT_DISTANCE     = 1;
  static constexpr uint8_t CMDSET_BIT_INT          = 4;
  static constexpr uint8_t CMDSET_BIT_COMBINE      = 5;

  typedef enum {
      ePIN0 = 0,
      ePIN1,
      ePINTotal
  } ePin_t;

  typedef enum {
      ePinInput = 0,
      ePinInputLow = 1,
      ePinInputHigh = 2,
      ePinOutputVCSEL = 3,
      ePinOutputLow = 4,
      ePinOutputHigh = 5,
  } ePinControl_t;

  typedef enum {
      eModeNoCalib = 0,
      eModeCalib = 1,
      eModeCalibAndAlgoState = 3
  } eCalibModeConfig_t;

  typedef union {
    struct {
      uint8_t  pon      : 1;
      uint8_t  reserve  : 5;
      uint8_t  cpuReady : 1;
      uint8_t  cpuReset : 1;
    };
    uint8_t value;
  } __attribute__ ((packed)) eEnableReg_t;

  typedef struct {
      uint8_t status;
      uint8_t regContents;
      uint8_t tid;
      uint8_t resultNumber;
      struct {
          uint8_t reliability : 6;
          uint8_t meastatus   : 2;
      } resultInfo;
      uint8_t disL;
      uint8_t disH;
      uint8_t sysclock0;
      uint8_t sysclock1;
      uint8_t sysclock2;
      uint8_t sysclock3;
  } sResult_t;

  /**
   * @brief Constructor for ESP-IDF version
   * @param enPin   GPIO used for EN (or GPIO_NUM_NC if unused)
   * @param intPin  GPIO used for INT (or GPIO_NUM_NC if unused)
   * @param i2cPort I2C port (e.g. I2C_NUM_0)
   * @param i2cAddr 7-bit I2C address (default 0x41)
   */
  DFRobot_TMF8x01(gpio_num_t enPin,
                  gpio_num_t intPin,
                  i2c_port_t i2cPort,
                  uint8_t i2cAddr = 0x41);

  virtual ~DFRobot_TMF8x01();

  /**
   * @brief Initialize sensor (assumes I2C already configured in ESP-IDF)
   * @return 0 on success, -1 on failure
   */
  int begin();

  void     sleep();
  bool     wakeup();

  uint32_t    getUniqueID();
  void getSensorModel(char *out, size_t out_len);
  void getSoftwareVersion(char *out, size_t out_len);

  bool getCalibrationData(uint8_t *data,
                          uint8_t len = SENSOR_MTF8x01_CALIBRATION_SIZE);
  bool setCalibrationData(uint8_t *data,
                          uint8_t len = SENSOR_MTF8x01_CALIBRATION_SIZE);

  // Public wrapper to control calibration mode from C API
  bool setCalibrationMode(eCalibModeConfig_t mode) {
      return setCaibrationMode(mode);
  }


  void     stopMeasurement();
  bool     isDataReady();
  uint16_t getDistance_mm();

  void enableIntPin();
  void disableIntPin();

  bool powerOn();
  bool powerDown();

  uint8_t getI2CAddress() const { return _addr; }

  void   pinConfig(ePin_t pin, ePinControl_t config);
  int8_t getJunctionTemperature_C();

protected:
  virtual bool downloadRamPatch() = 0;

  uint8_t  getCalibrationMode();
  bool     loadApplication();
  bool     loadBootloader();
  bool     waitForApplication();
  bool     waitForBootloader();
  bool     waitForCpuReady();
  void     modifyCmdSet(uint8_t index, uint8_t bit, bool val);
  uint8_t  getCPUState();
  uint8_t  getAppId();
  uint8_t  getRegContents();
  bool     checkStatusRegister(uint8_t status);
  bool     isI2CAddress(uint8_t addr);
  uint8_t  calChecksum(uint8_t *data, uint8_t len);
  bool     readStatusACK();

  void writeReg(uint8_t reg, const void* pBuf, size_t size);
  uint8_t readReg(uint8_t reg, void* pBuf, size_t size);

  void gpioInit();
  bool setCaibrationMode(eCalibModeConfig_t cailbMode);

  uint8_t   _measureCmdSet[9];
  uint8_t   _calibData[14];
  uint8_t   _algoStateData[11];

  i2c_port_t _i2cPort;

private:
  gpio_num_t _en;
  gpio_num_t _intPin;
  bool       _initialize;
  uint8_t    _count;
  uint8_t    _config;
  double     _timestamp;
  bool       _measureCmdFlag;
  uint8_t    _addr;
  uint32_t   _hostTime[5];
  uint32_t   _MoudleTime[5];
  sResult_t  _result;
};


// ---------------- TMF8701 concrete class ----------------

class DFRobot_TMF8701 : public DFRobot_TMF8x01 {
public:
  typedef enum{
      ePROXIMITY = 0,
      eDISTANCE  = 1,
      eCOMBINE   = 2
  } eDistaceMode_t;

  DFRobot_TMF8701(gpio_num_t enPin  = GPIO_NUM_NC,
                  gpio_num_t intPin = GPIO_NUM_NC,
                  i2c_port_t i2cPort = I2C_NUM_0,
                  uint8_t i2cAddr   = 0x41);

  bool startMeasurement(eCalibModeConfig_t cailbMode = eModeCalib,
                        eDistaceMode_t      disMode   = eCOMBINE);

protected:
  bool downloadRamPatch() override;
};

#endif  // __cplusplus
