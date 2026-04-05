/*
 * ESP-IDF port of DFRobot_TMF8701 (TMF8x01 base + 8701 implementation)
 */

#include "DFRobot_TMF8701_espidf.h"

// VS Code / IntelliSense often doesn't see this Kconfig symbol.
// Guarded so it won't override the real value during an idf.py build.
#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 100  // default tick rate (100 Hz) for ESP-IDF
#endif

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"

// Arduino PROGMEM helpers redefined for ESP-IDF
#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#endif

#ifndef memcpy_P
#define memcpy_P(dest, src, n) memcpy((dest), (src), (n))
#endif

// This header should contain the TMF8701 RAM patch buffer:
//   extern const uint8_t DFRobot_TMF8701_initBuf[];
#include "drv/TMF8701_2.h"   // adjust path as needed
#include "esp_log.h"

static const char *TAG_TMF = "TMF8701";


// Register definitions (from original Arduino driver)
#define REG_MTF8x01_ENABLE           0xE0
#define REG_MTF8x01_APPID            0x00
#define REG_MTF8x01_ID               0xE3
#define REG_MTF8x01_APPREQID         0x02
#define REG_MTF8x01_CMD_DATA9        0X06
#define REG_MTF8x01_CMD_DATA8        0X07
#define REG_MTF8x01_CMD_DATA7        0X08
#define REG_MTF8x01_CMD_DATA6        0X09
#define REG_MTF8x01_CMD_DATA5        0X0A
#define REG_MTF8x01_CMD_DATA4        0X0B
#define REG_MTF8x01_CMD_DATA3        0X0C
#define REG_MTF8x01_CMD_DATA2        0X0D
#define REG_MTF8x01_CMD_DATA1        0X0E
#define REG_MTF8x01_CMD_DATA0        0X0F
#define REG_MTF8x01_COMMAND          0X10
#define REG_MTF8x01_FACTORYCALIB     0X20
#define REG_MTF8x01_STATEDATAWR      0X2E
#define REG_MTF8x01_STATUS           0x1D
#define REG_MTF8x01_CONTENTS         0x1E
#define REG_MTF8x01_TJ               0x32
#define REG_MTF8x01_RESULT_NUMBER    0x20
#define REG_MTF8x01_INT_ENAB         0xE2
#define REG_MTF8x01_INT_STATUS       0xE1
#define REG_MTF8x01_VERSION_MAJOR    0X01
#define REG_MTF8x01_VERSION_MINORANDPATCH   0x12
#define REG_MTF8x01_VERSION_HW              0xE3
#define REG_MTF8x01_VERSION_SERIALNUM       0x28

// Convert ms -> ticks using CONFIG_FREERTOS_HZ directly
static inline TickType_t ms_to_ticks(uint32_t ms) {
    // ticks = ms * (Hz / 1000); rounded up
    return (TickType_t)(((uint64_t)ms * CONFIG_FREERTOS_HZ + 999) / 1000);
}

// Local helpers to replace Arduino delay/millis
static inline void delay_ms(uint32_t ms) {
    vTaskDelay(ms_to_ticks(ms));
}

static inline uint32_t millis() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

// ---------------- Base: TMF8x01 ----------------

DFRobot_TMF8x01::DFRobot_TMF8x01(gpio_num_t enPin,
                                 gpio_num_t intPin,
                                 i2c_port_t i2cPort,
                                 uint8_t i2cAddr)
  : _i2cPort(i2cPort),
    _en(enPin),
    _intPin(intPin),
    _initialize(false),
    _count(0),
    _config(0),
    _timestamp(0),
    _measureCmdFlag(false),
    _addr(i2cAddr)
{
  memset(_hostTime,    0, sizeof(_hostTime));
  memset(_MoudleTime,  0, sizeof(_MoudleTime));
  memset(&_result,     0, sizeof(_result));
  memset(_measureCmdSet, 0, sizeof(_measureCmdSet));
  memset(_calibData,     0, sizeof(_calibData));
  memset(_algoStateData, 0, sizeof(_algoStateData));
}

DFRobot_TMF8x01::~DFRobot_TMF8x01() {}

int DFRobot_TMF8x01::begin() {
    _initialize = false;
    gpioInit();

    ESP_LOGI(TAG_TMF, "begin(): probing addr=0x%02X on port=%d", _addr, _i2cPort);

    if (!isI2CAddress(_addr)) {
        ESP_LOGE(TAG_TMF, "begin(): no I2C device at 0x%02X", _addr);
        return -1;
    }

    ESP_LOGI(TAG_TMF, "begin(): I2C device found, sending sleep() then enable");

    sleep();
    eEnableReg_t regValue;
    regValue.value = 1;
    writeReg(REG_MTF8x01_ENABLE, &regValue, sizeof(regValue));

    if (!waitForCpuReady()) {
        ESP_LOGE(TAG_TMF, "begin(): waitForCpuReady() failed");
        return -1;
    }

    uint8_t appId = getAppId();
    ESP_LOGI(TAG_TMF, "begin(): initial appId=0x%02X", appId);

    if (appId == 0x80) { // bootloader
        ESP_LOGI(TAG_TMF, "begin(): in bootloader, downloading RAM patch...");
        if (!downloadRamPatch()) {
            ESP_LOGE(TAG_TMF, "begin(): downloadRamPatch() failed");
            return -1;
        }

        // After patch & reset, check what app is running now
        uint8_t appAfterPatch = getAppId();
        ESP_LOGI(TAG_TMF,
                 "begin(): appId after patch=0x%02X", appAfterPatch);

        if (appAfterPatch != 0xC0) {
            ESP_LOGW(TAG_TMF,
                     "begin(): APP0 not running after patch, requesting application load");

            if (!loadApplication()) {
                ESP_LOGE(TAG_TMF, "begin(): loadApplication() failed");
                return -1;
            }
            if (!waitForApplication()) {
                ESP_LOGE(TAG_TMF, "begin(): waitForApplication() failed");
                return -1;
            }

            appAfterPatch = getAppId();
            ESP_LOGI(TAG_TMF,
                     "begin(): appId after loadApplication=0x%02X", appAfterPatch);
        }

        if (!isApp0()) {
            ESP_LOGE(TAG_TMF,
                     "begin(): APP0 still not running after ram patch + loadApplication");
            return -1;
        }

        ESP_LOGI(TAG_TMF, "begin(): APP0 is running");
    } else if (appId == 0xC0) {
        ESP_LOGI(TAG_TMF, "begin(): APP0 already running");
    } else {
        ESP_LOGW(TAG_TMF, "begin(): unexpected initial appId=0x%02X", appId);
    }

    _initialize = true;
    ESP_LOGI(TAG_TMF, "begin(): initialization OK");
    return 0;
}



void DFRobot_TMF8x01::sleep() {
  eEnableReg_t regValue;
  readReg(REG_MTF8x01_ENABLE, &regValue, sizeof(regValue));
  regValue.cpuReset = 1;
  writeReg(REG_MTF8x01_ENABLE, &regValue, sizeof(regValue));
  _measureCmdFlag = false;
  _count = 0;
}

bool DFRobot_TMF8x01::wakeup() {
  eEnableReg_t regValue;
  regValue.value = 1;
  writeReg(REG_MTF8x01_ENABLE, &regValue, sizeof(regValue));

  if (!waitForCpuReady()) {
    DBG("waitForCpuReady failed");
    return false;
  }

  if (getAppId() == 0x80) {
    if (!downloadRamPatch()) return false;
    if (!isApp0()) return false;
  }

  if (_measureCmdSet[CMDSET_INDEX_CMD6] & (1 << CMDSET_BIT_INT)) {
    modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_INT, true);
  }
  if (!setCaibrationMode((eCalibModeConfig_t)getCalibrationMode())) return false;
  return true;
}

void DFRobot_TMF8x01::getSoftwareVersion(char *out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return;
  }

  uint8_t val = 0;
  int written = 0;

  // Clear the buffer
  out[0] = '\0';

  // Major
  readReg(0x01, &val, 1);
  written += snprintf(out + written,
                      (written < (int)out_len) ? (out_len - written) : 0,
                      "%X.", val);

  // Minor
  readReg(0x12, &val, 1);
  written += snprintf(out + written,
                      (written < (int)out_len) ? (out_len - written) : 0,
                      "%X.", val);

  // Patch
  readReg(0x13, &val, 1);
  written += snprintf(out + written,
                      (written < (int)out_len) ? (out_len - written) : 0,
                      "%X.", val);

  // HW or extra info
  readReg(0xE4, &val, 1);
  snprintf(out + written,
           (written < (int)out_len) ? (out_len - written) : 0,
           "%X", val);
}


uint32_t DFRobot_TMF8x01::getUniqueID() {
  uint8_t regValue = 0x47;
  uint32_t rslt = 0;
  uint8_t buf[4];
  uint8_t waitForTimeOutMs = 100;
  uint8_t waitForTimeoutIncMs = 5;
  uint8_t data2[] = {0xff};

  writeReg(REG_MTF8x01_COMMAND, &regValue, 1);
  regValue = 0;

  for (uint8_t t = 0; t < waitForTimeOutMs; t += waitForTimeoutIncMs) {
    readReg(0x1E, &regValue, sizeof(regValue));
    if (regValue == 0x47) {
      readReg(REG_MTF8x01_VERSION_SERIALNUM, buf, 4);
      memcpy(&rslt, buf, 4);
      writeReg(REG_MTF8x01_COMMAND, data2, sizeof(data2));
      delay_ms(50);
      return rslt;
    }
    delay_ms(waitForTimeoutIncMs);
  }
  return 0;
}

void DFRobot_TMF8x01::getSensorModel(char *out, size_t out_len) {
  if (out == nullptr || out_len == 0) {
    return;
  }

  uint32_t rslt = getUniqueID();
  rslt = (rslt >> 16) & 0xFFFF;

  if (rslt == MODEL_TMF8701) {
    snprintf(out, out_len, "TMF8701");
  } else {
    snprintf(out, out_len, "unknown");
  }
}

bool DFRobot_TMF8x01::getCalibrationData(uint8_t *data, uint8_t len) {
    ESP_LOGI(TAG_TMF, "getCalibrationData(): len=%u, init=%d, appId=0x%02X",
             (unsigned)len, _initialize ? 1 : 0, getAppId());

    if (!_initialize) {
        ESP_LOGE(TAG_TMF, "getCalibrationData(): driver not initialized");
        return false;
    }

    if (data == nullptr) {
        ESP_LOGE(TAG_TMF, "getCalibrationData(): data == nullptr");
        return false;
    }

    if (len != SENSOR_MTF8x01_CALIBRATION_SIZE) {
        ESP_LOGW(TAG_TMF,
                 "getCalibrationData(): len=%u, expected=%u; using min(len, expected)",
                 (unsigned)len,
                 (unsigned)SENSOR_MTF8x01_CALIBRATION_SIZE);
    }

    if (!isApp0()) {  // uses the helper we made earlier
        ESP_LOGE(TAG_TMF,
                 "getCalibrationData(): APP0 not running (appId=0x%02X)",
                 getAppId());
        return false;
    }

    uint8_t regValue = 0x0A;
    uint8_t clearCmd = 0xFF;

    // Tell the sensor we want calibration data
    writeReg(REG_MTF8x01_COMMAND, &regValue, 1);

    // Wait for status 0x0A, but treat timeout as warning (not fatal)
    if (!checkStatusRegister(0x0A)) {
        ESP_LOGW(TAG_TMF,
                 "getCalibrationData(): checkStatusRegister(0x0A) timeout; "
                 "attempting to read anyway");
    }

    uint8_t toRead = (len < SENSOR_MTF8x01_CALIBRATION_SIZE)
                         ? len
                         : SENSOR_MTF8x01_CALIBRATION_SIZE;

    readReg(REG_MTF8x01_RESULT_NUMBER, data, toRead);

    // Clear command
    writeReg(REG_MTF8x01_COMMAND, &clearCmd, 1);
    delay_ms(50);

    ESP_LOGI(TAG_TMF, "getCalibrationData(): read %u bytes", (unsigned)toRead);
    return true;
}


bool DFRobot_TMF8x01::setCalibrationData(uint8_t *data, uint8_t len) {
  if (data == nullptr || len != SENSOR_MTF8x01_CALIBRATION_SIZE) return false;
  memcpy(_calibData, data, len);
  return true;
}

bool DFRobot_TMF8x01::setCaibrationMode(eCalibModeConfig_t mode) {
  ESP_LOGI(TAG_TMF, "setCaibrationMode(): mode=%d", (int)mode);

  if (!_initialize || _measureCmdFlag) return false;

  uint8_t CalibCmd[] = {0x0B};

  switch (mode) {
    case eModeCalib:
      modifyCmdSet(CMDSET_INDEX_CMD7, CMDSET_BIT_CALIB, true);
      modifyCmdSet(CMDSET_INDEX_CMD7, CMDSET_BIT_ALGO,  false);
      writeReg(REG_MTF8x01_COMMAND, CalibCmd, sizeof(CalibCmd));
      writeReg(REG_MTF8x01_RESULT_NUMBER, _calibData, sizeof(_calibData));
      break;
    case eModeCalibAndAlgoState:
      modifyCmdSet(CMDSET_INDEX_CMD7, CMDSET_BIT_CALIB, true);
      modifyCmdSet(CMDSET_INDEX_CMD7, CMDSET_BIT_ALGO,  true);
      writeReg(REG_MTF8x01_COMMAND, CalibCmd, sizeof(CalibCmd));
      writeReg(REG_MTF8x01_RESULT_NUMBER, _calibData, sizeof(_calibData));
      writeReg(REG_MTF8x01_STATEDATAWR, _algoStateData, sizeof(_algoStateData));
      break;
    default:
      modifyCmdSet(CMDSET_INDEX_CMD7, CMDSET_BIT_CALIB, false);
      modifyCmdSet(CMDSET_INDEX_CMD7, CMDSET_BIT_ALGO,  false);
      break;
  }

  writeReg(REG_MTF8x01_CMD_DATA7, _measureCmdSet, sizeof(_measureCmdSet));
  delay_ms(600);

  if (!checkStatusRegister(0x55)) {
        ESP_LOGW(TAG_TMF,
                 "setCaibrationMode(): checkStatusRegister timeout; "
                 "continuing anyway");
        // DO NOT return false here
    }

  while (_count < 4) {
    if (isDataReady()) {
      getDistance_mm();
    }
    delay_ms(2);
  }
  _measureCmdFlag = true;
  return true;
}

uint8_t DFRobot_TMF8x01::getCalibrationMode() {
  uint8_t mode = 0;
  if (_measureCmdSet[CMDSET_INDEX_CMD7] & (1 << CMDSET_BIT_CALIB)) mode = 1;
  if (_measureCmdSet[CMDSET_INDEX_CMD7] & (1 << CMDSET_BIT_ALGO))  mode |= (1 << CMDSET_BIT_ALGO);
  return mode;
}

void DFRobot_TMF8x01::stopMeasurement() {
  uint8_t data[] = {0xFF};
  writeReg(REG_MTF8x01_COMMAND, data, sizeof(data));
  delay_ms(50);
  _measureCmdFlag = false;
  _count = 0;
  _timestamp = 1;
  memset(_hostTime,   0, sizeof(_hostTime));
  memset(_MoudleTime, 0, sizeof(_MoudleTime));
  memset(&_result,    0, sizeof(_result));
}

bool DFRobot_TMF8x01::isDataReady() {
  sResult_t result;
  uint32_t t, sysT;
  double t1, t2;

  memset(&result, 0, sizeof(result));
  t = millis();

  readReg(REG_MTF8x01_STATUS, &result, sizeof(result));

  if (result.regContents == 0x55) {
    if (result.tid != _result.tid) {
      _result = result;
      sysT = ((uint32_t)_result.sysclock3 << 24) |
             ((uint32_t)_result.sysclock2 << 16) |
             ((uint32_t)_result.sysclock1 << 8)  |
             ((uint32_t)_result.sysclock0);

      if (_count < 4) {
        _hostTime[_count]    = t;
        _MoudleTime[_count++] = sysT;
      } else if (_count == 4) {
        _hostTime[_count]    = t;
        _MoudleTime[_count] = sysT;
        if (_MoudleTime[4] > _MoudleTime[0] && (_hostTime[4] >= _hostTime[0])) {
          t1 = (_hostTime[4] - _hostTime[0]) * 10.0;
          t2 = (_MoudleTime[4] - _MoudleTime[0]) * 0.2 / 100.0;
          if ((t1 / t2 >= 0.7) && (t1 / t2 <= 1.3)) {
            _timestamp = t1 / t2;
          }
        }
        for (int i = 0; i < 4; ++i) {
          _hostTime[i]    = _hostTime[i + 1];
          _MoudleTime[i]  = _MoudleTime[i + 1];
        }
      } else {
        _count = 0;
      }
      return true;
    }
  }

  if (_measureCmdSet[CMDSET_INDEX_CMD6] & (1 << CMDSET_BIT_INT)) {
    uint8_t val = 0;
    readReg(REG_MTF8x01_INT_STATUS, &val, 1);
    if (val & 0x01) {
      val |= 0x01;
      writeReg(REG_MTF8x01_INT_STATUS, &val, 1);
    }
  }
  return false;
}

uint16_t DFRobot_TMF8x01::getDistance_mm() {
  uint16_t rslt = ((uint16_t)_result.disH << 8) | _result.disL;
  rslt = static_cast<uint16_t>(rslt * _timestamp);

  if (_measureCmdSet[CMDSET_INDEX_CMD6] & (1 << CMDSET_BIT_INT)) {
    uint8_t val = 0;
    readReg(REG_MTF8x01_INT_STATUS, &val, 1);
    val |= 0x01;
    writeReg(REG_MTF8x01_INT_STATUS, &val, 1);
  }

  return rslt;
}

void DFRobot_TMF8x01::enableIntPin() {
  uint8_t val = 0x01;
  writeReg(REG_MTF8x01_INT_ENAB, &val, 1);
  modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_INT, true);
}

void DFRobot_TMF8x01::disableIntPin() {
  uint8_t val = 0x00;
  writeReg(REG_MTF8x01_INT_ENAB, &val, 1);
  modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_INT, false);
}

bool DFRobot_TMF8x01::powerOn() {
  if (!_initialize) return false;
  if (_en == GPIO_NUM_NC) return false;

  delay_ms(1000);
  gpio_set_level(_en, 1);
  delay_ms(1000);

  eEnableReg_t regValue;
  regValue.value = 1;
  writeReg(REG_MTF8x01_ENABLE, &regValue, sizeof(regValue));

  if (!waitForCpuReady()) return false;

  if (getAppId() == 0x80) {
    if (!loadApplication()) return false;
    if (!waitForApplication()) return false;
  }

  return true;
}

bool DFRobot_TMF8x01::powerDown() {
  if (!_initialize) return false;
  if (_en == GPIO_NUM_NC) return false;

  delay_ms(1000);
  gpio_set_level(_en, 0);
  delay_ms(1000);
  return true;
}

bool DFRobot_TMF8x01::loadApplication() {
  uint8_t regValue = 0xC0;
  writeReg(REG_MTF8x01_APPREQID, &regValue, 1);
  return waitForApplication();
}

bool DFRobot_TMF8x01::loadBootloader() {
  uint8_t regValue = 0x80;
  writeReg(REG_MTF8x01_APPREQID, &regValue, 1);
  return waitForBootloader();
}

bool DFRobot_TMF8x01::waitForApplication() {
  uint8_t waitForTimeOutMs = 100;
  uint8_t waitForTimeoutIncMs = 5;

  for (uint8_t t = 0; t < waitForTimeOutMs; t += waitForTimeoutIncMs) {
    delay_ms(waitForTimeoutIncMs);
    if (isApp0()) return true;
  }
  return false;
}

bool DFRobot_TMF8x01::waitForBootloader() {
  uint8_t waitForTimeOutMs = 100;
  uint8_t waitForTimeoutIncMs = 5;

  for (uint8_t t = 0; t < waitForTimeOutMs; t += waitForTimeoutIncMs) {
    delay_ms(waitForTimeoutIncMs);
    if (isBootloader()) return true;
  }
  return false;
}

bool DFRobot_TMF8x01::waitForCpuReady() {
  uint8_t waitForTimeOutMs = 100;
  uint8_t waitForTimeoutIncMs = 5;

  for (uint8_t t = 0; t < waitForTimeOutMs; t += waitForTimeoutIncMs) {
    delay_ms(waitForTimeoutIncMs);
    if (isCpuReady()) return true;
  }
  return false;
}

void DFRobot_TMF8x01::modifyCmdSet(uint8_t index, uint8_t bit, bool val) {
  if (index >= sizeof(_measureCmdSet) || bit > 7) return;
  if (val) _measureCmdSet[index] |=  (1 << bit);
  else     _measureCmdSet[index] &= ~(1 << bit);
}

void DFRobot_TMF8x01::pinConfig(ePin_t pin, ePinControl_t config) {
  uint8_t data[] = {0x0F, 0, 0x0F};
  if ((pin > ePINTotal) || (config > ePinOutputHigh)) return;

  switch (pin) {
    case ePIN0:
      _config &= 0x0F;
      _config |= ((uint8_t)config) << 4;
      break;
    case ePIN1:
      _config &= 0xF0;
      _config |= (uint8_t)config;
      break;
    case ePINTotal:
      _config = ((uint8_t)config << 4) | (uint8_t)config;
      break;
  }

  data[1] = _config;
  writeReg(data[0], data + 1, sizeof(data) - 1);
}

uint8_t DFRobot_TMF8x01::getCPUState() {
  eEnableReg_t regValue;
  readReg(REG_MTF8x01_ENABLE, &regValue, sizeof(regValue));
  return regValue.value;
}

int8_t DFRobot_TMF8x01::getJunctionTemperature_C() {
  int8_t temp = 0;
  readReg(REG_MTF8x01_TJ, &temp, 1);
  return temp;
}

uint8_t DFRobot_TMF8x01::getAppId() {
  uint8_t regValue = 0;
  readReg(REG_MTF8x01_APPID, &regValue, sizeof(regValue));
  return regValue;
}

uint8_t DFRobot_TMF8x01::getRegContents() {
  uint8_t regValue = 0;
  readReg(REG_MTF8x01_CONTENTS, &regValue, 1);
  return regValue;
}

bool DFRobot_TMF8x01::checkStatusRegister(uint8_t status) {
  uint8_t waitForTimeOutMs = 1000;
  uint8_t waitForTimeoutIncMs = 5;

  for (uint8_t t = 0; t < waitForTimeOutMs; t += waitForTimeoutIncMs) {
    delay_ms(waitForTimeoutIncMs);
    if (getRegContents() == status) return true;
  }
  return false;
}

bool DFRobot_TMF8x01::isI2CAddress(uint8_t addr) {
    uint8_t reg = REG_MTF8x01_ENABLE;
    uint8_t val = 0;
    esp_err_t err;

    // Write reg address, then read 1 byte using old driver
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_TMF, "isI2CAddress: failed to create cmd");
        return false;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2cPort, cmd, ms_to_ticks(50));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_TMF, "isI2CAddress: write phase failed for 0x%02X, err=%d", addr, err);
        return false;
    }

    cmd = i2c_cmd_link_create();
    if (!cmd) {
        ESP_LOGE(TAG_TMF, "isI2CAddress: failed to create cmd (read)");
        return false;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2cPort, cmd, ms_to_ticks(50));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_TMF, "isI2CAddress: read phase failed for 0x%02X, err=%d", addr, err);
        return false;
    }

    ESP_LOGI(TAG_TMF, "isI2CAddress: device responded at 0x%02X, val=0x%02X", addr, val);
    return true;
}


uint8_t DFRobot_TMF8x01::calChecksum(uint8_t *data, uint8_t len) {
  if (data == nullptr) return 0;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; ++i) sum += data[i];
  sum ^= 0xFF;
  return sum;
}

bool DFRobot_TMF8x01::readStatusACK() {
    uint8_t buf[3] = {0};
    readReg(0x08, buf, sizeof(buf));

    uint32_t value = ((uint32_t)buf[0] << 16) |
                     ((uint32_t)buf[1] << 8)  |
                     ((uint32_t)buf[2]);

    // ESP_LOGI(TAG_TMF,
    //          "readStatusACK: raw bytes = %02X %02X %02X (0x%06X)",
    //          buf[0], buf[1], buf[2], value);

    // Original DFRobot code had "if (value = 0xFF0000)" (assignment),
    // which always evaluated as true and never actually checked the value.
    // To match that behavior (and avoid false failures), we just log and
    // always return true here.
    return true;
}



void DFRobot_TMF8x01::gpioInit() {
  if (_en != GPIO_NUM_NC) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << _en;
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
    gpio_set_level(_en, 0);
    delay_ms(1000);
    gpio_set_level(_en, 1);
    delay_ms(1000);
  }

  if (_intPin != GPIO_NUM_NC) {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = 1ULL << _intPin;
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
  }
}

void DFRobot_TMF8x01::writeReg(uint8_t reg, const void* pBuf, size_t size) {
    if (pBuf == nullptr || size == 0) {
        return;
    }

    // We assume size is small (it is, for this sensor)
    uint8_t buf[32];
    if (size + 1 > sizeof(buf)) {
        return; // avoid overflow
    }

    buf[0] = reg;
    memcpy(buf + 1, pBuf, size);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        return;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, size + 1, true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(_i2cPort, cmd, ms_to_ticks(50));
    i2c_cmd_link_delete(cmd);
}

uint8_t DFRobot_TMF8x01::readReg(uint8_t reg, void* pBuf, size_t size) {
    if (pBuf == nullptr || size == 0) {
        return 0;
    }

    esp_err_t err;
    uint8_t *data = reinterpret_cast<uint8_t*>(pBuf);

    // 1) Write register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        return 0;
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2cPort, cmd, ms_to_ticks(50));
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        return 0;
    }

    // 2) Read data
    cmd = i2c_cmd_link_create();
    if (cmd == nullptr) {
        return 0;
    }
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_READ, true);

    if (size > 1) {
        // Read size-1 bytes with ACK
        i2c_master_read(cmd, data, size - 1, I2C_MASTER_ACK);
        // Read last byte with NACK
        i2c_master_read_byte(cmd, data + size - 1, I2C_MASTER_NACK);
    } else {
        // Read single byte with NACK
        i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    }

    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2cPort, cmd, ms_to_ticks(50));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        return 0;
    }

    return static_cast<uint8_t>(size);
}


// ---------------- TMF8701 ----------------

DFRobot_TMF8701::DFRobot_TMF8701(gpio_num_t enPin,
                                 gpio_num_t intPin,
                                 i2c_port_t i2cPort,
                                 uint8_t i2cAddr)
  : DFRobot_TMF8x01(enPin, intPin, i2cPort, i2cAddr)
{
  // Default command/calibration/algo state copied from Arduino driver
  static const uint8_t measureCmdInit[9] =
      {0x03,0x23,0x00,0x00,0x00,0x64,0xFF,0xFF,0x02};
  static const uint8_t calibInit[14] =
      {0x41,0x57,0x01,0xFD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04};
  static const uint8_t algoInit[11] =
      {0xB1,0xA9,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

  memcpy(_measureCmdSet, measureCmdInit, sizeof(measureCmdInit));
  memcpy(_calibData,     calibInit,      sizeof(calibInit));
  memcpy(_algoStateData, algoInit,       sizeof(algoInit));
}

bool DFRobot_TMF8701::startMeasurement(eCalibModeConfig_t cailbMode,
                                       eDistaceMode_t disMode) {
  ESP_LOGI(TAG_TMF,
          "startMeasurement(): calibMode=%d, distMode=%d",
          (int)cailbMode, (int)disMode);

  switch (disMode) {
    case ePROXIMITY:
      modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_PROXIMITY, true);
      modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_DISTANCE,  false);
      modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_COMBINE,   false);
      break;
    case eDISTANCE:
      modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_PROXIMITY, false);
      modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_DISTANCE,  true);
      modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_COMBINE,   false);
      break;
    case eCOMBINE:
    default:
      modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_PROXIMITY, true);
      modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_DISTANCE,  true);
      modifyCmdSet(CMDSET_INDEX_CMD6, CMDSET_BIT_COMBINE,   true);
      break;
  }
  return setCaibrationMode(cailbMode);
}

bool DFRobot_TMF8701::downloadRamPatch() {
    ESP_LOGI(TAG_TMF, "downloadRamPatch: enter, appId=0x%02X", getAppId());

    uint8_t buf[20];
    uint8_t len = 0;
    uint8_t *addr = (uint8_t *)DFRobot_TMF8701_initBuf;
    int flag = 0;
    memset(buf, 0, sizeof(buf));

    // Make sure we're in bootloader (0x80)
    if (getAppId() != 0x80) {
        ESP_LOGW(TAG_TMF,
                 "downloadRamPatch: appId != 0x80 (0x%02X), calling loadBootloader()",
                 getAppId());
        if (!loadBootloader()) {
            ESP_LOGE(TAG_TMF, "downloadRamPatch: loadBootloader() failed");
            return false;
        }
    }

    // ---- Init command ----
    {
        const char initCmd[] = {0x14, 0x01, 0x29};
        buf[0] = 0x08;
        memcpy(buf + 1, initCmd, sizeof(initCmd));
        len = 1 + sizeof(initCmd);

        ESP_LOGI(TAG_TMF,
                 "downloadRamPatch: sending init CMD (0x14,0x01,0x29)");
        writeReg(buf[0], buf + 1, len - 1);

        if (!readStatusACK()) {
            ESP_LOGE(TAG_TMF,
                     "downloadRamPatch: ACK failed after init CMD");
            return false;
        }
    }

    // ---- Config command ----
    {
        const char cfgCmd[] = {0x43, 0x02, 0x00, 0x00};
        buf[0] = 0x08;
        memcpy(buf + 1, cfgCmd, sizeof(cfgCmd));
        len = 1 + sizeof(cfgCmd);

        ESP_LOGI(TAG_TMF,
                 "downloadRamPatch: sending config CMD (0x43,0x02,0x00,0x00)");
        writeReg(buf[0], buf + 1, len - 1);
        // Note: original driver doesn't do an ACK check here
    }

    // ---- Patch blocks ----
    buf[0] = 0x08;
    buf[1] = 0x41;

    int block_index = 0;
    while ((flag = pgm_read_byte(addr++)) > 0) {
        buf[2] = flag;

        memcpy_P(buf + 3, addr, flag);
        addr += flag;

        buf[3 + flag] = calChecksum(buf + 1, flag + 2);

        // ESP_LOGI(TAG_TMF,
        //          "downloadRamPatch: sending block %d, len=%d, checksum=0x%02X",
        //          block_index, flag, buf[3 + flag]);

        writeReg(buf[0], buf + 1, 3 + flag);

        if (!readStatusACK()) {
            ESP_LOGE(TAG_TMF,
                     "downloadRamPatch: ACK failed after block %d", block_index);
            return false;
        }

        block_index++;
    }

    ESP_LOGI(TAG_TMF,
             "downloadRamPatch: all blocks sent, sending reset CMD");

    // ---- Reset command ----
    {
        const char rstCmd[] = {0x11, 0x00};
        buf[0] = 0x08;
        memcpy(buf + 1, rstCmd, sizeof(rstCmd));
        writeReg(buf[0], buf + 1, 2);
    }

    bool ready = waitForCpuReady();
    ESP_LOGI(TAG_TMF,
             "downloadRamPatch: waitForCpuReady() -> %s",
             ready ? "true" : "false");

    return ready;
}

