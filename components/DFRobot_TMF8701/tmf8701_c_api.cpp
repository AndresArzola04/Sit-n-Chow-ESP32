#include "tmf8701_c_api.h"
#include "DFRobot_TMF8701_espidf.h"
#include "esp_log.h"

struct DFRobot_TMF8701_C {
    DFRobot_TMF8701 impl;
};

extern "C" {

DFRobot_TMF8701_C *tmf8701_create(gpio_num_t enPin,
                                  gpio_num_t intPin,
                                  i2c_port_t i2cPort,
                                  uint8_t i2cAddr)
{
    auto *h = new DFRobot_TMF8701_C;
    h->impl = DFRobot_TMF8701(enPin, intPin, i2cPort, i2cAddr);
    return h;
}

void tmf8701_destroy(DFRobot_TMF8701_C *handle)
{
    delete handle;
}

int tmf8701_begin(DFRobot_TMF8701_C *handle)
{
    if (!handle) return -1;
    return handle->impl.begin();
}

bool tmf8701_start_measurement(DFRobot_TMF8701_C *handle)
{
    if (!handle) return false;

    // Default behavior when you ALREADY loaded calibration data:
    // use "calibration + algorithm" and COMBINE distance mode.
    return handle->impl.startMeasurement(
        DFRobot_TMF8x01::eCalibModeConfig_t::eModeCalibAndAlgoState,  // name may differ, see your enum
        DFRobot_TMF8701::eCOMBINE
    );
}


bool tmf8701_is_data_ready(DFRobot_TMF8701_C *handle)
{
    if (!handle) return false;
    return handle->impl.isDataReady();
}

uint16_t tmf8701_get_distance_mm(DFRobot_TMF8701_C *handle)
{
    if (!handle) return 0;
    return handle->impl.getDistance_mm();
}

bool tmf8701_get_calibration_data(DFRobot_TMF8701_C *handle,
                                  uint8_t *data,
                                  uint8_t len)
{
    if (!handle || !data || len == 0) {
        return false;
    }

    // Ask the C++ driver for up to 'len' bytes.
    // The driver itself knows its true calibration size.
    bool ok = handle->impl.getCalibrationData(data, len);

    if (!ok) {
        ESP_LOGE("TMF8701_C", "tmf8701_get_calibration_data: driver returned false");
    }
    return ok;
}


bool tmf8701_set_calibration_data(DFRobot_TMF8701_C *handle,
                                  const uint8_t *data,
                                  uint8_t len)
{
    if (!handle || !data || len == 0) {
        return false;
    }

    // get/setCalibrationData take a non-const pointer; copy into temp
    uint8_t tmp[32];
    if (len > sizeof(tmp)) {
        ESP_LOGE("TMF8701_C", "tmf8701_set_calibration_data: len(%u) too large", len);
        return false;
    }
    memcpy(tmp, data, len);

    bool ok = handle->impl.setCalibrationData(tmp, len);
    if (!ok) {
        ESP_LOGE("TMF8701_C", "tmf8701_set_calibration_data: driver returned false");
    }
    return ok;
}


bool tmf8701_set_calibration_mode(DFRobot_TMF8701_C *handle,
                                  tmf8701_calib_mode_t mode)
{
    if (!handle) {
        return false;
    }
    return handle->impl.setCalibrationMode(
        static_cast<DFRobot_TMF8x01::eCalibModeConfig_t>(mode)
    );
}

bool tmf8701_start_measurement_ex(DFRobot_TMF8701_C *handle,
                                  tmf8701_calib_mode_t calib_mode,
                                  tmf8701_distance_mode_t dist_mode)
{
    if (!handle) {
        return false;
    }

    DFRobot_TMF8x01::eCalibModeConfig_t c_mode =
        static_cast<DFRobot_TMF8x01::eCalibModeConfig_t>(calib_mode);

    DFRobot_TMF8701::eDistaceMode_t d_mode;
    switch (dist_mode) {
        case TMF8701_DISTANCE_MODE_PROXIMITY:
            d_mode = DFRobot_TMF8701::ePROXIMITY;
            break;
        case TMF8701_DISTANCE_MODE_DISTANCE:
            d_mode = DFRobot_TMF8701::eDISTANCE;
            break;
        case TMF8701_DISTANCE_MODE_COMBINE:
        default:
            d_mode = DFRobot_TMF8701::eCOMBINE;
            break;
    }

    return handle->impl.startMeasurement(c_mode, d_mode);
}


void tmf8701_get_model(DFRobot_TMF8701_C *handle,
                       char *out, uint32_t out_len)
{
    if (!handle) return;
    handle->impl.getSensorModel(out, out_len);
}

void tmf8701_get_version(DFRobot_TMF8701_C *handle,
                         char *out, uint32_t out_len)
{
    if (!handle) return;
    handle->impl.getSoftwareVersion(out, out_len);
}

} // extern "C"
