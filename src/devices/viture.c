#include "devices.h"
#include "driver.h"
#include "imu.h"
#include "logging.h"
#include "outputs.h"
#include "runtime_context.h"
#include "sdks/viture_one.h"
#include "strings.h"

#include <math.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VITURE_ID_PRODUCT_COUNT 7
#define VITURE_ID_VENDOR 0x35ca
#define VITURE_ONE_MODEL_NAME "One"
#define VITURE_ONE_LITE_MODEL_NAME "One Lite"
#define VITURE_PRO_MODEL_NAME "Pro"
#define IMU_HISTORY_LEN 5

const int viture_supported_id_product[VITURE_ID_PRODUCT_COUNT] = {
    0x1011, 0x1013, 0x1017, 0x1015, 0x101b, 0x1019, 0x101d,
};

const char* viture_supported_models[VITURE_ID_PRODUCT_COUNT] = {
    VITURE_ONE_MODEL_NAME, VITURE_ONE_MODEL_NAME, VITURE_ONE_MODEL_NAME,
    VITURE_ONE_LITE_MODEL_NAME, VITURE_ONE_LITE_MODEL_NAME,
    VITURE_PRO_MODEL_NAME, VITURE_PRO_MODEL_NAME,
};

const float VITURE_ONE_PITCH_ADJUSTMENT = 6.0;
const float VITURE_PRO_PITCH_ADJUSTMENT = 3.0;
static imu_quat_type adjustment_quat;
static imu_quat_type imu_history[IMU_HISTORY_LEN];
static int imu_history_index = 0;

static imu_quat_type slerp(imu_quat_type q1, imu_quat_type q2, float t) {
    float dot = q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w;
    if (dot < 0.0f) {
        q2.x = -q2.x; q2.y = -q2.y; q2.z = -q2.z; q2.w = -q2.w;
        dot = -dot;
    }
    if (dot > 0.9995f) {
        imu_quat_type result = {
            .x = q1.x + t*(q2.x - q1.x),
            .y = q1.y + t*(q2.y - q1.y),
            .z = q1.z + t*(q2.z - q1.z),
            .w = q1.w + t*(q2.w - q1.w)
        };
        float norm = sqrtf(result.x*result.x + result.y*result.y + result.z*result.z + result.w*result.w);
        result.x /= norm; result.y /= norm; result.z /= norm; result.w /= norm;
        return result;
    }
    float theta_0 = acosf(dot);
    float sin_theta_0 = sinf(theta_0);
    float theta = theta_0 * t;
    float sin_theta = sinf(theta);

    float s1 = cosf(theta) - dot * sin_theta / sin_theta_0;
    float s2 = sin_theta / sin_theta_0;

    imu_quat_type result = {
        .x = s1*q1.x + s2*q2.x,
        .y = s1*q1.y + s2*q2.y,
        .z = s1*q1.z + s2*q2.z,
        .w = s1*q1.w + s2*q2.w
    };
    return result;
}

static imu_quat_type slerp_average() {
    imu_quat_type result = imu_history[0];
    for (int i = 1; i < IMU_HISTORY_LEN; ++i) {
        result = slerp(result, imu_history[i], 1.0f / (i + 1));
    }
    return result;
}

static float float_from_imu_data(uint8_t *data) {
    float value = 0;
    uint8_t tem[4] = { data[3], data[2], data[1], data[0] };
    memcpy(&value, tem, 4);
    return value;
}

static bool old_firmware_version = true;
static bool connected = false;
static bool initialized = false;

void handle_viture_event(uint8_t *data, uint16_t len, uint32_t timestamp) {
    if (!connected || driver_disabled()) return;

    imu_quat_type quat;
    if (len >= 36 && !old_firmware_version) {
        quat.w = float_from_imu_data(data + 20);
        quat.x = float_from_imu_data(data + 24);
        quat.y = float_from_imu_data(data + 28);
        quat.z = float_from_imu_data(data + 32);
    } else {
        float euler_roll = float_from_imu_data(data);
        float euler_pitch = float_from_imu_data(data + 4);
        float euler_yaw = float_from_imu_data(data + 8);
        imu_euler_type euler = { .roll = euler_roll, .pitch = euler_pitch, .yaw = euler_yaw };
        quat = euler_to_quaternion_zxy(euler);
    }

    quat = multiply_quaternions(quat, adjustment_quat);
    imu_history[imu_history_index % IMU_HISTORY_LEN] = quat;
    imu_history_index++;

    imu_quat_type smoothed = slerp_average();
    driver_handle_imu_event(timestamp, smoothed);
}

bool sbs_mode_enabled = false;
void viture_mcu_callback(uint16_t msgid, uint8_t *data, uint16_t len, uint32_t ts) {
    if (msgid == MCU_SBS_ADJUSTMENT_MSG) {
        sbs_mode_enabled = data[0] == MCU_SBS_ADJUSTMENT_ENABLED;
    }
}

device_properties_type* viture_supported_device(uint16_t vendor_id, uint16_t product_id, uint8_t usb_bus, uint8_t usb_address) {
    if (vendor_id == VITURE_ID_VENDOR) {
        for (int i=0; i < VITURE_ID_PRODUCT_COUNT; i++) {
            if (product_id == viture_supported_id_product[i]) {
                device_properties_type* device = calloc(1, sizeof(device_properties_type));
                *device = viture_one_properties;
                device->hid_vendor_id = vendor_id;
                device->hid_product_id = product_id;
                device->model = (char *)viture_supported_models[i];

                if (equal(VITURE_PRO_MODEL_NAME, device->model)) {
                    device->fov = 43.0;
                    adjustment_quat = device_pitch_adjustment(VITURE_PRO_PITCH_ADJUSTMENT);
                } else {
                    adjustment_quat = device_pitch_adjustment(VITURE_ONE_PITCH_ADJUSTMENT);
                }

                return device;
            }
        }
    }

    return NULL;
};

static pthread_mutex_t viture_connection_mutex = PTHREAD_MUTEX_INITIALIZER;
static void disconnect(bool soft) {
    pthread_mutex_lock(&viture_connection_mutex);
    if (connected) {
        set_imu(false);
        connected = false;
    }

    // VITURE SDK freezes if we attempt deinit() while it's still physically connected, so only do this if the device is no longer present
    if (initialized && (!soft || !device_present())) {
        deinit();
        initialized = false;
    }
    pthread_mutex_unlock(&viture_connection_mutex);
}

bool viture_device_connect() {
    if (!connected || get_imu_state() != STATE_ON) {
        // newer firmware may require a bit of a wait after the device is plugged in before attempting to connect
        sleep(2);

        if (!initialized) initialized = init(handle_viture_event, viture_mcu_callback);
        connected = initialized && set_imu(true) == ERR_SUCCESS;
    }

    if (connected) {
        device_properties_type* device = device_checkout();
        if (device != NULL) {
            set_imu_fq(IMU_FREQUENCE_240);
            int imu_freq = get_imu_fq();
            if (imu_freq < IMU_FREQUENCE_60 || imu_freq > IMU_FREQUENCE_240) {
                imu_freq = IMU_FREQUENCE_60;
            }

            if (config()->debug_device) log_debug("VITURE: IMU frequency set to %s\n", frequency_to_string[imu_freq]);

            // use the current value in case the frequency we requested isn't supported
            device->imu_cycles_per_s = frequency_enum_to_value[imu_freq];
            device->imu_buffer_size = (int) device->imu_cycles_per_s / 60;

            // not a great way to check the firmware version but it's all we have
            old_firmware_version = equal(VITURE_PRO_MODEL_NAME, device->model) ? false : (device->imu_cycles_per_s == 60);
            if (old_firmware_version) log_message("VITURE: Detected old firmware version\n");

            device->sbs_mode_supported = !old_firmware_version;
            device->firmware_update_recommended = old_firmware_version;

            sbs_mode_enabled = get_3d_state() == STATE_ON;
        } else {
            disconnect(false);
        }
        device_checkin(device);
    }

    return connected;
}

void viture_block_on_device() {
    device_properties_type* device = device_checkout();
    if (device != NULL) {
        int imu_state = get_imu_state();
        if (connected && imu_state != ERR_WRITE_FAIL) wait_for_imu_start();
        while (connected && imu_state != ERR_WRITE_FAIL) {
            sleep(1);
            imu_state = get_imu_state();
        }
    }
    disconnect(true);
    device_checkin(device);
};

bool viture_device_is_sbs_mode() {
    return sbs_mode_enabled;
};

bool viture_device_set_sbs_mode(bool enabled) {
    sbs_mode_enabled = enabled;
    return set_3d(enabled) == ERR_SUCCESS;
};

bool viture_is_connected() {
    return connected;
};

void viture_disconnect(bool soft) {
    disconnect(soft);
};

const device_driver_type viture_driver = {
    .supported_device_func              = viture_supported_device,
    .device_connect_func                = viture_device_connect,
    .block_on_device_func               = viture_block_on_device,
    .device_is_sbs_mode_func            = viture_device_is_sbs_mode,
    .device_set_sbs_mode_func           = viture_device_set_sbs_mode,
    .is_connected_func                  = viture_is_connected,
    .disconnect_func                    = viture_disconnect
};
