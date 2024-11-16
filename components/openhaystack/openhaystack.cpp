#include "openhaystack.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <esp_bt_main.h>
#include <esp_bt.h>
#include <freertos/task.h>
#include <esp_gap_ble_api.h>
#include <cstring>
#include "esphome/core/hal.h"

#ifdef USE_ARDUINO
#include <esp32-hal-bt.h>
#endif

namespace esphome {
namespace openhaystack {

static const char *const TAG = "openhaystack";

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x0640, // 1s
    .adv_int_max =  0x0C80, // 2s
    .adv_type = ADV_TYPE_NONCONN_IND,
    .own_addr_type = BLE_ADDR_TYPE_RANDOM,
    .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void OpenHaystack::setup() {
  ESP_LOGCONFIG(TAG, "Setting up OpenHaystack device...");
  global_openhaystack = this;

  xTaskCreatePinnedToCore(OpenHaystack::ble_core_task,
                          "ble_task",  // name
                          10000,       // stack size (in words)
                          nullptr,     // input params
                          1,           // priority
                          nullptr,     // Handle, not needed
                          0            // core
  );
}

void OpenHaystack::ble_core_task(void *params) {
  ble_setup();

  while (true) {
    // Đợi quảng bá BLE xong rồi vào deep sleep
    vTaskDelay(pdMS_TO_TICKS(60000));  // Wait for 1 minute before waking up
    esp_deep_sleep_start();  // Vào chế độ deep sleep
  }
}

void OpenHaystack::ble_setup() {
  // Khởi tạo bộ nhớ không thay đổi (NVS) cho Bluetooth controller
  esp_err_t err = nvs_flash_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init failed: %d", err);
    return;
  }

#ifdef USE_ARDUINO
  if (!btStart()) {
    ESP_LOGE(TAG, "btStart failed: %d", esp_bt_controller_get_status());
    return;
  }
#else
  if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
    // start bt controller
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
      esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
      err = esp_bt_controller_init(&cfg);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
        return;
      }
      while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
        ;
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
      err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
        return;
      }
    }
    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED) {
      ESP_LOGE(TAG, "esp bt controller enable failed");
      return;
    }
  }
#endif

  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  err = esp_bluedroid_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_bluedroid_init failed: %d", err);
    return;
  }
  err = esp_bluedroid_enable();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_bluedroid_enable failed: %d", err);
    return;
  }

  // Đặt công suất BLE thấp
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N12); // Công suất -12 dBm

  set_addr_from_key(global_openhaystack->random_address_, global_openhaystack->advertising_key_.data());
  set_payload_from_key(global_openhaystack->adv_data_, global_openhaystack->advertising_key_.data());

  err = esp_ble_gap_register_callback(OpenHaystack::gap_event_handler);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", err);
    return;
  }

  err = esp_ble_gap_set_rand_addr(global_openhaystack->random_address_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "esp_ble_gap_set_rand_addr failed: %s", esp_err_to_name(err));
    return;
  }

  esp_ble_gap_config_adv_data_raw((uint8_t *) &global_openhaystack->adv_data_, sizeof(global_openhaystack->adv_data_));
}

void OpenHaystack::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  esp_err_t err;
  switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: {
      err = esp_ble_gap_start_advertising(&ble_adv_params);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_start_advertising failed: %d", err);
      }
      break;
    }
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: {
      err = param->adv_start_cmpl.status;
      if (err != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "BLE adv start failed: %s", esp_err_to_name(err));
      }
      break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: {
      err = param->adv_start_cmpl.status;
      if (err != ESP_BT_STATUS_SUCCESS) {
        ESP_LOGE(TAG, "BLE adv stop failed: %s", esp_err_to_name(err));
      } else {
        ESP_LOGD(TAG, "BLE stopped advertising successfully");
      }
      break;
    }
    default:
      break;
  }
}

OpenHaystack *global_openhaystack = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace openhaystack
}  // namespace esphome

#endif
