#include "netsgprotocol.h"

#define highbyte(val) (uint8_t)((val) >> 8)
#define lowbyte(val) (uint8_t)((val) &0xff)

namespace esphome {
namespace netsgprotocol {

static const char *const TAG = "netsgprotocol";

void NetSGProtocolComponent::setup() {
  if (poll_interval > 0)
    set_update_interval(poll_interval * 1000);
  else {
    set_update_interval(1000);
    ESP_LOGW(TAG, "poll interval not properly configured, defaulting to 1 second");
  }

  ESP_LOGI(TAG, "NET SG Protocol Setting up");
}

void NetSGProtocolComponent::dump_config() {
#ifdef USE_SENSOR
  LOG_SENSOR(TAG, "DC Voltage", dc_voltage_sensor_);
  LOG_SENSOR(TAG, "DC Current", dc_current_sensor_);
  LOG_SENSOR(TAG, "DC Power", dc_power_sensor_);
  LOG_SENSOR(TAG, "AC Voltage", ac_voltage_sensor_);
  LOG_SENSOR(TAG, "AC Current", ac_current_sensor_);
  LOG_SENSOR(TAG, "AC Power", ac_power_sensor_);
  ESP_LOGCONFIG(TAG, "Inverter Device ID: %d", inverter_device_id);
  ESP_LOGCONFIG(TAG, "Poll Interval: %d", poll_interval);

#endif
}

void NetSGProtocolComponent::loop() {}

void NetSGProtocolComponent::send_command_(uint8_t command, uint8_t value) {
  uint8_t *bufferPointer = &mBuffer[0];

  *bufferPointer++ = MAGIC_BYTE;
  *bufferPointer++ = command;
  *bufferPointer++ = 0x00;  // data box ID
  *bufferPointer++ = 0x00;  // data box ID
  *bufferPointer++ = 0x00;
  *bufferPointer++ = 0x00;
  *bufferPointer++ = (inverter_device_id >> 24) & 0xFF;
  *bufferPointer++ = (inverter_device_id >> 16) & 0xFF;
  *bufferPointer++ = (inverter_device_id >> 8) & 0xFF;
  *bufferPointer++ = inverter_device_id & 0xFF;
  *bufferPointer++ = 0x00;
  *bufferPointer++ = 0x00;
  *bufferPointer++ = 0x00;
  *bufferPointer++ = value;
  *bufferPointer++ = calcCRC(14);

  this->write_array(mBuffer, 15);
}

void NetSGProtocolComponent::update() {
  send_command_(CMD_STATUS);

  const uint32_t startTime = millis();
  while (millis() - startTime < 1000) {
    while (available()) {
      if (read() == MAGIC_BYTE && read() == CMD_STATUS) {
        ESP_LOGI(TAG, "Status header received");
        static uint8_t buffer[26];

        read_array(buffer, 26);

        uint32_t deviceID = buffer[6] << 24 | buffer[7] << 16 | buffer[8] << 8 | (buffer[9] & 0xFF);

        const uint32_t tempTotal = buffer[10] << 24 | buffer[11] << 16 | buffer[12] << 8 | (buffer[13] & 0xFF);
        float totalPower = (float) tempTotal;
        float dcVoltage = (buffer[15] << 8 | buffer[16]) / 100.0f;
        float dcCurrent = (buffer[17] << 8 | buffer[18]) / 100.0f;
        float acVoltage = (buffer[19] << 8 | buffer[20]) / 100.0f;
        float acCurrent = (buffer[21] << 8 | buffer[22]) / 100.0f;

        if (this->power_gen_total_sensor_ != nullptr) {
          this->power_gen_total_sensor_->publish_state(totalPower);
        }

        if (this->dc_voltage_sensor_ != nullptr) {
          this->dc_voltage_sensor_->publish_state(dcVoltage);
        }

        if (this->dc_current_sensor_ != nullptr) {
          this->dc_current_sensor_->publish_state(dcCurrent);
        }

        if (this->dc_power_sensor_ != nullptr) {
          this->dc_power_sensor_->publish_state(dcVoltage * dcCurrent);
        }

        if (this->ac_voltage_sensor_ != nullptr) {
          this->ac_voltage_sensor_->publish_state(acVoltage);
        }

        if (this->ac_current_sensor_ != nullptr) {
          this->ac_current_sensor_->publish_state(acCurrent);
        }

        if (this->ac_power_sensor_ != nullptr) {
          this->ac_power_sensor_->publish_state(acVoltage * acCurrent);
        }

        uint8_t state = buffer[25];        // not fully reversed
        uint8_t temperature = buffer[26];  // not fully reversed

        if (this->device_temperature_sensor_ != nullptr) {
          this->device_temperature_sensor_->publish_state(temperature);
        }

        uint8_t valid = buffer[14] == calcCRC(14);

        ESP_LOGI(TAG, "CRC %s\n", valid ? "valid" : "invalid");

        if (valid)
          return;
      }
    }
  }

  ESP_LOGI(TAG, "No valid response from inverter");
}

}  // namespace netsgprotocol
}  // namespace esphome
