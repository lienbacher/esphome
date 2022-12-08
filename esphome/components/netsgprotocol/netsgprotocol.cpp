#include "netsgprotocol.h"

#define highbyte(val) (uint8_t)((val) >> 8)
#define lowbyte(val) (uint8_t)((val) &0xff)

namespace esphome {
namespace netsgprotocol {

static const char *const TAG = "netsgprotocol";

void NetSGProtocolComponent::setup() {
  ESP_LOGI(TAG, "NET SG Protocol Setting up");

  if (poll_interval > 0)
    set_update_interval(poll_interval * 1000);
  else {
    set_update_interval(1000);
    ESP_LOGW(TAG, "poll interval not properly configured, defaulting to 1 second");
  }

  if (set_pin_ != nullptr) {
    ESP_LOGI(TAG, "set pin is configured,reading configuration from Radio");

    uint8_t *bufferPointer = &mBuffer[0];

    *bufferPointer++ = 0xAA;  // command byte
    *bufferPointer++ = 0x5C;  // command byte
    *bufferPointer++ = 0x00;  // module identifier
    *bufferPointer++ = 0x00;  // module identifier
    *bufferPointer++ = 0x00;  // networking identifier
    *bufferPointer++ = 0x00;  // networking identifier
    *bufferPointer++ = 0x00;  // NC must be 0
    *bufferPointer++ = 0x00;  // RF power
    *bufferPointer++ = 0x00;  // NC must be 0
    *bufferPointer++ = 0x00;  // Baudrate
    *bufferPointer++ = 0x00;  // NC must be 0
    *bufferPointer++ = 0x00;  // RF channel (0 - 127)
    *bufferPointer++ = 0x00;  // NC must be 0
    *bufferPointer++ = 0x00;  // NC must be 0
    *bufferPointer++ = 0x00;  // NC must be 0
    *bufferPointer++ = 0x12;  // Length
    *bufferPointer++ = 0x00;  // NC must be 0
    *bufferPointer++ = 0x18;  // Checksum

    set_pin_->pin_mode(gpio::FLAG_OUTPUT);
    set_pin_->setup();

    // enable programming mode
    set_pin_->digital_write(false);
    delayMicroseconds(1000);

    this->write_array(mBuffer, 18);

    const uint32_t startTime = millis();
    while (millis() - startTime < 1000) {
      while (available()) {
        static uint8_t buffer[18];

        if (read_array(buffer, 18)) {
          if (mBuffer[0] == 0xAA && mBuffer[1] == 0x5D && mBuffer[17] == calcCRC(17)) {
            uint16_t moduleID = mBuffer[2] << 8 | (mBuffer[3] & 0xFF);   /// Unique module identifier
            uint16_t networkID = mBuffer[4] << 8 | (mBuffer[5] & 0xFF);  /// Network identifier
            uint8_t rfPower = mBuffer[7];                                /// RF power
            uint8_t baudrate = mBuffer[9];                               /// Baudrate
            uint8_t rfChannel = mBuffer[11];                             /// RF channel

            ESP_LOGI(TAG,
                     "LC12S Radio configured: \n\tmoduleID: %04X\n\tnetworkID: %04X\n\t"
                     "rfPower: %d\n\tbaudrate: %d\n\trfChannel: %02X",
                     moduleID, networkID, rfPower, baudrate, rfChannel);

            set_pin_->digital_write(true);
            return;
          }
        } else {
          ESP_LOGE(TAG, "reading radio configuration bytes failed!");
          set_pin_->digital_write(true);
          return;
        }
      }  // while(available())
    }    // check timeout

    ESP_LOGW(TAG, "reading radio configuration timed out");

    // disable programming mode of radio
    set_pin_->digital_write(true);
  }
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
      ESP_LOGI(TAG, "%d bytes available()", available());

      static uint8_t buffer[27];

      if (!read_array(buffer, 27)) {
        ESP_LOGE(TAG, "failed to read buffer of size 27");
        return;
      }

      if (buffer[0] == MAGIC_BYTE && buffer[1] == CMD_STATUS) {
        ESP_LOGI(TAG, "Status header received");

        uint32_t deviceID = buffer[6] << 24 | buffer[7] << 16 | buffer[8] << 8 | (buffer[9] & 0xFF);

        const uint32_t tempTotal = buffer[10] << 24 | buffer[11] << 16 | buffer[12] << 8 | (buffer[13] & 0xFF);
        float totalPower = (float) tempTotal;
        float dcVoltage = (buffer[15] << 8 | buffer[16]) / 100.0f;
        float dcCurrent = (buffer[17] << 8 | buffer[18]) / 100.0f;
        float acVoltage = (buffer[19] << 8 | buffer[20]) / 100.0f;
        float acCurrent = (buffer[21] << 8 | buffer[22]) / 100.0f;

        uint8_t state = buffer[25];        // not fully reversed
        uint8_t temperature = buffer[26];  // not fully reversed

        uint8_t crc = 0;
        for (size_t i = 0; i < 14; ++i) {
          crc += buffer[i];
        }
        uint8_t valid = buffer[14] == crc;

        if (valid > 0) {
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

          if (this->device_temperature_sensor_ != nullptr) {
            this->device_temperature_sensor_->publish_state(temperature);
          }
        }

        ESP_LOGI(TAG,
                 "dcVoltage: %f, dcCurrent: %f, dcPower: %f, acVoltage: %f, acCurrent: %f, acPower: %f, temperature: "
                 "%d, totalPower: %f, state: %x",
                 dcVoltage, dcCurrent, (dcVoltage * dcCurrent), acVoltage, acCurrent, (acVoltage * acCurrent),
                 temperature, totalPower, state);
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
