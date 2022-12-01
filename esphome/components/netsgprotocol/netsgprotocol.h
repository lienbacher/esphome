#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace netsgprotocol {

#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)

// Commands
static const uint8_t MAGIC_BYTE = 0x43;
static const size_t BUFFER_SIZE = 32;
static const uint8_t CMD_STATUS = 0xC0;       /// Get status command (0xC0)
static const uint8_t CMD_CONTROL = 0xC1;      /// Control command (0xC1)
static const uint8_t CMD_POWER_GRADE = 0xC3;  /// Set power grade command (0xC3)

//  char cmd[2] = {enable ? 0xFF : 0xFE, 0x00};
class NetSGProtocolComponent : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  void loop() override;

  void set_dc_voltage_sensor(sensor::Sensor *sens) { this->dc_voltage_sensor_ = sens; };
  void set_dc_current_sensor(sensor::Sensor *sens) { this->dc_current_sensor_ = sens; };
  void set_dc_power_sensor(sensor::Sensor *sens) { this->dc_power_sensor_ = sens; };
  void set_ac_voltage_sensor(sensor::Sensor *sens) { this->ac_voltage_sensor_ = sens; };
  void set_ac_current_sensor(sensor::Sensor *sens) { this->ac_current_sensor_ = sens; };
  void set_ac_power_sensor(sensor::Sensor *sens) { this->ac_power_sensor_ = sens; };
  void set_power_gen_total_sesnor(sensor::Sensor *sens) { this->power_gen_total_sensor_ = sens; };
  void set_device_temperature_sensor(sensor::Sensor *sens) { this->device_temperature_sensor_ = sens; };
  void set_inverter_device_id(int value) { this->inverter_device_id = value; };
  void set_poll_interval(int value) { this->poll_interval = value; };
  void set_power_grade(int value) { this->power_grade = value; };

  int32_t last_periodic_millis = millis();

 protected:
  sensor::Sensor *dc_voltage_sensor_{nullptr};
  sensor::Sensor *dc_current_sensor_{nullptr};
  sensor::Sensor *dc_power_sensor_{nullptr};
  sensor::Sensor *ac_voltage_sensor_{nullptr};
  sensor::Sensor *ac_current_sensor_{nullptr};
  sensor::Sensor *ac_power_sensor_{nullptr};
  sensor::Sensor *power_gen_total_sensor_{nullptr};
  sensor::Sensor *device_temperature_sensor_{nullptr};

  void send_command_(uint8_t command, uint8_t value = 0x00);

  uint32_t inverter_device_id = 0;
  int poll_interval = -1;
  int power_grade = -1;

  uint8_t mBuffer[BUFFER_SIZE] = {0};

  uint8_t calcCRC(const size_t bytes) const {
    uint8_t crc = 0;
    for (size_t i = 0; i < bytes; ++i) {
      crc += mBuffer[i];
    }
    return crc;
  }
};

}  // namespace netsgprotocol
}  // namespace esphome
