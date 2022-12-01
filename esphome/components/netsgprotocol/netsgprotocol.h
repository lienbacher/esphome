#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace netsgprotocol {

#ifdef DEBUG_SERIAL
#define DEBUG(s) DEBUG_SERIAL.print(s)
#define DEBUGLN(s) DEBUG_SERIAL.println(s)
#if defined(__cplusplus) && (__cplusplus > 201703L)
#define DEBUGF(format, ...) DEBUG_SERIAL.printf_P(PSTR(format), __VA_OPT__(, ) __VA_ARGS__)
#else // !(defined(__cplusplus) && (__cplusplus >  201703L))
#define DEBUGF(format, ...) DEBUG_SERIAL.printf_P(PSTR(format), ##__VA_ARGS__)
#endif
#else
#define DEBUG(s)
#define DEBUGLN(s)
#define DEBUGF(format, ...)
#endif

#define CHECK_BIT(var, pos) (((var) >> (pos)) & 1)

// Commands
static const uint8_t MAGIC_BYTE = 0x43;
static const size_t BUFFER_SIZE = 32;
static const uint8_t CMD_STATUS = 0xC0; /// Get status command (0xC0)
static const uint8_t CMD_CONTROL = 0xC1; /// Control command (0xC1)
static const uint8_t CMD_POWER_GRADE = 0xC3; /// Set power grade command (0xC3)

// Commands values
static const uint8_t CMD_MAX_MOVE_VALUE = 0x0000;
static const uint8_t CMD_MAX_STILL_VALUE = 0x0001;
static const uint8_t CMD_DURATION_VALUE = 0x0002;
// Command Header & Footer
static const uint8_t CMD_FRAME_HEADER[4] = {0xFD, 0xFC, 0xFB, 0xFA};
static const uint8_t CMD_FRAME_END[4] = {0x04, 0x03, 0x02, 0x01};
// Data Header & Footer
static const uint8_t DATA_FRAME_HEADER[4] = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t DATA_FRAME_END[4] = {0xF8, 0xF7, 0xF6, 0xF5};
/*
Data Type: 6th byte
Target states: 9th byte
    Moving target distance: 10~11th bytes
    Moving target energy: 12th byte
    Still target distance: 13~14th bytes
    Still target energy: 15th byte
    Detect distance: 16~17th bytes
*/
enum PeriodicDataStructure : uint8_t {
  DATA_TYPES = 5,
  TARGET_STATES = 8,
  MOVING_TARGET_LOW = 9,
  MOVING_TARGET_HIGH = 10,
  MOVING_ENERGY = 11,
  STILL_TARGET_LOW = 12,
  STILL_TARGET_HIGH = 13,
  STILL_ENERGY = 14,
  DETECT_DISTANCE_LOW = 15,
  DETECT_DISTANCE_HIGH = 16,
};
enum PeriodicDataValue : uint8_t { HEAD = 0XAA, END = 0x55, CHECK = 0x00 };

enum AckDataStructure : uint8_t { COMMAND = 6, COMMAND_STATUS = 7 };

//  char cmd[2] = {enable ? 0xFF : 0xFE, 0x00};
class NetSGProtocolComponent : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  void loop() override;

  void set_dc_voltage_sensor(sensor::Sensor *sens) {this->dc_voltage_sensor_ = sens; };
  void set_dc_current_sensor(sensor::Sensor *sens) {this->dc_current_sensor_ = sens; };
  void set_dc_power_sensor(sensor::Sensor *sens) {this->dc_power_sensor_ = sens; };
  void set_ac_voltage_sensor(sensor::Sensor *sens) {this->ac_voltage_sensor_ = sens; };
  void set_ac_current_sensor(sensor::Sensor *sens) {this->ac_current_sensor_ = sens; };
  void set_ac_power_sensor(sensor::Sensor *sens) {this->ac_power_sensor_ = sens; };
  void set_power_gen_total_sesnor(sensor::Sensor *sens) {this->power_gen_total_sensor_ = sens; };
  void set_device_temperature_sensor(sensor::Sensor *sens) {this->device_temperature_sensor_ = sens; };
  void set_is_on(binary_sensor::BinarySensor *sens) {this->is_on_sensor_ = sens; };
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
binary_sensor::BinarySensor *is_on_sensor_{nullptr};
  
  void send_command_(uint8_t command, uint8_t value = 0x00);

  uint32_t inverter_device_id = 0;
  int poll_interval = -1;
  int power_grade = -1;

  uint8_t mBuffer[BUFFER_SIZE] = {0};

  uint8_t calcCRC(const size_t bytes) const {
    uint8_t crc = 0;
    for (size_t i = 0; i < bytes; ++i)
    {
      crc += mBuffer[i];
    }
    return crc;
  }
};

}  // namespace netsgprotocol
}  // namespace esphome
