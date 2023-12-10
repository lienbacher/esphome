#pragma once

//#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"

#include <vector>

namespace esphome {
namespace ads1256 {

// ADS1256 Register address
enum ADS1256RegisterAdress {
  ADS1256_RADD_STATUS = 0x00,
  ADS1256_RADD_MUX = 0x01,
  ADS1256_RADD_ADCON = 0x02,
  ADS1256_RADD_DATARATE = 0x03,
  ADS1256_RADD_IO = 0x04,
  ADS1256_RADD_OFC0 = 0x05,
  ADS1256_RADD_OFC1 = 0x06,
  ADS1256_RADD_OFC2 = 0x07,
  ADS1256_RADD_FSC0 = 0x08,
  ADS1256_RADD_FSC1 = 0x09,
  ADS1256_RADD_FSC2 = 0x0A,
};

enum ADS1256Command {
  // ADS1256 Command
  ADS1256_CMD_WAKEUP= 0x00,
  ADS1256_CMD_RDATA= 0x01,
  ADS1256_CMD_RDATAC= 0x03,
  ADS1256_CMD_SDATAC= 0x0f,
  ADS1256_CMD_RREG= 0x10,
  ADS1256_CMD_WREG= 0x50,
  ADS1256_CMD_SELFCAL= 0xF0,
  ADS1256_CMD_SELFOCAL= 0xF1,
  ADS1256_CMD_SELFGCAL= 0xF2,
  ADS1256_CMD_SYSOCAL= 0xF3,
  ADS1256_CMD_SYSGCAL= 0xF4,
  ADS1256_CMD_SYNC= 0xFC,
  ADS1256_CMD_STANDBY= 0xFD,
  ADS1256_CMD_RESET= 0xFE,
};

  // define multiplexer codes
enum ADS1256MultiplexerPositive {
  ADS1256_MUXP_AIN0= 0x00,
  ADS1256_MUXP_AIN1= 0x10,
  ADS1256_MUXP_AIN2= 0x20,
  ADS1256_MUXP_AIN3= 0x30,
  ADS1256_MUXP_AIN4= 0x40,
  ADS1256_MUXP_AIN5= 0x50,
  ADS1256_MUXP_AIN6= 0x60,
  ADS1256_MUXP_AIN7= 0x70,
  ADS1256_MUXP_AINCOM= 0x80,
};

enum ADS1256MultiplexerNegative {
  ADS1256_MUXN_AIN0= 0x00,
  ADS1256_MUXN_AIN1= 0x01,
  ADS1256_MUXN_AIN2= 0x02,
  ADS1256_MUXN_AIN3= 0x03,
  ADS1256_MUXN_AIN4= 0x04,
  ADS1256_MUXN_AIN5= 0x05,
  ADS1256_MUXN_AIN6= 0x06,
  ADS1256_MUXN_AIN7= 0x07,
  ADS1256_MUXN_AINCOM= 0x08,
};

// define gain codes
enum ADS1256Gain {
  ADS1256_GAIN_1 = 0x00,  // 5V
  ADS1256_GAIN_2 = 0x01,  // 2.5V
  ADS1256_GAIN_4 = 0x02,  // 1.25V
  ADS1256_GAIN_8 = 0x03,  // 0.625V
  ADS1256_GAIN_16 = 0x04, // 312.5mV
  ADS1256_GAIN_32 = 0x05, // 156.25mv
  ADS1256_GAIN_64 = 0x06, // 78.125mV
};

// define drate codes
/*
        NOTE : 	Data Rate vary depending on crystal frequency. Data rates
   listed below assumes the crystal frequency is 7.68Mhz
                for other frequency consult the datasheet.
*/

enum ADS1256Datarate {
  ADS1256_DATARATE_30000SPS = 0xF0,
  ADS1256_DATARATE_15000SPS = 0xE0,
  ADS1256_DATARATE_7500SPS = 0xD0,
  ADS1256_DATARATE_3750SPS = 0xC0,
  ADS1256_DATARATE_2000SPS = 0xB0,
  ADS1256_DATARATE_1000SPS = 0xA1,
  ADS1256_DATARATE_500SPS = 0x92,
  ADS1256_DATARATE_100SPS = 0x82,
  ADS1256_DATARATE_60SPS = 0x72,
  ADS1256_DATARATE_50SPS = 0x63,
  ADS1256_DATARATE_30SPS = 0x53,
  ADS1256_DATARATE_25SPS = 0x43,
  ADS1256_DATARATE_15SPS = 0x33,
  ADS1256_DATARATE_10SPS = 0x23,
  ADS1256_DATARATE_5SPS = 0x13,
  ADS1256_DATARATE_2SPS = 0x03,
};

class ADS1256Sensor;

struct ADS1256Store {
  volatile bool data_ready;
  ISRInternalGPIOPin pin;
  
  static void drdy_isr(ADS1256Store *store);
};

class ADS1256Component : public Component, 
                         public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_TRAILING,
                                         spi::DATA_RATE_10MHZ> {
 public:
  void register_sensor(ADS1256Sensor *obj) { this->sensors_.push_back(obj); }
  /// Set up the internal sensor array.
  void setup() override;
  void dump_config() override;
  /// HARDWARE_LATE setup priority
  float get_setup_priority() const override { return setup_priority::DATA; }
  void set_continuous_mode(bool continuous_mode) { continuous_mode_ = continuous_mode; }
  void set_drdy_pin(InternalGPIOPin *value) { this->drdy_pin_ = value; }
  void set_pdwn_pin(GPIOPin *value) { this->pdwn_pin_ = value; }
  void set_gain(ADS1256Gain gain) { gain_ = gain; }
  uint8_t get_gain() const { return gain_; }

  /// Helper method to request a measurement from a sensor.
  float request_measurement(ADS1256Sensor *sensor);

  void loop() override;
  
 protected:
  std::vector<ADS1256Sensor *> sensors_;
  uint16_t prev_config_{0};
  bool continuous_mode_;
  volatile static bool dataIsReady_;
  InternalGPIOPin *drdy_pin_{nullptr};
  GPIOPin *pdwn_pin_{nullptr};
  ADS1256Store store_;
  ADS1256Gain gain_;
  ADS1256Sensor *current_sensor_{nullptr};
  volatile bool rdy_{false};
};

/// Internal holder class that is in instance of Sensor so that the hub can create individual sensors.
class ADS1256Sensor : public sensor::Sensor, public PollingComponent, public voltage_sampler::VoltageSampler {
 public:
  ADS1256Sensor(ADS1256Component *parent) : parent_(parent) {}
  void update() override;
  void set_multiplexer_p(ADS1256MultiplexerPositive multiplexer) { multiplexer_p_ = multiplexer; }
  void set_multiplexer_n(ADS1256MultiplexerNegative multiplexer) { multiplexer_n_ = multiplexer; }
  void set_datarate(ADS1256Datarate datarate) { datarate_ = datarate; }
  float sample() override;
  uint8_t get_multiplexer_p() const { return multiplexer_p_; }
  uint8_t get_multiplexer_n() const { return multiplexer_n_; }
  
  uint8_t get_datarate() const { return datarate_; }

 protected:
  ADS1256Component *parent_;
  ADS1256MultiplexerPositive multiplexer_p_;
  ADS1256MultiplexerNegative multiplexer_n_;
  ADS1256Datarate datarate_;
};

}  // namespace ads1256
}  // namespace esphome

//#endif  // USE_ESP32