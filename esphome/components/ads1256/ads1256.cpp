#include "ads1256.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ads1256 {

static const char *const TAG = "ads1256";
static const uint8_t ADS1256_REGISTER_CONVERSION = 0x00;
static const uint8_t ADS1256_REGISTER_CONFIG = 0x01;

static const uint8_t ADS1256_DATA_RATE_860_SPS = 0b111;  // 3300_SPS for ADS1015

void ADS1256Store::drdy_isr(ADS1256Store *store) {
  store->data_ready = true;
}

void ADS1256Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ADS1256...");

  this->rdy_ = false;

  if(this->pdwn_pin_ != nullptr)
    this->pdwn_pin_->setup();

  this->spi_setup();
  this->cs_->setup();

  // set up data read interrupt
  this->drdy_pin_->setup();
  this->drdy_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
  this->store_.pin = this->drdy_pin_->to_isr();
  this->store_.data_ready = false;
  this->drdy_pin_->attach_interrupt(ADS1256Store::drdy_isr, &this->store_, gpio::INTERRUPT_FALLING_EDGE);

  // reset the ADS
  this->enable();
  this->write_byte(ADS1256_CMD_RESET);
  this->disable();

  // wait for the drdy pin to go low again after reset
  while(this->drdy_pin_->digital_read()) {}


  
  //this->enable();
  // read status register
  //this->write_byte(ADS1256_CMD_RREG | ADS1256_RADD_STATUS);
  //this->write_byte(0);
  //uint8_t status = this->read_byte();
  //ESP_LOGCONFIG(TAG, "ADS1256 Status Register: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(status));

  // turn buffer off
  //status = status | 2; // flip the buffer bit on
  //this->write_byte(ADS1256_CMD_WREG | ADS1256_RADD_STATUS);
  //this->write_byte(0);
  //this->write_byte(status);

  // read datarate register
  //this->write_byte(ADS1256_CMD_RREG | ADS1256_RADD_DATARATE);
  //this->write_byte(0);
  //status = this->read_byte();
  //ESP_LOGCONFIG(TAG, "ADS1256 Datarate Register: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(status));

  // read gain register
  //this->write_byte(ADS1256_CMD_RREG | ADS1256_RADD_ADCON);
  //this->write_byte(0);
  //status = this->read_byte();
  //ESP_LOGCONFIG(TAG, "ADS1256 ADCON Register: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(status));
  //this->disable();
  
  //delayMicroseconds(20);
  
  // write gain
  this->enable();
  this->write_byte(ADS1256_CMD_WREG | ADS1256_RADD_ADCON);
  this->write_byte(0);
  this->write_byte(ADS1256_RADD_ADCON | this->gain_);
  this->disable();

  // wait for drdy  
  while(this->drdy_pin_->digital_read()) {}

  // configure datarate
  this->enable();
  this->write_byte(ADS1256_CMD_WREG | ADS1256_RADD_DATARATE);
  this->write_byte(0);
  this->write_byte(this->sensors_.front()->get_datarate());
  //this->write_byte(ADS1256_DATARATE_2SPS);

  this->disable();

  this->rdy_ = true;

  // // read registers again to check for success
  // this->enable();
  // // read status register
  // this->write_byte(ADS1256_CMD_RREG | ADS1256_RADD_STATUS);
  // this->write_byte(0);
  // status = this->read_byte();
  // ESP_LOGCONFIG(TAG, "ADS1256 Status Register: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(status));

  // // read datarate register
  // this->write_byte(ADS1256_CMD_RREG | ADS1256_RADD_DATARATE);
  // this->write_byte(0);
  // status = this->read_byte();
  // ESP_LOGCONFIG(TAG, "ADS1256 Datarate Register: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(status));

  // // read gain register
  // this->write_byte(ADS1256_CMD_RREG | ADS1256_RADD_ADCON);
  // this->write_byte(0);
  // status = this->read_byte();
  // ESP_LOGCONFIG(TAG, "ADS1256 ADCON Register: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(status));
  // this->disable();
  
  // delayMicroseconds(20);
}

void ADS1256Component::dump_config() {
  ESP_LOGCONFIG(TAG, "ADS1256:");
  LOG_PIN("  CS Pin: ", this->cs_);
  LOG_PIN("  DRDY Pin: ", this->drdy_pin_);
  LOG_PIN("  PDWN_PIN: ", this->pdwn_pin_);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with ADS1256 failed!");
  }

  ESP_LOGCONFIG(TAG, "    Gain: %u", this->get_gain());

  for (auto *sensor : this->sensors_) {
    LOG_SENSOR("  ", "Sensor", sensor);
    ESP_LOGCONFIG(TAG, "    Multiplexer Positive: %u", sensor->get_multiplexer_p());
    ESP_LOGCONFIG(TAG, "    Multiplexer Negative: %u", sensor->get_multiplexer_n());
    ESP_LOGCONFIG(TAG, "    Datarate: %u", sensor->get_datarate());
  }
}

void ADS1256Component::loop() {
  if (!this->store_.data_ready || !this->rdy_) {
    return;
  }

  //ESP_LOGD(TAG, "DRDY L");
  this->store_.data_ready = false;

  // Combine all 3-bytes to 24-bit data using byte shifting.
  this->enable();

  this->write_byte(ADS1256_CMD_RDATAC);
  delayMicroseconds(7);

  uint8_t first, middle, last;
  first = read_byte();
  middle = read_byte();
  last = read_byte();

  this->disable();

  //ESP_LOGCONFIG(TAG, "fst:  0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(first));
  //ESP_LOGCONFIG(TAG, "mid: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(middle));
  //ESP_LOGCONFIG(TAG, "lst:   0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(last));

  //long value = ((long)first << 16) + ((long)middle << 8) + ((long)last);
  long value = ((long)first << 16) | ((long)middle << 8) | ((long)last);

  if (value >> 23 == 1) //if the 24th digit (sign) is 1, the number is negative
  {
    value = value - 16777216;  //conversion for the negative sign
    //"mirroring" around zero
  }

  const float _VREF = 2.5;
  // 0.542mV 100g
  // 0.356mV 0g
  // 0.186 diff

  float millivolts;
  float divider = 8388608.0f;
  switch (this->get_gain()) {
    case ADS1256_GAIN_1:
      millivolts = (value * 5000) / divider;
      break;
    case ADS1256_GAIN_2:
      millivolts = (value * 2500) / divider;
      break;
    case ADS1256_GAIN_4:
      millivolts = (value * 1250) / divider;
      break;
    case ADS1256_GAIN_8:
      millivolts = (value * 625) / divider;
      break;
    case ADS1256_GAIN_16:
      millivolts = (value * 312.5) / divider;
      break;
    case ADS1256_GAIN_32:
      millivolts = (value * 156.25) / divider;
      break;
    case ADS1256_GAIN_64:
      millivolts = (value * 78.125) / divider;
      break;
    default:
      millivolts = NAN;
  }

  //float voltage = ((2 * _VREF) / 8388608) * value / (pow(2, this->get_gain())); //8388608 = 2^{23} - 1
  float voltage = millivolts / 1000.0f;
  //REF: p23, Table 16.

  //ESP_LOGCONFIG(TAG, "voltage: %f, value: %d", voltage, value);

  this->sensors_.front()->publish_state(voltage);
}

float ADS1256Component::request_measurement(ADS1256Sensor *sensor) {
  ESP_LOGD(TAG, "'%s': measurement requested from ", sensor->get_name().c_str());

  if(sensor == current_sensor_)
    return 0;
  else
    this->current_sensor_ = sensor;

  unsigned char mux_config = sensor->get_multiplexer_p() | sensor->get_multiplexer_n();
  this->enable();
  this->transfer_byte(ADS1256_CMD_WREG | ADS1256_RADD_MUX);   // write register mux
  this->transfer_byte(0);   // write 1+0 registers
  this->transfer_byte(mux_config);
  this->disable();

  return 0;

  // delay(1);

  // this->enable();

  // this->write_byte(ADS1256_CMD_RDATA);
  
  // // Combine all 3-bytes to 24-bit data using byte shifting.
  // uint8_t first, middle, last;
  // first = read_byte();
  // middle = read_byte();
  // last = read_byte();

  // ESP_LOGCONFIG(TAG, "first byte:  0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(first));
  // ESP_LOGCONFIG(TAG, "middle byte: 0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(middle));
  // ESP_LOGCONFIG(TAG, "last byte:   0b" BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(last));

  // long value = ((long)first << 16) + ((long)middle << 8) + ((long)last);

  // if (value & 0x00800000) { // if the 24 bit value is negative reflect it to 32bit
  //   value |= 0xff000000;
  // }
  
  // this->disable();

  //return (float) value;

  // float millivolts;
  // float divider = (sensor->get_resolution() == ADS1256_16_BITS) ? 32768.0f : 2048.0f;
  // switch (sensor->get_gain()) {
  //   case ADS1256_GAIN_6P144:
  //     millivolts = (signed_conversion * 6144) / divider;
  //     break;
  //   case ADS1256_GAIN_4P096:
  //     millivolts = (signed_conversion * 4096) / divider;
  //     break;
  //   case ADS1256_GAIN_2P048:
  //     millivolts = (signed_conversion * 2048) / divider;
  //     break;
  //   case ADS1256_GAIN_1P024:
  //     millivolts = (signed_conversion * 1024) / divider;
  //     break;
  //   case ADS1256_GAIN_0P512:
  //     millivolts = (signed_conversion * 512) / divider;
  //     break;
  //   case ADS1256_GAIN_0P256:
  //     millivolts = (signed_conversion * 256) / divider;
  //     break;
  //   default:
  //     millivolts = NAN;
  // }

  //this->status_clear_warning();
  //return millivolts / 1e3f;
}

float ADS1256Sensor::sample() { 
  ESP_LOGD(TAG, "sampling: '%s'", this->get_name().c_str());
  return this->parent_->request_measurement(this); 
}

void ADS1256Sensor::update() {
//   float v = this->parent_->request_measurement(this);
//   if (!std::isnan(v)) {
//     ESP_LOGD(TAG, "'%s': Got Voltage=%fV", this->get_name().c_str(), v);
//     this->publish_state(v);
//   }
}

}  // namespace ads1256
}  // namespace esphome
