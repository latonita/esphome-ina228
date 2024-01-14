#include "ina228.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include <cinttypes>

namespace esphome {
namespace ina228 {

static const char *const TAG = "ina228";

static const double VBUS_LSB = 0.0001953125;
static const double DIE_TEMP_LSB = 0.0078125;
static const double V_SHUNT_LSB_RANGE0 = 0.0003125;
static const double V_SHUNT_LSB_RANGE1 = 0.000078125;

bool INA228Component::read_u16_(uint8_t reg, uint16_t &out) {
  uint16_t data_in{0};
  auto ret = this->read_bytes(reg, (uint8_t *) &data_in, 2);
  out = byteswap(data_in);
  return ret;
}

bool INA228Component::read_s20_4_(uint8_t reg, int32_t &out) {
  int32_t data_in{0};
  auto ret = this->read_bytes(reg, (uint8_t *) &data_in, 3);

  bool sign = data_in & 0x80;
  out = byteswap(data_in & 0xffffff) >> 4;
  if (sign)
    out *= -1;
  return ret;
}

bool INA228Component::write_u16_(uint8_t reg, uint16_t val) {
  uint16_t data_out = byteswap(val);
  auto ret = this->write_bytes(reg, (uint8_t *) &data_out, 2);
  return ret;
}

void INA228Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up INA228...");

  ConfigurationRegister cfg{0};
  cfg.RST = 1;
  auto ret = this->write_u16_(RegisterMap::REG_CONFIG, cfg.raw_u16);
  if (!ret) {
    ESP_LOGE(TAG, "Reset failed, check comm?");
    this->mark_failed();
    return;
  }
  delay(1);

  uint16_t manufacturer_id = 0, dev_id = 0;
  ret = ret && this->read_u16_(RegisterMap::REG_MANUFACTURER_ID, manufacturer_id);
  ret = ret && this->read_u16_(RegisterMap::REG_DEVICE_ID, dev_id);
  if (ret) {
    ESP_LOGE(TAG, "ID read failed");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Manufacturer: 0x%04X, Device ID: 0x%04X", manufacturer_id, dev_id);
  delay(1);

  this->configure_shunt_(this->max_current_a_, this->shunt_resistance_ohm_);
  delay(2);

  AdcConfigurationRegister adc_cfg{0};
  ret = this->read_u16_(RegisterMap::REG_ADC_CONFIG, adc_cfg.raw_u16);
  adc_cfg.MODE = 0x0f;  // Fh = Continuous bus voltage, shunt voltage and temperature
  ret = ret && this->write_u16_(RegisterMap::REG_ADC_CONFIG, adc_cfg.raw_u16);
  delay(2);
}

bool INA228Component::configure_shunt_(double max_current, double r_shunt) {
  this->current_lsb_ = max_current / (2 << 19);  // 2 power of 19
  this->shunt_cal_ = (uint16_t) ((double) (13107.2 * 1000000) * this->current_lsb_ * r_shunt);

  ESP_LOGI(TAG, "New Rshunt=%f ohm, max current=%.3f", r_shunt, max_current);
  ESP_LOGI(TAG, "New CURRENT_LSB=%f, SHUNT_CAL=%u", this->current_lsb_, this->shunt_cal_);

  return this->write_u16_(RegisterMap::REG_SHUNT_CAL, this->shunt_cal_);
}

bool INA228Component::set_adc_range_(bool adc_range) {
  ConfigurationRegister cfg{0};
  auto ret = this->read_u16_(RegisterMap::REG_CONFIG, cfg.raw_u16);
  cfg.ADCRANGE = adc_range;
  ret = ret && this->write_u16_(RegisterMap::REG_CONFIG, cfg.raw_u16);

  this->adc_range_ = adc_range;

  return ret;
}

bool INA228Component::read_volt_shunt_(double &volt_out) {
  int32_t volt_reading = 0;
  auto ret = this->read_s20_4_(RegisterMap::REG_VSHUNT, volt_reading);
  volt_out = (volt_reading) * (this->adc_range_ ? V_SHUNT_LSB_RANGE1 : V_SHUNT_LSB_RANGE0);

  return ret;
}

bool INA228Component::read_voltage_(double &volt_out) {
  int32_t volt_reading = 0;
  auto ret = this->read_s20_4_(RegisterMap::REG_VBUS, volt_reading);
  volt_out = (volt_reading) *VBUS_LSB;

  return ret;
}

bool INA228Component::read_die_temp_(double &temp) {
  uint16_t temp_reading = 0;
  auto ret = this->read_u16_(RegisterMap::REG_DIETEMP, temp_reading);
  temp_reading = byteswap(temp_reading);
  temp = (double) temp_reading * DIE_TEMP_LSB;

  return ret;
}

bool INA228Component::read_current_(double &amps_out) {
  int32_t amps_reading = 0;
  auto ret = this->read_s20_4_(RegisterMap::REG_VBUS, amps_reading);
  amps_out = (amps_reading) * this->current_lsb_;

  return ret;
}

bool INA228Component::read_power_(double &power_out) {
  int32_t power_reading = 0;
  auto ret = this->read_bytes((uint8_t) RegisterMap::REG_POWER, (uint8_t *) &power_reading, 3);

  power_reading = byteswap(power_reading & 0xffffff) >> 8;
  power_out = 3.2 * this->current_lsb_ * power_reading;

  return ret;
}

bool INA228Component::read_energy_(double &joules_out) {
  uint64_t joules_reading = 0;  // Only 40 bits used
  auto ret = this->read_bytes((uint8_t) RegisterMap::REG_VSHUNT, (uint8_t *) &joules_reading, 5);

  joules_reading = byteswap(joules_reading & 0xffffffffffULL);
  joules_out = 16 * 3.2 * this->current_lsb_ * (double) joules_reading;

  return ret;
}

bool INA228Component::read_charge_(double &coulombs_out) {
  uint64_t coulombs_reading = 0;  // Only 40 bits used
  auto ret = this->read_bytes((uint8_t) RegisterMap::REG_VSHUNT, (uint8_t *) &coulombs_reading, 5);

  coulombs_reading = byteswap(coulombs_reading & 0xffffffffffULL);
  coulombs_out = this->current_lsb_ * (double) coulombs_reading;

  return ret;
}

bool INA228Component::clear_energy_counter_() {
  ConfigurationRegister cfg{0};
  auto ret = this->read_u16_(RegisterMap::REG_CONFIG, cfg.raw_u16);
  cfg.RSTACC = true;
  ret = ret && this->write_u16_(RegisterMap::REG_CONFIG, cfg.raw_u16);
  return ret;
}

void INA228Component::dump_config() {
  ESP_LOGCONFIG(TAG, "INA228:");
  LOG_I2C_DEVICE(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with INA228 failed!");
    return;
  }
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Shunt Voltage", this->shunt_voltage_sensor_);
  LOG_SENSOR("  ", "Bus Voltage", this->bus_voltage_sensor_);
  LOG_SENSOR("  ", "Die Temperature", this->die_temperature_sensor_);
  LOG_SENSOR("  ", "Current", this->current_sensor_);
  LOG_SENSOR("  ", "Power", this->power_sensor_);
  LOG_SENSOR("  ", "Energy", this->energy_sensor_);
  LOG_SENSOR("  ", "Charge", this->charge_sensor_);
}

float INA228Component::get_setup_priority() const { return setup_priority::DATA; }

void INA228Component::update() {
  bool all_okay{true};
  if (this->shunt_voltage_sensor_ != nullptr) {
    double shunt_voltage;
    if (!this->read_volt_shunt_(shunt_voltage)) {
      all_okay = false;
      this->status_set_warning();
    }
    this->shunt_voltage_sensor_->publish_state(shunt_voltage);
  }

  if (this->bus_voltage_sensor_ != nullptr) {
    double bus_voltage;
    if (!this->read_voltage_(bus_voltage)) {
      all_okay = false;
      this->status_set_warning();
    }
    this->bus_voltage_sensor_->publish_state(bus_voltage);
  }

  if (this->current_sensor_ != nullptr) {
    double current;
    if (!this->read_current_(current)) {
      all_okay = false;
      this->status_set_warning();
    }
    this->current_sensor_->publish_state(current);
  }

  if (this->power_sensor_ != nullptr) {
    double power;
    if (!this->read_power_(power)) {
      all_okay = false;
      this->status_set_warning();
    }
    this->power_sensor_->publish_state(power);
  }

  if (this->energy_sensor_ != nullptr) {
    double energy;
    if (!this->read_energy_(energy)) {
      all_okay = false;
      this->status_set_warning();
    }
    this->energy_sensor_->publish_state(energy);
  }

  if (this->charge_sensor_ != nullptr) {
    double charge;
    if (!this->read_charge_(charge)) {
      all_okay = false;
      this->status_set_warning();
    }
    this->charge_sensor_->publish_state(charge);
  }

  if (all_okay)
    this->status_clear_warning();
}

}  // namespace ina228
}  // namespace esphome
