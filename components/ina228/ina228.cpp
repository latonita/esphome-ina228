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
  auto ret = this->read_register(reg, (uint8_t *) &data_in, 2, false);
  ESP_LOGD(TAG, "read_u16_ 0x%02X, ret= %d, raw 0x%04X", reg, ret, data_in);
  out = byteswap(data_in);
  return ret == i2c::ERROR_OK;
}

bool INA228Component::read_s20_4_(uint8_t reg, int32_t &out) {
  // Two's complement value. Highest bit is the sign
  int32_t data_in{0};
  auto ret = this->read_register(reg, (uint8_t *) &data_in, 3, false);
  ESP_LOGD(TAG, "read_s20_4_ 0x%02X, ret= %d, raw 0x%08X", reg, ret, data_in);

  bool sign = data_in & 0x80;
  data_in = byteswap(data_in & 0xFFFFFF) >> 12;
  if (sign)
    data_in += 0xFFF00000;
  out = data_in;
  return ret == i2c::ERROR_OK;
}

bool INA228Component::write_u16_(uint8_t reg, uint16_t val) {
  uint16_t data_out = byteswap(val);
  auto ret = this->write_register(reg, (uint8_t *) &data_out, 2);
  if (ret != i2c::ERROR_OK) {
    ESP_LOGD(TAG, "write_u16 failed ret=%d, reg=0x%02X, val=0x%04X", ret, reg, val);
  }
  return ret == i2c::ERROR_OK;
}

void INA228Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up INA228...");

  ConfigurationRegister cfg{0};
  cfg.RST = true;
  auto ret = this->write_u16_(RegisterMap::REG_CONFIG, cfg.raw_u16);
  if (!ret) {
    ESP_LOGE(TAG, "Reset failed, check connection");
    this->mark_failed();
    return;
  }
  delay(1);

  uint16_t manufacturer_id{0}, dev_id{0}, rev_id{0};
  this->read_u16_(RegisterMap::REG_MANUFACTURER_ID, manufacturer_id);
  this->read_u16_(RegisterMap::REG_DEVICE_ID, dev_id);
  rev_id = dev_id & 0x0F;
  dev_id >>= 4;
  ESP_LOGI(TAG, "Manufacturer: 0x%04X, Device ID: 0x%04X, Revision: %d", manufacturer_id, dev_id, rev_id);
  delay(1);

  if (manufacturer_id != 0x5449 || dev_id != 0x228) {
    ESP_LOGW(TAG, "Manufacturer ID and device IDs do not match original 0x5449 and 0x228.");
  }

  this->configure_shunt_(this->max_current_a_, this->shunt_resistance_ohm_);
  delay(2);

  this->clear_energy_counter_();
  delay(2);

  AdcConfigurationRegister adc_cfg{0};
  ret = this->read_u16_(RegisterMap::REG_ADC_CONFIG, adc_cfg.raw_u16);
  ESP_LOGD(TAG, "Read REG_ADC_CONFIG returned %s, 0x%04X", TRUEFALSE(ret), adc_cfg.raw_u16);
  adc_cfg.MODE = 0x0f;  // Fh = Continuous bus voltage, shunt voltage and temperature
  ret = this->write_u16_(RegisterMap::REG_ADC_CONFIG, adc_cfg.raw_u16);
  ESP_LOGD(TAG, "Read REG_ADC_CONFIG2 returned %s, 0x%04X", TRUEFALSE(ret), adc_cfg.raw_u16);
  delay(2);
}

bool INA228Component::configure_shunt_(double max_current, double r_shunt) {
  this->current_lsb_ = max_current / (2 << 19);  // 2 power of 19
  this->shunt_cal_ = (uint16_t) ((double) (13107.2 * 1000000) * this->current_lsb_ * r_shunt);

  ESP_LOGD(TAG, "New Rshunt=%f ohm, max current=%.3f", r_shunt, max_current);
  ESP_LOGD(TAG, "New CURRENT_LSB=%f, SHUNT_CAL=%u", this->current_lsb_, this->shunt_cal_);

  return this->write_u16_(RegisterMap::REG_SHUNT_CAL, this->shunt_cal_);
}

bool INA228Component::set_adc_range_(bool adc_range) {
  ConfigurationRegister cfg{0};
  auto ret = this->read_u16_(RegisterMap::REG_CONFIG, cfg.raw_u16);
  ESP_LOGD(TAG, "set_adc_range_ is %s, read: 0x%04X", TRUEFALSE(ret), cfg.raw_u16);
  cfg.ADCRANGE = adc_range;
  ret = ret && this->write_u16_(RegisterMap::REG_CONFIG, cfg.raw_u16);
  ESP_LOGD(TAG, "set_adc_range_ is %s, write: 0x%04X", TRUEFALSE(ret), cfg.raw_u16);
  this->adc_range_ = adc_range;

  return ret;
}

bool INA228Component::read_volt_shunt_(double &volt_out) {
  int32_t volt_reading = 0;
  auto ret = this->read_s20_4_(RegisterMap::REG_VSHUNT, volt_reading);
  ESP_LOGD(TAG, "read_volt_shunt_ is %s, 0x%08X", TRUEFALSE(ret), volt_reading);
  volt_out = (this->adc_range_ ? V_SHUNT_LSB_RANGE1 : V_SHUNT_LSB_RANGE0) * (double) volt_reading;

  return ret;
}

bool INA228Component::read_voltage_(double &volt_out) {
  int32_t volt_reading = 0;
  auto ret = this->read_s20_4_(RegisterMap::REG_VBUS, volt_reading);
  ESP_LOGD(TAG, "read_voltage_ is %s, 0x%08X", TRUEFALSE(ret), volt_reading);
  volt_out = VBUS_LSB * (double) volt_reading;

  return ret;
}

bool INA228Component::read_die_temp_(double &temp) {
  uint16_t temp_reading = 0;
  auto ret = this->read_u16_(RegisterMap::REG_DIETEMP, temp_reading);
  ESP_LOGD(TAG, "read_voltage_ is %s, 0x%04X", TRUEFALSE(ret), temp_reading);
  temp = DIE_TEMP_LSB * (double) temp_reading;

  return ret;
}

bool INA228Component::read_current_(double &amps_out) {
  int32_t amps_reading = 0;
  auto ret = this->read_s20_4_(RegisterMap::REG_CURRENT, amps_reading);
  ESP_LOGD(TAG, "read_voltage_ is %s, 0x%08X", TRUEFALSE(ret), amps_reading);
  amps_out = this->current_lsb_ * (double) amps_reading;

  return ret;
}

bool INA228Component::read_power_(double &power_out) {
  uint32_t power_reading = 0;
  auto ret = this->read_register((uint8_t) RegisterMap::REG_POWER, (uint8_t *) &power_reading, 3);
  ESP_LOGD(TAG, "read_power_1 ret= %d, 0x%08X", ret, power_reading);

  power_reading = byteswap(power_reading & 0xffffff) >> 8;
  ESP_LOGD(TAG, "read_power_2 ret= %d, 0x%08X", ret, power_reading);
  power_out = 3.2 * this->current_lsb_ * (double) power_reading;

  return ret == i2c::ERROR_OK;
}

bool INA228Component::read_energy_(double &joules_out) {
  uint64_t joules_reading = 0;  // Only 40 bits used
  auto ret = this->read_register((uint8_t) RegisterMap::REG_ENERGY, (uint8_t *) &joules_reading, 5);
  ESP_LOGD(TAG, "read_energy_1 ret= %d, 0x%" PRIX64, ret, joules_reading);

  joules_reading = byteswap(joules_reading & 0xffffffffffULL) >> 24;
  ESP_LOGD(TAG, "read_energy_2 ret= %d, 0x%" PRIX64, ret, joules_reading);
  joules_out = 16.0f * 3.2f * this->current_lsb_ * (double) joules_reading;

  return ret == i2c::ERROR_OK;
}

bool INA228Component::read_charge_(double &coulombs_out) {
  int64_t coulombs_reading = 0;  // Only 40 bits used
  auto ret = this->read_register((uint8_t) RegisterMap::REG_CHARGE, (uint8_t *) &coulombs_reading, 5);
  ESP_LOGD(TAG, "read_charge_1 is %s, 0x%" PRIX64, TRUEFALSE(ret), coulombs_reading);

  bool sign = coulombs_reading & 0x80;
  coulombs_reading = byteswap(coulombs_reading & 0xffffffffffULL) >> 24;
  if (sign)
    coulombs_reading += 0xFFFFFF0000000000;
  ESP_LOGD(TAG, "read_charge_2 is %s, 0x%" PRIX64, TRUEFALSE(ret), coulombs_reading);
  coulombs_out = this->current_lsb_ * (double) coulombs_reading;

  return ret;
}

bool INA228Component::clear_energy_counter_() {
  ConfigurationRegister cfg{0};
  ESP_LOGD(TAG, "clear_energy_counter_");
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

  if (this->die_temperature_sensor_ != nullptr) {
    double temp;
    if (!this->read_die_temp_(temp)) {
      all_okay = false;
      this->status_set_warning();
    }
    this->die_temperature_sensor_->publish_state(temp);
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
