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

#define BIT(nr) (1UL << (nr))

void INA228Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up INA228...");

  this->configure_shunt_(this->max_current_a_, this->shunt_resistance_ohm_);
  ConfigurationRegister cfg{0};
  cfg.RST = 1;
  cfg = byteswap(cfg);
  auto ret = this->write_bytes(RegisterMap::REG_CONFIG, (uint8_t *) &cfg, 2);
  if (!ret) {
    ESP_LOGE(TAG, "Reset failed, check comm?");
    this->mark_failed();
    return;
  }
  delay(1);

  uint16_t manufacturer_id = 0, dev_id = 0;
  ret = ret && this->read_bytes(RegisterMap::REG_MANUFACTURER_ID, (uint8_t *) &manufacturer_id, 2);
  ret = ret && this->read_bytes(RegisterMap::REG_DEVICE_ID, (uint8_t *) &dev_id, 2);
  if (ret) {
    ESP_LOGE(TAG, "ID read failed");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Manufacturer: 0x%04X, Device ID: 0x%04X", manufacturer_id, dev_id);
  delay(1);
}

bool INA228Component::configure_shunt_(double max_current, double r_shunt) {
  this->current_lsb_ = max_current / (2 << 19);  // 2 power of 19
  this->shunt_cal_ = (uint16_t) ((double) (13107.2 * 1000000) * this->current_lsb_ * r_shunt);

  ESP_LOGI(TAG, "New Rshunt=%f ohm, max current=%.3f", r_shunt, max_current);
  ESP_LOGI(TAG, "New CURRENT_LSB=%f, SHUNT_CAL=%u", this->current_lsb_, this->shunt_cal_);

  return this->write_bytes((uint8_t) RegisterMap::REG_SHUNT_CAL, (uint8_t *) &this->shunt_cal_, 2);
}

bool INA228Component::set_adc_range_(bool adc_range) {
  ConfigurationRegister cfg{0};
  //  auto ret = this->read_byte_16((uint8_t) RegisterMap::REG_CONFIG, &cfg);
  auto ret = this->read_bytes((uint8_t) RegisterMap::REG_CONFIG, (uint8_t *) &cfg.raw, 2);
  cfg.raw = byteswap(cfg.raw);
  cfg.ADCRANGE = adc_range;

  cfg.raw = byteswap(cfg.raw);
  ret = ret && this->write_bytes((uint8_t) RegisterMap::REG_CONFIG, (uint8_t *) &cfg.raw, 2);

  this->adc_range_ = adc_range;
  return ret;
}

bool INA228Component::read_voltage_(double &volt_out) {
  int32_t volt_reading = 0;
  auto ret = this->read_bytes((uint8_t) RegisterMap::REG_VBUS, (uint8_t *) &volt_reading, 3);

  bool sign = volt_reading & 0x80;
  volt_reading = byteswap(volt_reading & 0xffffff) >> 4;
  if (sign)
    volt_reading *= -1;

  volt_out = (volt_reading) *VBUS_LSB;

  return ret;
}

bool INA228Component::read_current_(double &amps_out) {
  int32_t amps_reading = 0;
  auto ret = this->read_bytes((uint8_t) RegisterMap::REG_VBUS, (uint8_t *) &amps_reading, 3);

  bool sign = amps_reading & 0x80;
  amps_reading = byteswap(amps_reading & 0xffffff) >> 4;
  if (sign)
    amps_reading *= -1;
  amps_out = (amps_reading) * this->current_lsb_;

  return ret;
}

bool INA228Component::read_die_temp_(double &temp) {
  uint16_t temp_reading = 0;
  auto ret = this->read_bytes((uint8_t) RegisterMap::REG_DIETEMP, (uint8_t *) &temp_reading, 2);
  temp_reading = byteswap(temp_reading);
  temp = (double) temp_reading * DIE_TEMP_LSB;

  return ret;
}

bool INA228Component::read_volt_shunt_(double &volt_out) {
  int32_t volt_reading = 0;
  auto ret = this->read_bytes((uint8_t) RegisterMap::REG_VSHUNT, (uint8_t *) &volt_reading, 3);

  bool sign = volt_reading & 0x80;
  volt_reading = byteswap(volt_reading & 0xffffff) >> 4;
  if (sign)
    volt_reading *= -1;
  volt_out = (volt_reading) * (this->adc_range_ ? V_SHUNT_LSB_RANGE1 : V_SHUNT_LSB_RANGE0);

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
  auto ret = this->read_bytes((uint8_t) RegisterMap::REG_CONFIG, (uint8_t *) &cfg.raw, 2);
  cfg = byteswap(cfg);

  cfg.RSTACC = true;  // BIT(14);  // CONFIG->RSTACC
  cfg = byteswap(cfg);
  ret = ret && this->write_bytes((uint8_t) RegisterMap::REG_CONFIG, (uint8_t *) &cfg, 2);

  return ret;
}

// bool  INA228Component::read_alert_flag(uint16_t *flag_out)
// {
//     if (flag_out == nullptr) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     uint16_t buf = 0;
//     auto ret = read_u16((uint8_t)RegisterMap::REG_DIAG_ALRT, &buf);
//     if (ret != ESP_OK) {
//         return ret;
//     }

//     *flag_out = buf;
//     return ret;
// }

// bool  INA228Component::write_alert_flag(uint16_t flag)
// {
//     return write_u16((uint8_t)RegisterMap::REG_DIAG_ALRT, flag);
// }

/*

bool  INA228Component::write(uint8_t cmd, const uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(trans_buf, TRANS_SIZE);
    auto ret = i2c_master_start(handle);
    ret = ret ?: i2c_master_write_byte(handle, (addr_msb | I2C_MASTER_WRITE), true);
    ret = ret ?: i2c_master_write_byte(handle, cmd, true);
    ret = ret ?: i2c_master_write(handle, buf, len, true);
    ret = ret ?: i2c_master_stop(handle);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to prepare transaction: 0x%x", ret);
        return ret;
    }

    ret = i2c_master_cmd_begin(i2c_port, handle, wait_ticks);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send: 0x%x", ret);
        return ret;
    }

    return ESP_OK;
}



bool  INA228Component::write_u8(uint8_t cmd, uint8_t data)
{
    return write(cmd, (uint8_t *)&data, sizeof(data), wait_ticks);
}

bool  INA228Component::write_u16(uint8_t cmd, uint16_t data)
{
    uint16_t data_send = __bswap16(data);
    return write(cmd, (uint8_t *)&data_send, sizeof(data_send), wait_ticks);
}


bool  INA228Component::read_u8(uint8_t cmd, uint8_t *out)
{
    return read(cmd, out, sizeof(uint8_t), wait_ticks);
}

bool  INA228Component::read_u16(uint8_t cmd, uint16_t *out)
{
    if (out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t data_out = 0;
    auto ret = read(cmd, (uint8_t *)&data_out, sizeof(data_out), wait_ticks);
    if (ret != ESP_OK) {
        return ret;
    }

    *out = __bswap16(data_out);
    return ESP_OK;
}


*/

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
