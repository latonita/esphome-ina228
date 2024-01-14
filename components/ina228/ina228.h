#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace ina228 {

enum RegisterMap : uint8_t {
  REG_CONFIG = 0x00,
  REG_ADC_CONFIG = 0x01,
  REG_SHUNT_CAL = 0x02,
  REG_SHUNT_TEMPCO = 0x03,
  REG_VSHUNT = 0x04,
  REG_VBUS = 0x05,
  REG_DIETEMP = 0x06,
  REG_CURRENT = 0x07,
  REG_POWER = 0x08,
  REG_ENERGY = 0x09,
  REG_CHARGE = 0x0A,
  REG_DIAG_ALRT = 0x0B,
  REG_SOVL = 0x0C,
  REG_SUVL = 0x0D,
  REG_BOVL = 0x0E,
  REG_BUVL = 0x0F,
  REG_TEMP_LIMIT = 0x10,
  REG_PWR_LIMIT = 0x11,
  REG_MANUFACTURER_ID = 0x3E,
  REG_DEVICE_ID = 0x3F
};

// enum AdcRange : uint8_t {
//   ADC_RANGE_0 = 0,
//   ADC_RANGE_1 = 1,
// };

enum AdcSpeed : uint8_t {
  ADC_SPEED_50US = 0,
  ADC_SPEED_84US = 1,
  ADC_SPEED_150US = 2,
  ADC_SPEED_280US = 3,
  ADC_SPEED_540US = 4,
  ADC_SPEED_1052US = 5,
  ADC_SPEED_2074US = 6,
  ADC_SPEED_4120US = 7,
};

enum AdcSample : uint8_t {
  ADC_SAMPLE_1 = 0,
  ADC_SAMPLE_4 = 1,
  ADC_SAMPLE_16 = 2,
  ADC_SAMPLE_64 = 3,
  ADC_SAMPLE_128 = 4,
  ADC_SAMPLE_256 = 5,
  ADC_SAMPLE_512 = 6,
  ADC_SAMPLE_1024 = 7,
};

#pragma pack(push)
#pragma pack(1)
union ConfigurationRegister {
  uint16_t raw_u16;
  struct {
    uint8_t reserved_0_3 : 4;
    bool ADCRANGE;
    bool TEMPCOMP;
    uint8_t CONVDLY;  // Sets the Delay for initial ADC conversion in steps of 2 ms.
    bool RSTACC;
    bool RST;
  };
};

union AdcConfigurationRegister {
  uint16_t raw_u16;
  struct {
    AdcSample AVG : 3;
    AdcSpeed VTCT : 3;
    AdcSpeed VSHCT : 3;
    AdcSpeed VBUSCT : 3;
    uint8_t MODE : 4;
  };
};

#pragma pack(pop)

class INA228Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void set_shunt_resistance_ohm(float shunt_resistance_ohm) { shunt_resistance_ohm_ = shunt_resistance_ohm; }
  void set_max_current_a(float max_current_a) { max_current_a_ = max_current_a; }

  void set_shunt_voltage_sensor(sensor::Sensor *sensor) { shunt_voltage_sensor_ = sensor; }
  void set_bus_voltage_sensor(sensor::Sensor *sensor) { bus_voltage_sensor_ = sensor; }
  void set_die_temperature_sensor(sensor::Sensor *sensor) { die_temperature_sensor_ = sensor; }
  void set_current_sensor(sensor::Sensor *sensor) { current_sensor_ = sensor; }
  void set_power_sensor(sensor::Sensor *sensor) { power_sensor_ = sensor; }
  void set_energy_sensor(sensor::Sensor *sensor) { energy_sensor_ = sensor; }
  void set_charge_sensor(sensor::Sensor *sensor) { charge_sensor_ = sensor; }

 protected:
  bool configure_shunt_(double max_current, double r_shunt);
  bool set_adc_range_(bool adc_range);
  bool read_voltage_(double &volt_out);
  bool read_current_(double &amps_out);
  bool read_die_temp_(double &temp);
  bool read_volt_shunt_(double &volt_out);
  bool read_power_(double &power_out);
  bool read_energy_(double &joules_out);
  bool read_charge_(double &coulombs_out);
  bool clear_energy_counter_();

  bool read_u16_(uint8_t reg, uint16_t &out);
  bool read_s20_4_(uint8_t reg, int32_t &out);
  bool write_u16_(uint8_t reg, uint16_t val);
  

  double shunt_resistance_ohm_;
  double max_current_a_;
  bool adc_range_{false};
  uint8_t addr_msb_{0};
  uint16_t shunt_cal_{0};
  double current_lsb_{0};

  sensor::Sensor *shunt_voltage_sensor_{nullptr};
  sensor::Sensor *bus_voltage_sensor_{nullptr};
  sensor::Sensor *die_temperature_sensor_{nullptr};
  sensor::Sensor *current_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *energy_sensor_{nullptr};
  sensor::Sensor *charge_sensor_{nullptr};
};

}  // namespace ina228
}  // namespace esphome
