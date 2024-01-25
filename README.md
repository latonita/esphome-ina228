INA228 85-V, 20-Bit, Ultra-Precise Power/Energy/Charge Monitor With I2C Interface

I2C versions (INA228, INA237, INA238)
SPI versions (INA229, INA239)


EspHome perliminary component, before submitting PR
```
# Example configuration

esphome:
  name: ina228-test

esp32:
  board: esp32dev
  framework:
    type: arduino

# esp8266:
#   board: nodemcuv2

logger:
  level: DEBUG

external_components:
  source: github://latonita/esphome-ina228
  components: [ina2xx_base, ina2xx_i2c]

i2c:
    #...
spi:
    #...
sensor:
  - platform: ina2xx_i2c
    address: 0x40
    shunt_resistance: 0.001100 ohm
    max_current: 40 A

    # adc_range 0 -> shunt V ±163.84mV , default
    # adc_range 1 -> shunt V ±40.96mV
    adc_range: 0

    # shunt resistor temp coefficient, ppm/�C, integer 0 - 16383
    #temperature_coefficient: 0
    
    update_interval: 60s
    shunt_voltage:
      name: "INA228 Shunt Voltage"
    bus_voltage:
      name: "INA228 Bus Voltage"
    current:
      name: "INA228 Current"
    power:
      name: "INA228 Power"
    energy:
      name: "INA228 Energy"
    charge:
      name: "INA228 Charge"

sensor:
  - platform: ina2xx_spi
    cs_pin: GPIO22
    shunt_resistance: 0.001100 ohm
    max_current: 40 A

    # adc_range 0 -> shunt V ±163.84mV , default
    # adc_range 1 -> shunt V ±40.96mV
    adc_range: 0

    # shunt resistor temp coefficient, ppm/�C, integer 0 - 16383
    #temperature_coefficient: 0
    
    update_interval: 60s
    shunt_voltage:
      name: "INA228 Shunt Voltage"
    bus_voltage:
      name: "INA228 Bus Voltage"
    current:
      name: "INA228 Current"
    power:
      name: "INA228 Power"
    energy:
      name: "INA228 Energy"
    charge:
      name: "INA228 Charge"

```
