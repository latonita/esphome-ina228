INA228 85-V, 20-Bit, Ultra-Precise Power/Energy/Charge Monitor With I2C Interface

EspHome perliminary component
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
  components: [ina228]

i2c:

sensor:
  - platform: ina228
    address: 0x40
    shunt_resistance: 0.001 ohm
    max_current: 10 A
    
    # adc_range 0 -> shunt V ±163.84mV 
    # adc_range 1 -> shunt V ±40.96mV
    adc_range: 0
    
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