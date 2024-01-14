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
    shunt_resistance: 0.1 ohm
    shunt_voltage:
      name: "Shunt Voltage"
    bus_voltage:
      name: "Bus Voltage"
    current:
      name: "Current"
    power:
      name: "Power"
    energy:
      name: "Energy"
    charge:
      name: "Charge"
    update_interval: 60s



```