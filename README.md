INA2xx family of digital power monitors
=======================================

**This is EspHome perliminary component, before [PR#6138](https://github.com/esphome/esphome/pull/6138) is merged in main branch**

Supported devices
-----------------
The ``ina2xx`` sensor platform allows you to use family of Texas Instruments current and power 
sensors with ESPHome.

- [INA228](http://www.ti.com/lit/ds/symlink/ina228.pdf), I²C 85-V, 20-Bit, Ultra-Precise Power/Energy/Charge Monitor
- [INA229](http://www.ti.com/lit/ds/symlink/ina229.pdf>), SPI 85-V, 20-Bit, Ultra-Precise Power/Energy/Charge Monitor
- [INA238](http://www.ti.com/lit/ds/symlink/ina238.pdf>), I²C 85-V, 16-Bit, High-Precision Power Monitor
- [INA239](http://www.ti.com/lit/ds/symlink/ina239.pdf>), SPI 85-V, 16-Bit, High-Precision Power Monitor
- [INA237](http://www.ti.com/lit/ds/symlink/ina237.pdf>), I²C 85-V, 16-Bit, Precision Power Monitor


# Example configuration entry for I²C
```
external_components:
  source: github://latonita/esphome-ina228
  components: [ina2xx_base, ina2xx_i2c]

i2c:
  sda: D1
  scl: D2
    
sensor:
  - platform: ina2xx_i2c
    id: my_change_sensor
    model: INA228
    address: 0x40
    shunt_resistance: 0.010 ohm
    max_current: 10 A
    adc_time: 4120 us
    adc_averaging: 128
    update_interval: 60s
    current:
      name: "INA2xx Current"
    bus_voltage:
      name: "INA2xx Bus Voltage"
    charge:
      name: "INA2xx Charge"

```
# Example configuration entry for SPI
```
external_components:
  source: github://latonita/esphome-ina228
  components: [ina2xx_base, ina2xx_i2c]

spi:
  clk_pin: D0
  mosi_pin: D1
  miso_pin: D2
    
sensor:
  - platform: ina2xx_spi
    model: INA229
    cs_pin: D3
    shunt_resistance: 0.001130 ohm
    max_current: 40 A
    adc_range: 0
    adc_time: 4120 us
    adc_averaging: 128
    temperature_coefficient: 50
    current:
      name: "INA2xx Current"
    power:
      name: "INA2xx Power"
```

Configuration variables:
------------------------

- **model** (**Required): The model of the INA2xx sensor. For I2C valid options are ``INA228``, ``INA237``, ``INA238``,
  for SPI options are ``INA229`` and ``INA239``.
- **shunt_resistance** (float): The value of the shunt resistor used for current calculation. No default value.
- **max_current** (float): The maximum current you are expecting. Component will use it to 
  calibrate the sensor. No default value.
- **adc_range** (*Optional*, ``0`` or ``1``): Selects the range for differential input across shunt
  resistor. ``0`` for ±163.84 mV, ``1`` for ±40.96 mV range. Defaults to ``0``.
- **temperature_coefficient** (*Optional*, integer from ``0`` to ``16383``): Temperature coefficient (ppm/°C) of the 
  shunt for temperature compensation correction. Only applicable to INA228 and INA229 devices. Zero value means 
  no compensation is done. Defaults to ``0``.
- **update_interval** (*Optional*, :ref:`config-time`): The interval to check the sensor. Defaults to ``60s``.
- All other options for SPI/I²C devices as descibed in respective documentation.


Sensors
-------
The component offers nine sensors. You can configure all or any subset of the sensors. Each configured sensor 
is reported  separately on each update_interval. The ``name`` option is required for each sensor configured. 
All other options from :ref:`Sensor <config-sensor>`.

- **shunt_voltage** (*Optional*): Differential voltage measured across the shunt, mV
- **bus_voltage** (*Optional*): Bus voltage output, V
- **temperature** (*Optional*): Internal die temperature measurement, °C
- **current** (*Optional*): Calculated current output, A
- **power** (*Optional*): Calculated power output, W
- **energy** (*Optional*): Calculated energy output, Wh (*INA228/229 only*)
- **energy_joules** (*Optional*): Calculated energy output, Joules (*INA228/229 only*)
- **charge** (*Optional*): Calculated charge output, Ah (*INA228/229 only*)
- **charge_coulombs** (*Optional*): Calculated charge output, Coulombs (*INA228/229 only*)

Lambda calls
------------

The component exposes one function to reset INA228/INA229 energy and charge accumulators.

- ``reset_energy_counters()``

```
      // Within lambda, reset counters.
      id(my_charge_sensor).reset_energy_counters();
```
To simplify the use of this function, you can use the following example to add a button to reset the counters.

```

      button:
        - platform: template
          name: "Reset counters"
          on_press:
            - lambda: "id(my_change_sensor).reset_energy_counters();"

```