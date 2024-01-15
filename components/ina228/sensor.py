import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_ID,
    CONF_BUS_VOLTAGE,
    CONF_CURRENT,
    CONF_MAX_CURRENT,
    CONF_POWER,
    CONF_SHUNT_RESISTANCE,
    CONF_SHUNT_VOLTAGE,
    CONF_TEMPERATURE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_VOLT,
    UNIT_WATT,
)

CODEOWNERS = ["@latonita"]
DEPENDENCIES = ["i2c"]

UNIT_JOULE = "J"
UNIT_COULOMB = "C" 

CONF_CHARGE = "charge"
CONF_ENERGY = "energy"

ina228_ns = cg.esphome_ns.namespace("ina228")
INA228Component = ina228_ns.class_(
    "INA228Component", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(INA228Component),
            cv.Optional(CONF_SHUNT_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=5,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_BUS_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=5,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=5,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CURRENT): sensor.sensor_schema(
                unit_of_measurement=UNIT_AMPERE,
                accuracy_decimals=5,
                device_class=DEVICE_CLASS_CURRENT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_POWER): sensor.sensor_schema(
                unit_of_measurement=UNIT_WATT,
                accuracy_decimals=5,
                device_class=DEVICE_CLASS_POWER,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_ENERGY): sensor.sensor_schema(
                unit_of_measurement=UNIT_JOULE,
                accuracy_decimals=5,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_CHARGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_COULOMB,
                accuracy_decimals=5,
                device_class=DEVICE_CLASS_ENERGY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SHUNT_RESISTANCE, default=0.1): cv.All(
                cv.resistance, cv.Range(min=0.0)
            ),
            cv.Optional(CONF_MAX_CURRENT, default=10.0): cv.All(
                cv.current, cv.Range(min=0.0)
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x40))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_shunt_resistance_ohm(config[CONF_SHUNT_RESISTANCE]))
    cg.add(var.set_max_current_a(config[CONF_MAX_CURRENT]))

    if conf := config.get(CONF_SHUNT_VOLTAGE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_shunt_voltage_sensor(sens))

    if conf := config.get(CONF_BUS_VOLTAGE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_bus_voltage_sensor(sens))

    if conf := config.get(CONF_TEMPERATURE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_die_temperature_sensor(sens))

    if conf := config.get(CONF_CURRENT):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_current_sensor(sens))

    if conf := config.get(CONF_POWER):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_power_sensor(sens))

    if conf := config.get(CONF_ENERGY):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_energy_sensor(sens))

    if conf := config.get(CONF_CHARGE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_charge_sensor(sens))
