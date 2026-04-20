import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from .. import (
    CONF_SPRINKLIFY_CONTROLLER_ID,
    SprinklifyController,
    SprinklifyControllerItemBaseSchema,
)

CODEOWNERS = ["@lnorbi"]
BinarySensorTypeEnum = SprinklifyController.enum("BinarySensorType", True)

# Controller binary sensors
CONF_FLOW_OK = "flow_ok"
CONF_PRESSURE_OK = "pressure_ok"

# Additional icons
ICON_GAUGE = "mdi:gauge"

SENSOR_TYPES = {
    CONF_FLOW_OK: binary_sensor.binary_sensor_schema(
        icon=ICON_GAUGE,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    CONF_PRESSURE_OK: binary_sensor.binary_sensor_schema(
        icon=ICON_GAUGE,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
}

CONFIG_SCHEMA = SprinklifyControllerItemBaseSchema.extend(
    {cv.Optional(type): schema for type, schema in SENSOR_TYPES.items()}
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_SPRINKLIFY_CONTROLLER_ID])
    # cg.add_define("USE_SPRINKLIFY_BINARY_SENSOR")
    for type_ in SENSOR_TYPES:
        if conf := config.get(type_):
            sens = await binary_sensor.new_binary_sensor(conf)
            binary_sensor_type = getattr(BinarySensorTypeEnum, type_.upper())
            cg.add(paren.set_binary_sensor(binary_sensor_type, sens))
