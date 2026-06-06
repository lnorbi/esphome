import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import CONF_AUTO_MODE, ENTITY_CATEGORY_NONE

from .. import (
    CONF_SPRINKLIFY_CONTROLLER_ID,
    SprinklifyController,
    SprinklifyControllerItemBaseSchema,
    sprinklify_ns,
)

CODEOWNERS = ["@lnorbi"]
AutoModeSwitch = sprinklify_ns.class_(
    "AutoModeSwitch",
    switch.Switch,
    cg.Component,
    cg.Parented.template(SprinklifyController),
)
WinterModeSwitch = sprinklify_ns.class_(
    "WinterModeSwitch",
    switch.Switch,
    cg.Component,
    cg.Parented.template(SprinklifyController),
)

# Sprinklify switches
CONF_WINTER_MODE = "winter_mode"

# Additional icons
ICON_AUTO_MODE = "mdi:cog-play-outline"
ICON_WINTER_MODE = "mdi:snowflake"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_AUTO_MODE): switch.switch_schema(
                AutoModeSwitch,
                icon=ICON_AUTO_MODE,
                entity_category=ENTITY_CATEGORY_NONE,
                default_restore_mode="RESTORE_DEFAULT_ON",
            ),
            cv.Optional(CONF_WINTER_MODE): switch.switch_schema(
                WinterModeSwitch,
                icon=ICON_WINTER_MODE,
                entity_category=ENTITY_CATEGORY_NONE,
                default_restore_mode="RESTORE_DEFAULT_OFF",
            ),
        }
    ).extend(SprinklifyControllerItemBaseSchema)
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_SPRINKLIFY_CONTROLLER_ID])

    for switch_type in [CONF_AUTO_MODE, CONF_WINTER_MODE]:
        if conf := config.get(switch_type):
            sw_var = await switch.new_switch(conf)
            await cg.register_parented(sw_var, parent)
            await cg.register_component(sw_var, conf)
            # below a call is formulated like this: set_auto_mode_switch
            cg.add(getattr(parent, f"set_{switch_type}_switch")(sw_var))
