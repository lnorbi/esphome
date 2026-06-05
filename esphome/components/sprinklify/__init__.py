from esphome import core
import esphome.codegen as cg
from esphome.components import (
    binary_sensor,
    button,
    number,
    output,
    sensor,
    switch as esphome_switch,
    text_sensor,
    time,
)
from esphome.components.pulse_meter.sensor import PulseMeterSensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_HOUR,
    CONF_ID,
    CONF_INITIAL_VALUE,
    CONF_LED,
    CONF_MAX_VALUE,
    CONF_MIN_VALUE,
    CONF_MINUTE,
    CONF_NAME,
    CONF_RESTORE_MODE,
    CONF_RESTORE_VALUE,
    CONF_SECOND,
    CONF_STEP,
    CONF_TIME_ID,
    CONF_UNIT_OF_MEASUREMENT,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_PRESSURE,
    DEVICE_CLASS_SWITCH,
    DEVICE_CLASS_VOLUME,
    DEVICE_CLASS_VOLUME_FLOW_RATE,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ENTITY_CATEGORY_NONE,
    ICON_POWER,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_LITRE,
    UNIT_MINUTE,
)

AUTO_LOAD = ["number", "button", "text_sensor"]

# Pressure sensor sub-schema
CONF_PRESSURE_INPUT = "pressure_input"
CONF_PRESSURE_CALIBRATION = "pressure_calibration"
CONF_V_AT_MIN_PRESSURE = "v_at_min_pressure"
CONF_V_AT_MAX_PRESSURE = "v_at_max_pressure"
CONF_P_MAX = "p_max"
CONF_OPAMP_OUTPUT_AT_5V = "opamp_output_at_5v"
CONF_EMA_ALPHA = "ema_alpha"
# Pressure slope / direction
CONF_SLOPE_SENSOR = "slope_sensor"
CONF_DIRECTION_SENSOR = "direction_sensor"
CONF_STABLE_THRESHOLD = "stable_threshold"

# Flow sensor sub-schema
CONF_FLOW_INPUT = "flow_input"
CONF_TOTAL_INPUT = "total_input"

# Controller-level config keys
CONF_CONTROLLER_STATE_SENSOR = "controller_state_sensor"
CONF_PRESSURE_SENSOR = "pressure_sensor"
CONF_FLOW_SENSOR = "flow_sensor"
CONF_ENABLE_SWITCH = "enable_switch"
CONF_EMPTY_PRESSURE = "empty_pressure"
CONF_PUMP_START_PRESSURE = "pump_start_pressure"
CONF_MAX_PRESSURE = "max_pressure"
CONF_PULSES_PER_LITER = "pulses_per_liter"
CONF_MASTER_TRIGGER = "master_trigger"
CONF_MIN_FLOW_THRESHOLD = "min_flow_threshold"
CONF_NO_FLOW_SAFETY_TIMEOUT = "no_flow_safety_timeout"
CONF_DRY_RUN_TIMEOUT = "dry_run_timeout"
CONF_INTERLOCK_DELAY = "interlock_delay"
CONF_PUMP_UNBLOCK_AT = "pump_unblock_at"
CONF_STATUS_LED_RED = "status_led_red"
CONF_STATUS_LED_GREEN = "status_led_green"
CONF_PRESSURE_LED = "pressure_led"
CONF_FLOW_LED = "flow_led"
CONF_PUMPS = "pumps"
# Controller-level number overrides
CONF_PUMP_START_PRESSURE_NUMBER = "pump_start_pressure_number"
CONF_MAX_PRESSURE_NUMBER = "max_pressure_number"

# Per-pump config key names
CONF_SPRINKLIFY_CONTROLLER_ID = "sprinklify_controller_id"
CONF_RELAY = "relay"
CONF_STATE_SENSOR = "state_sensor"
CONF_MAX_RUNTIME = "max_runtime"
CONF_AUTO_RESET_WAIT_TIME = "auto_reset_wait_time"
CONF_INSTALLED_SWITCH = "installed_switch"
CONF_RESET_BUTTON = "reset_button"
CONF_RUN_SWITCH = "run_switch"
CONF_TOTAL_VOLUME_SENSOR = "total_volume_sensor"
CONF_TOTAL_RUNTIME_SENSOR = "total_runtime_sensor"
# Per-pump number overrides
CONF_MAX_RUNTIME_NUMBER = "max_runtime_number"
CONF_AUTO_RESET_WAIT_TIME_NUMBER = "auto_reset_wait_time_number"

# Forced restore mode for some switches
FORCED_RESTORE_MODE = "ALWAYS_OFF"

# Additional icons
ICON_POWER_PLUG = "mdi:power-plug-outline"

# ---------------------------------------------------------------------------
# Namespace and component class declarations
# ---------------------------------------------------------------------------

sprinklify_ns = cg.esphome_ns.namespace("sprinklify")
SprinklifyController = sprinklify_ns.class_("SprinklifyController", cg.Component)

# Child entities
SprinklifyPressureSensor = sprinklify_ns.class_(
    "SprinklifyPressureSensor",
    cg.Component,
    sensor.Sensor,
    cg.Parented.template(SprinklifyController),
)
SprinklifyFlowSensor = sprinklify_ns.class_(
    "SprinklifyFlowSensor",
    cg.Component,
    sensor.Sensor,
    cg.Parented.template(SprinklifyController),
)
InstalledSwitch = sprinklify_ns.class_(
    "InstalledSwitch", esphome_switch.Switch, cg.Parented.template(SprinklifyController)
)
PumpRunSwitch = sprinklify_ns.class_(
    "PumpRunSwitch", esphome_switch.Switch, cg.Parented.template(SprinklifyController)
)
PumpResetButton = sprinklify_ns.class_(
    "PumpResetButton", button.Button, cg.Parented.template(SprinklifyController)
)
# Number entity class
SprinklifyNumber = sprinklify_ns.class_(
    "SprinklifyNumber",
    cg.Component,
    number.Number,
    cg.Parented.template(SprinklifyController),
)

# ---------------------------------------------------------------------------
# Schema helpers
# ---------------------------------------------------------------------------

PRESSURE_CALIBRATION_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_V_AT_MIN_PRESSURE): cv.All(
            cv.voltage, cv.float_range(min=0.1, max=1.0)
        ),
        cv.Required(CONF_V_AT_MAX_PRESSURE): cv.All(
            cv.voltage, cv.float_range(min=4.0, max=5.0)
        ),
        cv.Required(CONF_P_MAX): cv.All(cv.pressure, cv.float_range(min=4.0, max=16.0)),
        cv.Required(CONF_OPAMP_OUTPUT_AT_5V): cv.All(
            cv.voltage, cv.float_range(min=0.5, max=2.0)
        ),
    }
)

# The pressure sensor is a self-contained sub-schema:
#   - sensor.sensor_schema provides name, icon, filters, etc.
#   - pressure_input references the raw ADC sensor
#   - calibration captures the sensor + OpAmp physical characteristics
# device_class, unit, and accuracy are hardcoded — they are facts about
# what this entity always represents, not user choices.
PRESSURE_SENSOR_SCHEMA = sensor.sensor_schema(
    SprinklifyPressureSensor,
    device_class=DEVICE_CLASS_PRESSURE,
    entity_category=ENTITY_CATEGORY_NONE,
    unit_of_measurement="bar",
    accuracy_decimals=2,
).extend(
    {
        cv.Required(CONF_PRESSURE_INPUT): cv.use_id(sensor.Sensor),
        cv.Required(CONF_PRESSURE_CALIBRATION): PRESSURE_CALIBRATION_SCHEMA,
        cv.Optional(CONF_EMA_ALPHA, default=0.1): cv.float_range(min=0.01, max=1.0),
        # Slope / direction — optional; omit if not needed in HA or by hub logic
        cv.Optional(CONF_SLOPE_SENSOR): sensor.sensor_schema(
            sensor.Sensor,
            unit_of_measurement="bar/s",
            accuracy_decimals=3,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DIRECTION_SENSOR): text_sensor.text_sensor_schema(
            text_sensor.TextSensor, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
        ),
        # Rate below which change is considered stable, in bar/s.
        # Tune to your sensor noise floor.
        cv.Optional(CONF_STABLE_THRESHOLD, default=0.05): cv.positive_float,
    }
)

FLOW_SENSOR_SCHEMA = sensor.sensor_schema(
    SprinklifyFlowSensor,
    unit_of_measurement="L/min",
    accuracy_decimals=2,
    device_class=DEVICE_CLASS_VOLUME_FLOW_RATE,
    entity_category=ENTITY_CATEGORY_NONE,
    state_class=STATE_CLASS_MEASUREMENT,
).extend(
    {
        cv.Required(CONF_FLOW_INPUT): cv.use_id(PulseMeterSensor),
        cv.Required(CONF_TOTAL_INPUT): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_PULSES_PER_LITER, default=360): cv.float_range(min=1.0),
    }
)

SprinklifyControllerItemBaseSchema = cv.Schema(
    {
        cv.GenerateID(CONF_SPRINKLIFY_CONTROLLER_ID): cv.use_id(SprinklifyController),
    }
)


# Number schema helper — unit/min/max/step defined per parameter
def sprinklify_number_schema(
    unit, min_val, max_val, step, initial=None, validator=cv.positive_float
):
    return number.number_schema(
        SprinklifyNumber, entity_category=ENTITY_CATEGORY_CONFIG
    ).extend(
        {
            cv.Optional(CONF_UNIT_OF_MEASUREMENT, default=unit): cv.string,
            cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
            cv.Optional(CONF_MIN_VALUE, default=min_val): validator,
            cv.Optional(CONF_MAX_VALUE, default=max_val): validator,
            cv.Optional(CONF_STEP, default=step): validator,
            cv.Optional(CONF_INITIAL_VALUE, default=initial or min_val): validator,
        }
    )


def validate_config(config):
    if config[CONF_PUMP_START_PRESSURE] <= config[CONF_EMPTY_PRESSURE]:
        raise cv.Invalid(
            f"{CONF_PUMP_START_PRESSURE} must be greater than {CONF_EMPTY_PRESSURE}"
        )
    if config[CONF_PUMP_START_PRESSURE] >= config[CONF_MAX_PRESSURE]:
        raise cv.Invalid(
            f"{CONF_PUMP_START_PRESSURE} must be less than {CONF_MAX_PRESSURE}"
        )
    return config


def validate_pump_config(pump_config):
    if not (run_switch_config := pump_config.get(CONF_RUN_SWITCH)):
        return pump_config

    restore_mode = run_switch_config.get(CONF_RESTORE_MODE)
    switch_name = run_switch_config.get(CONF_NAME, "Unknown")
    if restore_mode != FORCED_RESTORE_MODE:
        raise cv.Invalid(
            f"Pump switch '{switch_name}': "
            f"{CONF_RUN_SWITCH}.{CONF_RESTORE_MODE} is fixed to "
            f"{FORCED_RESTORE_MODE}."
        )
    return pump_config


# ---------------------------------------------------------------------------
# Per-pump schema
# ---------------------------------------------------------------------------
PUMP_SCHEMA = cv.All(
    cv.Schema(
        {
            # Hardware
            cv.Required(CONF_RELAY): cv.use_id(output.BinaryOutput),
            cv.Required(CONF_LED): cv.use_id(output.BinaryOutput),
            # Reporting & control
            cv.Optional(CONF_STATE_SENSOR): text_sensor.text_sensor_schema(
                text_sensor.TextSensor, icon="mdi:status"
            ),
            cv.Optional(CONF_INSTALLED_SWITCH): esphome_switch.switch_schema(
                InstalledSwitch,
                device_class=DEVICE_CLASS_SWITCH,
                entity_category=ENTITY_CATEGORY_NONE,
                default_restore_mode="RESTORE_DEFAULT_ON",
                icon=ICON_POWER_PLUG,
            ),
            cv.Optional(CONF_RUN_SWITCH): esphome_switch.switch_schema(
                PumpRunSwitch,
                device_class=DEVICE_CLASS_SWITCH,
                entity_category=ENTITY_CATEGORY_NONE,
                default_restore_mode=FORCED_RESTORE_MODE,
                icon=ICON_POWER,
            ),
            cv.Optional(CONF_RESET_BUTTON): button.button_schema(
                PumpResetButton,
                entity_category=ENTITY_CATEGORY_NONE,
            ),
            # Compile-time defaults
            cv.Optional(CONF_MAX_RUNTIME, default="120m"): cv.All(
                cv.positive_time_period_minutes,
                cv.Range(min=core.TimePeriod(minutes=30), max=core.TimePeriod(hours=4)),
            ),
            cv.Optional(CONF_AUTO_RESET_WAIT_TIME): cv.All(
                cv.positive_time_period_minutes,
                cv.Range(min=core.TimePeriod(hours=3), max=core.TimePeriod(hours=48)),
            ),
            # Runtime-configurable options
            cv.Optional(CONF_MAX_RUNTIME_NUMBER): sprinklify_number_schema(
                unit="min",
                min_val=30,
                max_val=240,
                step=10,
                initial=120,
                validator=cv.positive_int,
            ),
            cv.Optional(CONF_AUTO_RESET_WAIT_TIME_NUMBER): sprinklify_number_schema(
                unit="h",
                min_val=3,
                max_val=48,
                step=1,
                initial=24,
                validator=cv.positive_int,
            ),
            # Pump statistics
            cv.Optional(CONF_TOTAL_VOLUME_SENSOR): sensor.sensor_schema(
                sensor.Sensor,
                device_class=DEVICE_CLASS_VOLUME,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                unit_of_measurement=UNIT_LITRE,
                accuracy_decimals=1,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
            cv.Optional(CONF_TOTAL_RUNTIME_SENSOR): sensor.sensor_schema(
                sensor.Sensor,
                device_class=DEVICE_CLASS_DURATION,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                unit_of_measurement=UNIT_MINUTE,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
        }
    ),
    validate_pump_config,
)


# ---------------------------------------------------------------------------
# Controller-level schema
# ---------------------------------------------------------------------------

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SprinklifyController),
            cv.Required(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Required(CONF_PRESSURE_SENSOR): PRESSURE_SENSOR_SCHEMA,
            cv.Required(CONF_FLOW_SENSOR): FLOW_SENSOR_SCHEMA,
            cv.Optional(CONF_CONTROLLER_STATE_SENSOR): text_sensor.text_sensor_schema(
                text_sensor.TextSensor,
                entity_category=ENTITY_CATEGORY_NONE,  # ← explicitly visible
            ),
            cv.Optional(CONF_STATUS_LED_GREEN): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_STATUS_LED_RED): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_PRESSURE_LED): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_FLOW_LED): cv.use_id(output.BinaryOutput),
            cv.Optional(CONF_MASTER_TRIGGER): cv.use_id(binary_sensor.BinarySensor),
            cv.Optional(CONF_EMPTY_PRESSURE, default=0.1): cv.All(
                cv.pressure, cv.Range(min=0.0, max=1.0)
            ),
            cv.Optional(CONF_PUMP_START_PRESSURE, default=2.5): cv.All(
                cv.pressure, cv.Range(min=1.0, max=4.0)
            ),
            cv.Optional(CONF_MAX_PRESSURE, default=6.0): cv.All(
                cv.pressure, cv.Range(min=4.5, max=10.0)
            ),
            cv.Optional(CONF_MIN_FLOW_THRESHOLD, default=15.0): cv.All(
                cv.float_with_unit("flow rate", "(l/min|L/min)", optional_unit=True),
                cv.Range(min=5.0, max=25.0),
            ),
            cv.Optional(CONF_NO_FLOW_SAFETY_TIMEOUT, default="60s"): cv.All(
                cv.positive_time_period_seconds,
                cv.Range(
                    min=core.TimePeriod(seconds=10), max=core.TimePeriod(seconds=90)
                ),
            ),
            cv.Optional(CONF_DRY_RUN_TIMEOUT, default="5s"): cv.All(
                cv.positive_time_period_seconds,
                cv.Range(
                    min=core.TimePeriod(seconds=4), max=core.TimePeriod(seconds=10)
                ),
            ),
            cv.Required(CONF_PUMP_UNBLOCK_AT): cv.time_of_day,
            cv.Required(CONF_INTERLOCK_DELAY): cv.All(
                cv.positive_time_period_seconds,
                cv.Range(
                    min=core.TimePeriod(seconds=1),
                    max=core.TimePeriod(seconds=10),
                ),
            ),
            cv.Optional(CONF_PUMP_START_PRESSURE_NUMBER): sprinklify_number_schema(
                "bar",
                min_val=1.0,
                max_val=4.0,
                step=0.1,
                initial=2.5,
                validator=cv.positive_float,
            ),
            cv.Optional(CONF_MAX_PRESSURE_NUMBER): sprinklify_number_schema(
                "bar",
                min_val=4.5,
                max_val=10.0,
                step=0.1,
                initial=6.0,
                validator=cv.positive_float,
            ),
            cv.Required(CONF_PUMPS): cv.All(
                cv.ensure_list(PUMP_SCHEMA),
                cv.Length(min=1, max=3),
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    validate_config,
)


# ---------------------------------------------------------------------------
# Codegen helpers
# ---------------------------------------------------------------------------
async def new_sprinklify_number(conf, hub_id, initial_value_override=None):
    """
    Creates and wires a SprinklifyNumber entity.

    initial_value_override: when supplied, overrides the schema's own
    CONF_INITIAL_VALUE. Used for per-pump numbers where the compile-time
    default comes from the pump's time-period config (converted to the
    number's unit) rather than the number schema's own default.
    """
    initial = (
        initial_value_override
        if initial_value_override is not None
        else conf[CONF_INITIAL_VALUE]
    )
    num = await number.new_number(
        conf,
        min_value=conf[CONF_MIN_VALUE],
        max_value=conf[CONF_MAX_VALUE],
        step=conf[CONF_STEP],
    )
    await cg.register_component(num, conf)
    await cg.register_parented(num, hub_id)
    cg.add(num.set_initial_value(initial))
    return num


# ---------------------------------------------------------------------------
# to_code
# ---------------------------------------------------------------------------


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # Emit pump count define first — sizes the fixed C++ arrays
    num_pumps = len(config[CONF_PUMPS])
    cg.add_define("SPRINKLIFY_PUMP_COUNT", num_pumps)
    print(f"[sprinklify] SPRINKLIFY_PUMP_COUNT = {num_pumps}")

    cg.add(var.set_time(await cg.get_variable(config[CONF_TIME_ID])))

    # --- Pressure sensor — dedicated class, self-contained signal chain ---
    # new_sensor() registers the HA entity; register_component() ensures setup()
    # is called so the ADC callback is wired before the first reading arrives.
    pres_conf = config[CONF_PRESSURE_SENSOR]
    pres_sensor = await sensor.new_sensor(pres_conf)
    await cg.register_component(pres_sensor, pres_conf)
    await cg.register_parented(pres_sensor, config[CONF_ID])
    cg.add(
        pres_sensor.set_pressure_input(
            await cg.get_variable(pres_conf[CONF_PRESSURE_INPUT])
        )
    )
    cal = pres_conf[CONF_PRESSURE_CALIBRATION]
    cg.add(
        pres_sensor.set_calibration(
            cal[CONF_V_AT_MIN_PRESSURE],
            cal[CONF_V_AT_MAX_PRESSURE],
            cal[CONF_P_MAX],
            cal[CONF_OPAMP_OUTPUT_AT_5V],
        )
    )
    # Optional slope sensor
    if CONF_SLOPE_SENSOR in pres_conf:
        slope_sens = await sensor.new_sensor(pres_conf[CONF_SLOPE_SENSOR])
        cg.add(pres_sensor.set_slope_sensor(slope_sens))
    # Slope stable threshold — always present (has default)
    cg.add(pres_sensor.set_stable_threshold(pres_conf[CONF_STABLE_THRESHOLD]))
    # EMA alpha configuration
    cg.add(pres_sensor.set_ema_alpha(pres_conf[CONF_EMA_ALPHA]))
    # Optional direction text sensor
    if CONF_DIRECTION_SENSOR in pres_conf:
        dir_sens = await text_sensor.new_text_sensor(pres_conf[CONF_DIRECTION_SENSOR])
        cg.add(pres_sensor.set_direction_sensor(dir_sens))
    # Register with controller
    cg.add(var.set_pressure_sensor(pres_sensor))

    # Flow sensor
    flow_conf = config[CONF_FLOW_SENSOR]
    flow_sensor = await sensor.new_sensor(flow_conf)
    await cg.register_component(flow_sensor, flow_conf)
    await cg.register_parented(flow_sensor, config[CONF_ID])
    cg.add(
        flow_sensor.set_pulse_meter(await cg.get_variable(flow_conf[CONF_FLOW_INPUT]))
    )
    cg.add(
        flow_sensor.set_pulse_total_sensor(
            await cg.get_variable(flow_conf[CONF_TOTAL_INPUT])
        )
    )
    cg.add(flow_sensor.set_pulses_per_liter(flow_conf[CONF_PULSES_PER_LITER]))
    # Register with controller
    cg.add(var.set_flow_sensor(flow_sensor))

    # Master trigger input
    if CONF_MASTER_TRIGGER in config:
        cg.add(
            var.set_master_trigger(await cg.get_variable(config[CONF_MASTER_TRIGGER]))
        )

    # Various settings
    cg.add(var.set_empty_pressure(config[CONF_EMPTY_PRESSURE]))
    cg.add(var.set_pump_start_pressure(config[CONF_PUMP_START_PRESSURE]))
    cg.add(var.set_max_pressure(config[CONF_MAX_PRESSURE]))
    cg.add(var.set_min_flow(config[CONF_MIN_FLOW_THRESHOLD]))
    cg.add(
        var.set_no_flow_safety_timeout(
            config[CONF_NO_FLOW_SAFETY_TIMEOUT].total_milliseconds
        )
    )
    cg.add(var.set_dry_run_timeout(config[CONF_DRY_RUN_TIMEOUT].total_milliseconds))
    cg.add(var.set_interlock_delay(config[CONF_INTERLOCK_DELAY].total_milliseconds))
    if CONF_PUMP_UNBLOCK_AT in config:
        unblock_at = config[CONF_PUMP_UNBLOCK_AT]
        cg.add(
            var.set_unblock_time(
                unblock_at[CONF_HOUR], unblock_at[CONF_MINUTE], unblock_at[CONF_SECOND]
            )
        )
    if CONF_PUMP_START_PRESSURE_NUMBER in config:
        num = await new_sprinklify_number(
            config[CONF_PUMP_START_PRESSURE_NUMBER], config[CONF_ID]
        )
        cg.add(var.set_pump_start_pressure_number(num))

    if CONF_MAX_PRESSURE_NUMBER in config:
        num = await new_sprinklify_number(
            config[CONF_MAX_PRESSURE_NUMBER], config[CONF_ID]
        )
        cg.add(var.set_max_pressure_number(num))

    if CONF_CONTROLLER_STATE_SENSOR in config:
        state_sens = await text_sensor.new_text_sensor(
            config[CONF_CONTROLLER_STATE_SENSOR]
        )
        cg.add(var.set_controller_state_sensor(state_sens))

    if CONF_STATUS_LED_GREEN in config:
        green_led_output = await cg.get_variable(config[CONF_STATUS_LED_GREEN])
        cg.add(var.set_status_led_green_output(green_led_output))
    if CONF_STATUS_LED_RED in config:
        red_led_output = await cg.get_variable(config[CONF_STATUS_LED_RED])
        cg.add(var.set_status_led_red_output(red_led_output))
    if CONF_PRESSURE_LED in config:
        press_led_output = await cg.get_variable(config[CONF_PRESSURE_LED])
        cg.add(var.set_pressure_led_output(press_led_output))
    if CONF_FLOW_LED in config:
        flow_led_output = await cg.get_variable(config[CONF_FLOW_LED])
        cg.add(var.set_flow_led_output(flow_led_output))

    # --- Per-pump configuration ---
    for i, pump_conf in enumerate(config[CONF_PUMPS]):
        relay = await cg.get_variable(pump_conf[CONF_RELAY])
        led = await cg.get_variable(pump_conf[CONF_LED])
        if CONF_AUTO_RESET_WAIT_TIME in pump_conf:
            auto_reset = pump_conf[CONF_AUTO_RESET_WAIT_TIME].total_milliseconds
        else:
            auto_reset = 0
        cg.add(
            var.add_pump(
                i,
                relay,
                led,
                pump_conf[CONF_MAX_RUNTIME].total_milliseconds,
                auto_reset,
            )
        )
        if CONF_STATE_SENSOR in pump_conf:
            sens = await text_sensor.new_text_sensor(pump_conf[CONF_STATE_SENSOR])
            cg.add(var.set_pump_state_sensor(i, sens))

        if CONF_RESET_BUTTON in pump_conf:
            btn = await button.new_button(pump_conf[CONF_RESET_BUTTON])
            await cg.register_parented(btn, config[CONF_ID])
            cg.add(btn.set_pump_index(i))

        if CONF_INSTALLED_SWITCH in pump_conf:
            sw = await esphome_switch.new_switch(pump_conf[CONF_INSTALLED_SWITCH])
            # await cg.register_component(sw, pump_conf[CONF_INSTALLED_SWITCH])
            await cg.register_parented(sw, config[CONF_ID])
            cg.add(sw.set_pump_index(i))
            cg.add(var.set_pump_installed_switch(i, sw))

        if CONF_RUN_SWITCH in pump_conf:
            sw = await esphome_switch.new_switch(pump_conf[CONF_RUN_SWITCH])
            await cg.register_parented(sw, config[CONF_ID])
            cg.add(sw.set_pump_index(i))
            cg.add(var.set_pump_run_switch(i, sw))

        if CONF_TOTAL_VOLUME_SENSOR in pump_conf:
            stat = await sensor.new_sensor(pump_conf[CONF_TOTAL_VOLUME_SENSOR])
            cg.add(var.set_pump_total_volume_sensor(i, stat))

        if CONF_TOTAL_RUNTIME_SENSOR in pump_conf:
            stat = await sensor.new_sensor(pump_conf[CONF_TOTAL_RUNTIME_SENSOR])
            cg.add(var.set_pump_total_runtime_sensor(i, stat))

        # Per-pump Number entities.
        # initial_value_override converts the compile-time time-period value
        # (milliseconds) to the number's native unit so first-boot behaviour
        # matches what the user declared in the static config.
        if CONF_MAX_RUNTIME_NUMBER in pump_conf:
            initial_min = pump_conf[CONF_MAX_RUNTIME].total_milliseconds / 60000.0
            num = await new_sprinklify_number(
                pump_conf[CONF_MAX_RUNTIME_NUMBER],
                config[CONF_ID],
                initial_value_override=initial_min,
            )
            cg.add(var.set_pump_max_runtime_number(i, num))

        if CONF_AUTO_RESET_WAIT_TIME_NUMBER in pump_conf:
            if CONF_AUTO_RESET_WAIT_TIME in pump_conf:
                initial_h = (
                    pump_conf[CONF_AUTO_RESET_WAIT_TIME].total_milliseconds
                    / 3_600_000.0
                )
            else:
                initial_h = 24  # matches number schema initial default
            num = await new_sprinklify_number(
                pump_conf[CONF_AUTO_RESET_WAIT_TIME_NUMBER],
                config[CONF_ID],
                initial_value_override=initial_h,
            )
            cg.add(var.set_pump_auto_reset_wait_time_number(i, num))
