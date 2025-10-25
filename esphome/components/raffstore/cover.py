from esphome import automation
import esphome.codegen as cg  # type: ignore
from esphome.components import cover  # type: ignore
import esphome.config_validation as cv  # type: ignore
from esphome.const import (  # type: ignore
    CONF_CLOSE_ACTION,
    CONF_CLOSE_DURATION,
    CONF_ID,
    CONF_OPEN_ACTION,
    CONF_OPEN_DURATION,
    CONF_STOP_ACTION,
)

CONF_FULL_TILT_DURATION = "full_tilt_duration"
CONF_INTERLOCK_WAIT_TIME = "interlock_wait_time"

raffstore_ns = cg.esphome_ns.namespace("raffstore")
Raffstore = raffstore_ns.class_("Raffstore", cover.Cover, cg.Component)

CONFIG_SCHEMA = (
    cover.cover_schema(Raffstore)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(Raffstore),
            cv.Required(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
            cv.Required(CONF_FULL_TILT_DURATION): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_INTERLOCK_WAIT_TIME, default="0s"
            ): cv.positive_time_period_milliseconds,
            cv.Required(CONF_STOP_ACTION): automation.validate_automation(single=True),
            cv.Required(CONF_OPEN_ACTION): automation.validate_automation(single=True),
            cv.Required(CONF_CLOSE_ACTION): automation.validate_automation(single=True),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    await automation.build_automation(
        var.get_stop_trigger(), [], config[CONF_STOP_ACTION]
    )

    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    await automation.build_automation(
        var.get_open_trigger(), [], config[CONF_OPEN_ACTION]
    )

    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    await automation.build_automation(
        var.get_close_trigger(), [], config[CONF_CLOSE_ACTION]
    )

    cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    cg.add(var.set_full_tilt_duration(config[CONF_FULL_TILT_DURATION]))
    cg.add(var.set_interlock_wait_time(config[CONF_INTERLOCK_WAIT_TIME]))
    await cover.register_cover(var, config)
