"""Doppler pass detection on a live microphone.

Finds tonal sources moving past the microphone - aircraft, helicopters,
vehicles with a whine - by the S-shaped glide of their pitch, fits the moving-
source model to it and reports each pass's speed, closest distance and
frequency. Independent of the spectral analyzer: it takes its own copy of the
audio from the audio_source and runs in its own task.
"""

import esphome.codegen as cg
from esphome.components import audio_source, sensor, text_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    DEVICE_CLASS_SPEED,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_DECIBEL,
    UNIT_HERTZ,
    UNIT_METER,
    UNIT_PERCENT,
)

CODEOWNERS = ["@eman"]
AUTO_LOAD = ["sensor", "text_sensor"]
DEPENDENCIES = ["audio_source"]
ESP_PLATFORMS = ["esp32"]

CONF_SOURCE = "source"
CONF_F_LOW = "f_low"
CONF_F_HIGH = "f_high"
CONF_MIN_PROMINENCE = "min_prominence"
CONF_MAX_RESIDUAL = "max_residual"
CONF_MIN_DURATION = "min_duration"
CONF_MIN_RISE = "min_rise"
CONF_MIN_TIME_SCALE = "min_time_scale"
CONF_TEMPERATURE_UNIT = "temperature_unit"
CONF_SPEED = "speed"
CONF_DISTANCE = "distance"
CONF_FREQUENCY = "frequency"
CONF_SWING = "swing"
CONF_LEVEL = "level"
CONF_PASSES = "passes"
CONF_LOAD = "load"
CONF_LAST_PASS = "last_pass"

UNIT_METER_PER_SECOND = "m/s"

doppler_ns = cg.esphome_ns.namespace("doppler")
Doppler = doppler_ns.class_("Doppler", cg.PollingComponent)


def _validate_range(config):
    if config[CONF_F_LOW] >= config[CONF_F_HIGH]:
        raise cv.Invalid("f_low must be below f_high")
    return config


# Published once per pass. Set force_update on these in the YAML if two
# passes in a row could read the same value and each should be a history row.
def _pass_sensor(**kw):
    return sensor.sensor_schema(state_class=STATE_CLASS_MEASUREMENT, **kw)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Doppler),
            cv.Required(CONF_SOURCE): cv.use_id(audio_source.AudioSource),
            # Tonal lines are looked for between these. The decimated stream
            # is 12 kHz, so the top is near its Nyquist.
            cv.Optional(CONF_F_LOW, default=40.0): cv.float_range(min=20.0, max=4000.0),
            cv.Optional(CONF_F_HIGH, default=5000.0): cv.float_range(min=100.0, max=5500.0),
            # How far a line must stand above the median of its +-88 Hz
            # neighbourhood to start or extend a ridge.
            cv.Optional(CONF_MIN_PROMINENCE, default=8.0): cv.float_range(min=3.0, max=30.0),
            # RMS distance of the chain from the fitted curve. Real passes
            # have fitted to 0.2-0.5 Hz (1.46 Hz bins); noise to 0.6-2 Hz.
            cv.Optional(CONF_MAX_RESIDUAL, default=1.0): cv.float_range(min=0.2, max=10.0),
            cv.Optional(CONF_MIN_DURATION, default="2s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=500), max=cv.TimePeriod(seconds=60)),
            ),
            # A passing source is loudest at closest approach; an engine
            # changing RPM is not. dB above the level at the plateaus.
            cv.Optional(CONF_MIN_RISE, default=2.0): cv.float_range(min=0.0, max=30.0),
            # Closest distance over speed. A glide quicker than this is a
            # pitch step (birds, engines shifting) as often as a pass, so it
            # is not reported. 5 s is a car 50 m off; 0s reports close
            # vehicles too, with those false alarms.
            cv.Optional(CONF_MIN_TIME_SCALE, default="5s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(max=cv.TimePeriod(seconds=60)),
            ),
            # Air temperature sets the speed of sound, which scales every
            # speed and distance: 1% per 6 C. Without it, 15 C is assumed.
            cv.Optional(CONF_TEMPERATURE): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_TEMPERATURE_UNIT, default="C"): cv.one_of("C", "F", upper=True),
            # Per pass, of its best-fitting line.
            cv.Optional(CONF_SPEED): _pass_sensor(
                unit_of_measurement=UNIT_METER_PER_SECOND,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_SPEED,
                icon="mdi:airplane",
            ),
            cv.Optional(CONF_DISTANCE): _pass_sensor(
                unit_of_measurement=UNIT_METER,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_DISTANCE,
            ),
            cv.Optional(CONF_FREQUENCY): _pass_sensor(
                unit_of_measurement=UNIT_HERTZ,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_FREQUENCY,
            ),
            cv.Optional(CONF_SWING): _pass_sensor(
                unit_of_measurement=UNIT_HERTZ,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_FREQUENCY,
            ),
            cv.Optional(CONF_LEVEL): _pass_sensor(
                unit_of_measurement=UNIT_DECIBEL,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
            ),
            cv.Optional(CONF_PASSES): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
                icon="mdi:counter",
            ),
            # Share of real time the detector task is busy, per update.
            cv.Optional(CONF_LOAD): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:cpu-32-bit",
            ),
            # JSON with the fitted curve's parameters, enough to redraw it.
            cv.Optional(CONF_LAST_PASS): text_sensor.text_sensor_schema(icon="mdi:sine-wave"),
        }
    )
    .extend(cv.polling_component_schema("60s")),
    _validate_range,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_source(await cg.get_variable(config[CONF_SOURCE])))
    cg.add(var.set_frequency_range(config[CONF_F_LOW], config[CONF_F_HIGH]))
    cg.add(var.set_min_prominence(config[CONF_MIN_PROMINENCE]))
    cg.add(var.set_max_residual(config[CONF_MAX_RESIDUAL]))
    cg.add(var.set_min_duration(config[CONF_MIN_DURATION].total_milliseconds / 1000.0))
    cg.add(var.set_min_rise(config[CONF_MIN_RISE]))
    cg.add(var.set_min_time_scale(config[CONF_MIN_TIME_SCALE].total_milliseconds / 1000.0))
    if CONF_TEMPERATURE in config:
        temp = await cg.get_variable(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(temp, config[CONF_TEMPERATURE_UNIT] == "F"))
    for key, setter in (
        (CONF_SPEED, "set_speed_sensor"),
        (CONF_DISTANCE, "set_distance_sensor"),
        (CONF_FREQUENCY, "set_frequency_sensor"),
        (CONF_SWING, "set_swing_sensor"),
        (CONF_LEVEL, "set_level_sensor"),
        (CONF_PASSES, "set_count_sensor"),
        (CONF_LOAD, "set_load_sensor"),
    ):
        if key in config:
            cg.add(getattr(var, setter)(await sensor.new_sensor(config[key])))
    if CONF_LAST_PASS in config:
        cg.add(var.set_last_pass_text_sensor(await text_sensor.new_text_sensor(config[CONF_LAST_PASS])))
