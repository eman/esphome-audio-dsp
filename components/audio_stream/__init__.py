"""Live audio as an endless WAV over HTTP.

Anything can consume it without a plugin: ffmpeg, sox, VLC, Audacity, go2rtc.
The capture path never waits on a socket - samples go into a ring buffer and a
client that cannot keep up loses audio rather than stalling the analysis.
"""

import esphome.codegen as cg
from esphome.components import audio_source
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT, CONF_SAMPLE_RATE

CODEOWNERS = ["@eman"]
ESP_PLATFORMS = ["esp32"]
DEPENDENCIES = ["network", "audio_source"]

CONF_SOURCE = "source"
CONF_BITS_PER_SAMPLE = "bits_per_sample"
CONF_MAX_CLIENTS = "max_clients"
CONF_BUFFER_DURATION = "buffer_duration"

audio_stream_ns = cg.esphome_ns.namespace("audio_stream")
AudioStream = audio_stream_ns.class_("AudioStream", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AudioStream),
        cv.Required(CONF_SOURCE): cv.use_id(audio_source.AudioSource),
        cv.Optional(CONF_PORT, default=8080): cv.port,
        # 24 keeps the microphone's full word. 16 halves the bandwidth and is
        # plenty for listening, but throws away resolution that matters when
        # the signal of interest is 70 dB below full scale.
        cv.Optional(CONF_BITS_PER_SAMPLE, default=24): cv.one_of(16, 24, int=True),
        cv.Optional(CONF_MAX_CLIENTS, default=4): cv.int_range(min=1, max=8),
        # Stream at a lower rate than the source, for listening over a link
        # that cannot carry the full rate. Must divide the source rate exactly.
        # The analyzer is unaffected - it reads the source directly.
        #
        # 48 kHz 24-bit is 1.15 Mbit/s; 16 kHz 16-bit is 0.26 Mbit/s.
        cv.Optional(CONF_SAMPLE_RATE): cv.int_range(min=4000, max=96000),
        # Slack before a slow client starts losing audio.
        cv.Optional(CONF_BUFFER_DURATION, default="1s"): cv.All(
            cv.positive_time_period_milliseconds,
            cv.Range(
                min=cv.TimePeriod(milliseconds=100),
                max=cv.TimePeriod(seconds=30),
            ),
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_source(await cg.get_variable(config[CONF_SOURCE])))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_bits(config[CONF_BITS_PER_SAMPLE]))
    cg.add(var.set_max_clients(config[CONF_MAX_CLIENTS]))
    if (out_rate := config.get(CONF_SAMPLE_RATE)) is not None:
        cg.add(var.set_stream_rate(out_rate))
    cg.add(var.set_buffer_ms(config[CONF_BUFFER_DURATION].total_milliseconds))
