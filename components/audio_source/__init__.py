"""Shared I2S capture for the audio-dsp components.

Owns the peripheral and fans blocks of samples out to whatever consumes them,
so the analyser and the streaming server can run from one microphone.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components.esp32 import include_builtin_idf_component
from esphome.const import CONF_ID, CONF_SAMPLE_RATE

CODEOWNERS = ["@eman"]

ESP_PLATFORMS = ["esp32"]

CONF_BCLK_PIN = "bclk_pin"
CONF_WS_PIN = "ws_pin"
CONF_DIN_PIN = "din_pin"
CONF_USE_RIGHT_SLOT = "use_right_slot"
CONF_USE_APLL = "use_apll"
CONF_DMA_BUFFERS = "dma_buffers"
CONF_DMA_FRAME_SIZE = "dma_frame_size"
CONF_BLOCK_SIZE = "block_size"

audio_source_ns = cg.esphome_ns.namespace("audio_source")
AudioSource = audio_source_ns.class_("AudioSource", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AudioSource),
        cv.Required(CONF_BCLK_PIN): cv.int_range(min=0, max=63),
        cv.Required(CONF_WS_PIN): cv.int_range(min=0, max=63),
        cv.Required(CONF_DIN_PIN): cv.int_range(min=0, max=63),
        cv.Optional(CONF_SAMPLE_RATE, default=48000): cv.int_range(min=8000, max=96000),
        # Most breakouts strap L/R low, putting the mic in the left slot.
        cv.Optional(CONF_USE_RIGHT_SLOT, default=False): cv.boolean,
        # Defaults to true, unlike ESPHome's i2s_audio. A delta-sigma mic puts
        # bit-clock jitter into its noise floor, and the general-purpose PLL is
        # disturbed by whatever else the chip is doing. See audio_source.cpp.
        cv.Optional(CONF_USE_APLL, default=True): cv.boolean,
        # DMA depth is what covers a consumer that blocks: an FFT of 32768
        # points takes ~114 ms, and the default here is 341 ms at 48 kHz. Too
        # small and audio is lost silently, which is exactly what ESPHome's
        # i2s_audio does with its fixed 4 x 256 (21 ms).
        cv.Optional(CONF_DMA_BUFFERS, default=16): cv.int_range(min=2, max=64),
        cv.Optional(CONF_DMA_FRAME_SIZE, default=1023): cv.int_range(min=64, max=4092),
        cv.Optional(CONF_BLOCK_SIZE, default=1024): cv.int_range(min=64, max=8192),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_pins(config[CONF_BCLK_PIN], config[CONF_WS_PIN], config[CONF_DIN_PIN]))
    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_use_right_slot(config[CONF_USE_RIGHT_SLOT]))
    cg.add(var.set_use_apll(config[CONF_USE_APLL]))
    cg.add(var.set_dma_buffers(config[CONF_DMA_BUFFERS]))
    cg.add(var.set_dma_frame_size(config[CONF_DMA_FRAME_SIZE]))
    cg.add(var.set_block_size(config[CONF_BLOCK_SIZE]))

    # ESPHome excludes esp_driver_i2s from its IDF build, so driver/i2s_std.h
    # is not found without this.
    include_builtin_idf_component("esp_driver_i2s")
