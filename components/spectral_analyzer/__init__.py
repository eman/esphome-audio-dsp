"""Frequency-domain analysis of a live microphone.

Takes blocks of samples from an audio_source, runs a windowed FFT over them,
and publishes per-band level, SNR, peak frequency, peak prominence and peak
stability - plus a periodic full-spectrum scan that reports whatever tonal
peaks it finds anywhere in the spectrum.
"""
from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import audio_source, sensor, text_sensor
import esphome.final_validate as fv
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_SAMPLE_RATE,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL,
    UNIT_HERTZ,
)

AUTO_LOAD = ["sensor", "text_sensor"]
DEPENDENCIES = ["audio_source"]
ESP_PLATFORMS = ["esp32"]

CONF_SOURCE = "source"
CONF_FFT_SIZE = "fft_size"
CONF_BROADBAND_RMS = "broadband_rms"
CONF_NOISE_FLOOR = "noise_floor"
CONF_BANDS = "bands"
CONF_F_LOW = "f_low"
CONF_F_HIGH = "f_high"
CONF_LEVEL = "level"
CONF_SNR = "snr"
CONF_PEAK_FREQUENCY = "peak_frequency"
CONF_PEAK_PROMINENCE = "peak_prominence"
CONF_PEAK_STABILITY = "peak_stability"
CONF_SPECTRUM_SCAN = "spectrum_scan"
CONF_TOP_N = "top_n"
CONF_MIN_PROMINENCE = "min_prominence"
CONF_PERSISTENCE = "persistence"
CONF_PEAKS = "peaks"
CONF_BAND = "band"
CONF_HARMONICS = "harmonics"
CONF_TOLERANCE = "tolerance"
CONF_ORDERS = "orders"
CONF_ORDER = "order"
CONF_RELATIVE_LEVEL = "relative_level"
CONF_PROMINENCE = "prominence"
CONF_FREQUENCY = "frequency"
CONF_ZOOM = "zoom"
CONF_PEAK_AGREEMENT = "peak_agreement"
CONF_AGREEMENT_TOLERANCE = "agreement_tolerance"
CONF_PEAK_LEVEL_SPREAD = "peak_level_spread"
CONF_LEVEL_OFFSET = "level_offset"
CONF_SECOND_PEAK_FREQUENCY = "second_peak_frequency"
CONF_SECOND_PEAK_PROMINENCE = "second_peak_prominence"
CONF_MODULATION_FREQUENCY = "modulation_frequency"
CONF_MODULATION_PROMINENCE = "modulation_prominence"
CONF_MODULATION_DEPTH = "modulation_depth"
CONF_MODULATION_F_LOW = "modulation_f_low"
CONF_MODULATION_F_HIGH = "modulation_f_high"

# A zoom frame longer than this is a minute of source behavior averaged into
# one number; nothing this component looks at holds still that long.
ZOOM_MAX_FRAME_S = 60.0

spectral_analyzer_ns = cg.esphome_ns.namespace("spectral_analyzer")
SpectralAnalyzer = spectral_analyzer_ns.class_("SpectralAnalyzer", cg.PollingComponent)


def _db_sensor():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_DECIBEL,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
    )


def _hz_sensor():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_HERTZ,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _validate_scan(conf):
    if conf[CONF_F_HIGH] and conf[CONF_F_HIGH] <= conf[CONF_F_LOW]:
        raise cv.Invalid(f"{CONF_F_HIGH} must be greater than {CONF_F_LOW}")
    return conf


# The discovery half of the node: sweeps the whole spectrum on a slow cadence
# and reports whatever stands proud of its local noise floor, whether or not a
# band was configured for it.
SCAN_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_UPDATE_INTERVAL, default="5min"): cv.positive_time_period_seconds,
            cv.Optional(CONF_TOP_N, default=8): cv.int_range(min=1, max=32),
            # 6 dB over the local median is a tone you can hear once you know
            # to listen; below ~4 dB the list fills with noise ripple.
            cv.Optional(CONF_MIN_PROMINENCE, default=6.0): cv.float_range(min=1.0, max=60.0),
            # Scans a peak must appear in before it is marked steady. A source
            # that runs for hours reads differently from a one-off transient.
            cv.Optional(CONF_PERSISTENCE, default=2): cv.int_range(min=1, max=255),
            cv.Optional(CONF_F_LOW, default=20.0): cv.positive_float,
            # 0 means "up to Nyquist".
            cv.Optional(CONF_F_HIGH, default=0.0): cv.float_range(min=0.0),
            cv.Optional(CONF_PEAKS): text_sensor.text_sensor_schema(),
        }
    ),
    _validate_scan,
)


def _hz_fine_sensor():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_HERTZ,
        accuracy_decimals=3,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _db_relative_sensor():
    # Not signal_strength: a level relative to the fundamental is a ratio.
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_DECIBEL,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _percent_sensor():
    return sensor.sensor_schema(
        unit_of_measurement="%",
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _validate_order(value):
    value = cv.float_range(min=0.02, max=32.0)(value)
    if abs(value - 1.0) < 0.02:
        raise cv.Invalid("order 1 is the fundamental itself; use 2 and up, or below 1")
    return value


def _validate_orders(orders):
    seen = [o[CONF_ORDER] for o in orders]
    if len(seen) != len(set(seen)):
        raise cv.Invalid("each harmonic order may appear only once")
    return orders


# Harmonics of the band's peak, measured at order x f0. Tells a machine (a
# series) from a whistle or a resonance (mostly a single line).
HARMONICS_SCHEMA = cv.Schema(
    {
        # Search half-width per order: H3 is looked for within +/-3x this.
        # Grows with order because a drifting fundamental smears its nth
        # harmonic n times wider.
        cv.Optional(CONF_TOLERANCE, default=2.0): cv.float_range(min=0.1, max=50.0),
        # The fundamental must clear this prominence or nothing is measured:
        # harmonics of a noise peak are noise.
        cv.Optional(CONF_MIN_PROMINENCE, default=6.0): cv.float_range(min=0.0, max=60.0),
        cv.Required(CONF_ORDERS): cv.All(
            cv.ensure_list(
                cv.Schema(
                    {
                        # Integers look up the series; fractions look under
                        # it. 0.2 on a 700 Hz line is the shaft of a 5-blade
                        # fan. Tolerance scales with the order either way.
                        cv.Required(CONF_ORDER): _validate_order,
                        cv.Optional(CONF_RELATIVE_LEVEL): _db_relative_sensor(),
                        cv.Optional(CONF_PROMINENCE): _db_sensor(),
                        cv.Optional(CONF_FREQUENCY): _hz_sensor(),
                    }
                )
            ),
            cv.Length(min=1),
            _validate_orders,
        ),
    }
)

def _validate_zoom(conf):
    if conf[CONF_MODULATION_F_HIGH] <= conf[CONF_MODULATION_F_LOW]:
        raise cv.Invalid(f"{CONF_MODULATION_F_HIGH} must be greater than {CONF_MODULATION_F_LOW}")
    return conf


# A high-resolution spectrum of this band alone; see dsp.h ZoomFFT.
ZOOM_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_FFT_SIZE, default=2048): cv.one_of(
                256, 512, 1024, 2048, 4096, 8192, 16384, int=True
            ),
            cv.Optional(CONF_PEAK_FREQUENCY): _hz_fine_sensor(),
            cv.Optional(CONF_PEAK_PROMINENCE): _db_sensor(),
            # The second strongest separate line in the band. Two similar
            # machines put two lines here; one machine puts noise here.
            cv.Optional(CONF_SECOND_PEAK_FREQUENCY): _hz_fine_sensor(),
            cv.Optional(CONF_SECOND_PEAK_PROMINENCE): _db_sensor(),
            # Periodic modulation of the band's envelope: the throb. Two tones
            # df apart beat at df, so modulation_frequency equal to the two
            # lines' separation is the beating test, run every frame.
            cv.Optional(CONF_MODULATION_FREQUENCY): _hz_sensor(),
            cv.Optional(CONF_MODULATION_PROMINENCE): _db_sensor(),
            cv.Optional(CONF_MODULATION_DEPTH): _percent_sensor(),
            cv.Optional(CONF_MODULATION_F_LOW, default=0.5): cv.float_range(min=0.05, max=100.0),
            cv.Optional(CONF_MODULATION_F_HIGH, default=5.0): cv.float_range(min=0.1, max=200.0),
        }
    ),
    _validate_zoom,
)


def zoom_decimation(sample_rate, bandwidth):
    """Must match dsp::ZoomFFT::decimation_for."""
    return max(1, int(sample_rate * 0.7 / max(bandwidth, 0.1)))


def _validate_band(conf):
    if conf[CONF_F_HIGH] <= conf[CONF_F_LOW]:
        raise cv.Invalid(f"{CONF_F_HIGH} must be greater than {CONF_F_LOW}")
    return conf


BAND_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Required(CONF_NAME): cv.string,
            cv.Required(CONF_F_LOW): cv.positive_float,
            cv.Required(CONF_F_HIGH): cv.positive_float,
            cv.Optional(CONF_LEVEL): _db_sensor(),
            cv.Optional(CONF_SNR): _db_sensor(),
            cv.Optional(CONF_PEAK_FREQUENCY): _hz_sensor(),
            # The tonal detection metric: strongest bin vs the band's median.
            cv.Optional(CONF_PEAK_PROMINENCE): _db_sensor(),
            # Spread of the per-frame peak across the interval: small means a
            # tone is holding its frequency, regardless of how loud it is.
            # Robust (scaled median absolute deviation), so one frame that
            # lost the tone to noise does not read as a wandering peak.
            cv.Optional(CONF_PEAK_STABILITY): _hz_sensor(),
            # Percent of frames whose peak sits within agreement_tolerance of
            # the interval's median peak. The blunt companion to stability.
            cv.Optional(CONF_PEAK_AGREEMENT): _percent_sensor(),
            cv.Optional(CONF_AGREEMENT_TOLERANCE, default=3.0): cv.float_range(min=0.1, max=100.0),
            # Robust spread of the per-frame peak level, in dB: a steady
            # source reads ~1 dB, one that throbs or fades reads more.
            cv.Optional(CONF_PEAK_LEVEL_SPREAD): _db_relative_sensor(),
            cv.Optional(CONF_HARMONICS): HARMONICS_SCHEMA,
            cv.Optional(CONF_ZOOM): ZOOM_SCHEMA,
        }
    ),
    _validate_band,
)


def _validate_with_rate(config, sr):
    nyquist = sr / 2
    for band in config[CONF_BANDS]:
        if band[CONF_F_HIGH] >= nyquist:
            raise cv.Invalid(
                f"band '{band[CONF_NAME]}' f_high ({band[CONF_F_HIGH]} Hz) must be below "
                f"the Nyquist frequency ({nyquist} Hz)"
            )
        zoom = band.get(CONF_ZOOM)
        if zoom is not None:
            d = zoom_decimation(sr, band[CONF_F_HIGH] - band[CONF_F_LOW])
            frame_s = zoom[CONF_FFT_SIZE] * d / sr
            if frame_s > ZOOM_MAX_FRAME_S:
                raise cv.Invalid(
                    f"band '{band[CONF_NAME]}' zoom: {zoom[CONF_FFT_SIZE]} points over a "
                    f"{band[CONF_F_HIGH] - band[CONF_F_LOW]:g} Hz band makes {frame_s:.0f} s "
                    f"frames ({sr / d / zoom[CONF_FFT_SIZE]:.3f} Hz bins); use a smaller "
                    f"{CONF_FFT_SIZE} or a wider band"
                )
            # The envelope spectrum shares the frame, so its bins are the
            # same width; a search floor under two bins is looking at the DC
            # skirt, and a ceiling above the decimated Nyquist is empty.
            ebin = sr / d / zoom[CONF_FFT_SIZE]
            if zoom[CONF_MODULATION_F_LOW] < 2 * ebin:
                raise cv.Invalid(
                    f"band '{band[CONF_NAME]}' zoom: {CONF_MODULATION_F_LOW} "
                    f"{zoom[CONF_MODULATION_F_LOW]:g} Hz is under two envelope bins "
                    f"({2 * ebin:.2f} Hz) for this frame; raise it or use a larger {CONF_FFT_SIZE}"
                )
            if zoom[CONF_MODULATION_F_HIGH] >= sr / d / 2:
                raise cv.Invalid(
                    f"band '{band[CONF_NAME]}' zoom: {CONF_MODULATION_F_HIGH} must be below "
                    f"{sr / d / 2:.1f} Hz for this band's decimation"
                )
    scan = config.get(CONF_SPECTRUM_SCAN)
    if scan is not None and scan[CONF_F_HIGH] >= nyquist:
        raise cv.Invalid(
            f"{CONF_SPECTRUM_SCAN} {CONF_F_HIGH} ({scan[CONF_F_HIGH]} Hz) must be below "
            f"the Nyquist frequency ({nyquist} Hz); leave it at 0 to scan up to Nyquist"
        )
    # Each publish needs at least one complete FFT frame; ask for two so an
    # interval never lands empty just because of where a frame boundary fell.
    frame_ms = 1000.0 * config[CONF_FFT_SIZE] / sr
    interval_ms = config[CONF_UPDATE_INTERVAL].total_milliseconds
    if interval_ms < 2 * frame_ms:
        raise cv.Invalid(
            f"{CONF_UPDATE_INTERVAL} ({interval_ms:.0f} ms) is too short for "
            f"{CONF_FFT_SIZE} {config[CONF_FFT_SIZE]} at {sr} Hz: each frame takes "
            f"{frame_ms:.0f} ms, so use at least {2 * frame_ms:.0f} ms"
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SpectralAnalyzer),
            cv.Required(CONF_SOURCE): cv.use_id(audio_source.AudioSource),
            cv.Optional(CONF_FFT_SIZE, default=32768): cv.one_of(
                1024, 2048, 4096, 8192, 16384, 32768, 65536, int=True
            ),
            cv.Optional(CONF_BROADBAND_RMS): _db_sensor(),
            cv.Optional(CONF_NOISE_FLOOR): _db_sensor(),
            # Added to broadband, floor and band levels. 0 publishes dBFS.
            # An ICS-43434 reads -26 dBFS at 94 dB SPL, so 120 publishes
            # dB SPL. Ratios (SNR, prominence, relative) are unaffected.
            cv.Optional(CONF_LEVEL_OFFSET, default=0.0): cv.float_range(min=-200.0, max=200.0),
            cv.Optional(CONF_BANDS, default=[]): cv.ensure_list(BAND_SCHEMA),
            cv.Optional(CONF_SPECTRUM_SCAN): SCAN_SCHEMA,
        }
    ).extend(cv.polling_component_schema("5s")),
)


def _final_validate(config):
    """Check band and scan ranges against the source's actual sample rate.

    The rate lives on the audio source, not here, so these checks have to wait
    until the whole config is known.
    """
    full = fv.full_config.get()
    src = full.get("audio_source") or {}
    sr = src.get(CONF_SAMPLE_RATE, 48000)
    return _validate_with_rate(config, sr)


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_source(await cg.get_variable(config[CONF_SOURCE])))
    cg.add(var.set_fft_size(config[CONF_FFT_SIZE]))
    if config[CONF_LEVEL_OFFSET] != 0.0:
        cg.add(var.set_level_offset(config[CONF_LEVEL_OFFSET]))

    if CONF_BROADBAND_RMS in config:
        cg.add(var.set_rms_sensor(await sensor.new_sensor(config[CONF_BROADBAND_RMS])))
    if CONF_NOISE_FLOOR in config:
        cg.add(var.set_floor_sensor(await sensor.new_sensor(config[CONF_NOISE_FLOOR])))

    if CONF_SPECTRUM_SCAN in config:
        scan = config[CONF_SPECTRUM_SCAN]
        cg.add(var.set_scan_interval(int(scan[CONF_UPDATE_INTERVAL].total_seconds)))
        cg.add(var.set_scan_top_n(scan[CONF_TOP_N]))
        cg.add(var.set_scan_min_prominence(scan[CONF_MIN_PROMINENCE]))
        cg.add(var.set_scan_persistence(scan[CONF_PERSISTENCE]))
        cg.add(var.set_scan_range(scan[CONF_F_LOW], scan[CONF_F_HIGH]))
        if CONF_PEAKS in scan:
            cg.add(var.set_scan_text_sensor(await text_sensor.new_text_sensor(scan[CONF_PEAKS])))

    for band in config[CONF_BANDS]:
        level = await sensor.new_sensor(band[CONF_LEVEL]) if CONF_LEVEL in band else cg.nullptr
        snr = await sensor.new_sensor(band[CONF_SNR]) if CONF_SNR in band else cg.nullptr
        peak = (
            await sensor.new_sensor(band[CONF_PEAK_FREQUENCY])
            if CONF_PEAK_FREQUENCY in band
            else cg.nullptr
        )
        prominence = (
            await sensor.new_sensor(band[CONF_PEAK_PROMINENCE])
            if CONF_PEAK_PROMINENCE in band
            else cg.nullptr
        )
        stability = (
            await sensor.new_sensor(band[CONF_PEAK_STABILITY])
            if CONF_PEAK_STABILITY in band
            else cg.nullptr
        )
        agreement = (
            await sensor.new_sensor(band[CONF_PEAK_AGREEMENT])
            if CONF_PEAK_AGREEMENT in band
            else cg.nullptr
        )
        level_spread = (
            await sensor.new_sensor(band[CONF_PEAK_LEVEL_SPREAD])
            if CONF_PEAK_LEVEL_SPREAD in band
            else cg.nullptr
        )
        cg.add(
            var.add_band(
                band[CONF_NAME],
                band[CONF_F_LOW],
                band[CONF_F_HIGH],
                level,
                snr,
                peak,
                prominence,
                stability,
                agreement,
                level_spread,
            )
        )
        if band[CONF_AGREEMENT_TOLERANCE] != 3.0:
            cg.add(var.set_agreement_tolerance(band[CONF_AGREEMENT_TOLERANCE]))
        if CONF_HARMONICS in band:
            harm = band[CONF_HARMONICS]
            cg.add(var.set_harmonic_params(harm[CONF_TOLERANCE], harm[CONF_MIN_PROMINENCE]))
            for o in harm[CONF_ORDERS]:
                rel = (
                    await sensor.new_sensor(o[CONF_RELATIVE_LEVEL])
                    if CONF_RELATIVE_LEVEL in o
                    else cg.nullptr
                )
                prom = (
                    await sensor.new_sensor(o[CONF_PROMINENCE]) if CONF_PROMINENCE in o else cg.nullptr
                )
                freq = (
                    await sensor.new_sensor(o[CONF_FREQUENCY]) if CONF_FREQUENCY in o else cg.nullptr
                )
                cg.add(var.add_harmonic(o[CONF_ORDER], rel, prom, freq))
        if CONF_ZOOM in band:
            zoom = band[CONF_ZOOM]
            zpeak = (
                await sensor.new_sensor(zoom[CONF_PEAK_FREQUENCY])
                if CONF_PEAK_FREQUENCY in zoom
                else cg.nullptr
            )
            zprom = (
                await sensor.new_sensor(zoom[CONF_PEAK_PROMINENCE])
                if CONF_PEAK_PROMINENCE in zoom
                else cg.nullptr
            )

            async def opt(key):
                return await sensor.new_sensor(zoom[key]) if key in zoom else cg.nullptr

            cg.add(
                var.set_zoom(
                    zoom[CONF_FFT_SIZE],
                    zpeak,
                    zprom,
                    await opt(CONF_SECOND_PEAK_FREQUENCY),
                    await opt(CONF_SECOND_PEAK_PROMINENCE),
                    await opt(CONF_MODULATION_FREQUENCY),
                    await opt(CONF_MODULATION_PROMINENCE),
                    await opt(CONF_MODULATION_DEPTH),
                    zoom[CONF_MODULATION_F_LOW],
                    zoom[CONF_MODULATION_F_HIGH],
                )
            )


# ------------------------------------------------------------ actions
# Retuning a band at runtime, and putting it back.
#
# ESPHome builds its entity list at compile time, so a band cannot be *added*
# while running - there would be no sensors to publish it on. Declaring a few
# spare bands in YAML and aiming them at whatever the discovery scan turns up
# gives the same result without a reflash. Changes are saved to flash.
#
# Exposed to Home Assistant by wiring them to api actions in YAML; see
# example/full.yaml.

SetBandAction = spectral_analyzer_ns.class_("SetBandAction", automation.Action)
ResetBandAction = spectral_analyzer_ns.class_("ResetBandAction", automation.Action)
ResetAllBandsAction = spectral_analyzer_ns.class_("ResetAllBandsAction", automation.Action)

SET_BAND_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(SpectralAnalyzer),
        cv.Required(CONF_BAND): cv.templatable(cv.string),
        cv.Required(CONF_F_LOW): cv.templatable(cv.positive_float),
        cv.Required(CONF_F_HIGH): cv.templatable(cv.positive_float),
    }
)

RESET_BAND_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(SpectralAnalyzer),
        cv.Required(CONF_BAND): cv.templatable(cv.string),
    }
)

RESET_ALL_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(SpectralAnalyzer)})


@automation.register_action(
    "spectral_analyzer.set_band", SetBandAction, SET_BAND_SCHEMA, synchronous=True
)
async def set_band_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_band_name(await cg.templatable(config[CONF_BAND], args, cg.std_string)))
    cg.add(var.set_f_low(await cg.templatable(config[CONF_F_LOW], args, float)))
    cg.add(var.set_f_high(await cg.templatable(config[CONF_F_HIGH], args, float)))
    return var


@automation.register_action(
    "spectral_analyzer.reset_band", ResetBandAction, RESET_BAND_SCHEMA, synchronous=True
)
async def reset_band_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_band_name(await cg.templatable(config[CONF_BAND], args, cg.std_string)))
    return var


@automation.register_action(
    "spectral_analyzer.reset_all_bands", ResetAllBandsAction, RESET_ALL_SCHEMA, synchronous=True
)
async def reset_all_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var
