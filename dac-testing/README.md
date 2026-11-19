# Pico 2 W / PCM5102A / LM386 tone test

Produces a **1 kHz sine wave at very low volume**, sent as 16-bit stereo I2S at 48 kHz.
Both left and right DAC outputs receive the same tone; only the L output needs to go to your mono LM386 module.

## Wiring (Pico 2 W, physical header numbering)

| DAC input | Pico GPIO | Physical pin |
|---|---:|---:|
| LCK / LRCK | GP19 | 25 |
| BCK | GP20 | 26 |
| DIN | GP22 | 29 |
| GND | GND | 38 |
| VIN / VCC | 3V3(OUT), **only if your module supports 3.3 V at that pin** | 36 |

The DAC's analog `L` output goes through volume/attenuation to LM386 `VIN`. Ground connects to the common signal ground. The LM386 is powered by a suitable 5 V supply; **never power the LM386 from Pico 3V3**. The speaker goes to the module's designated speaker outputs.

**Module check:** This assumes the module contains a PCM5102A and is configured for standard I2S (not left-justified). Many modules support ground-tied SCK/MCLK and have hardware mode pads, but pin behavior differs by module. Verify chip markings, voltage requirements and mode pins prior to powering on.

## Build using Pico SDK / VS Code

1. Create a new Raspberry Pi Pico C/C++ project in VS Code for board **Pico 2 W**, or copy the files into an existing Pico SDK project.
2. Put `dac_tone_test.c` and `dac_tone_test.pio` beside your project's `CMakeLists.txt`.
3. If this is a *standalone* folder, copy `pico_sdk_import.cmake` from the Pico SDK's `external` directory into this folder (or use the VS Code Pico extension project generator's existing import setup). Set `PICO_BOARD=pico2_w` in your CMake project configuration.
4. Configure/build. Flash the generated `pcm5102a_tone_test.uf2` via BOOTSEL or the extension's Run option.
5. Start with LM386 volume at minimum. You should hear a quiet, continuous 1 kHz tone when you turn it up carefully.

## Debugging

- No sound: check VCC/ground, `FMT`/I2S setting, possible mute configuration, and amplifier gain.
- Expect approximately 48 kHz on LRCK (GP19), 1.536 MHz on BCK (GP20); 16-bit stereo = 32 bit clocks/frame.
- If DAC `L` has a sine but no sound: check LM386 VIN, amplifier power, speaker terminals.
- If the waveform is distorted or too loud: reduce the `TONE_AMPLITUDE` define and ensure analog attenuation.
- This program deliberately does not use the PCM1808, NE5532, or RIAA filter.
