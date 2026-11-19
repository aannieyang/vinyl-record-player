# Pico 2 W record player: PCM1808 -> RIAA -> PCM5102A

## Important: status
This is a **hardware-dependent first integration build**, not tested on your breadboard.
A PIO program generates and samples synchronized I2S using a single state machine. It
uses CPU polling, not DMA; it is suitable for initial debugging, with risk of
FIFO overrun/underrun under heavy workloads. Verify the pin clocks and audio
before relying on it for continuous playback.

## Before applying power
* **PCM1808** must be configured as **SLAVE** (MD0=0, MD1=0), **I2S** (FMT=0).
  The board may expose straps/jumpers. If the ADC is master, you MUST change its
  mode before connecting the Pico-generated LRCK and BCK (avoid output-to-output wiring).
* **PCM5102A** must be a true compatible breakout, set for I2S and enabled/unmuted.
  SCK grounding is module-dependent. Confirm the DAC breakout supply voltage.
* NE5532 output previously sat at **5 V DC**. Do NOT wire this directly to
  the PCM1808 input. Use the ADC module's specified analog AC-coupling and
  bias/reference network; verify output voltage and signal range first.
* Share signal-ground reference; keep the separate 10 V, 5 V, and 3.3 V positive
  rails distinct. Keep LM386 at low volume/with attenuation.
* Only the **LEFT** ADC channel is processed; output is dual-mono so your DAC's
  left output and one LM386/speaker can be used.

## Wiring (Pico physical pins, not GPIO numbers)
| Pico pin | GPIO | To |
|---|---|---|
| 17 | GP13 | PCM1808 SCKI/MCLK (12.288 MHz output) |
| 25 | GP19 | PCM1808 LRCK input + PCM5102A LCK input (48 kHz) |
| 26 | GP20 | PCM1808 BCK input + PCM5102A BCK input (3.072 MHz) |
| 27 | GP21 | PCM1808 DOUT output -> Pico input |
| 29 | GP22 | Pico -> PCM5102A DIN input |
| 38 | GND | Common ground reference |
| 36 | 3V3 OUT | DAC VCC **only if board explicitly supports 3.3 V input** |

PCM5102A L analog output -> attenuation/volume -> LM386 VIN; external LM386 5 V power
(if module supports 5 V), speaker to amplifier speaker terminals.

## Clocking
System 120 MHz; MCLK and PIO instruction rate use the same fractional divider
9.765625 (120 MHz / 9.765625 = 12.288 MHz). The PIO executes 256 instructions
per stereo frame, yielding 48 kHz LRCK and 3.072 MHz BCK (64 BCK/frame).
MCLK/PIO each have their own fractional divider. Verify all three frequencies on
an oscilloscope; fractional clock division may introduce some jitter.

## Build in VS Code / Pico SDK
Set **board to `pico2_w`** using the Raspberry Pi Pico extension. The SDK must
be installed and `PICO_SDK_PATH` set. As standalone CMake build:

    cmake -S . -B build -DPICO_BOARD=pico2_w
    cmake --build build -j4

Flash `build/pico2w_record_audio.uf2` using BOOTSEL or the Pico extension.

## First test
1. Confirm the original 1kHz DAC-only test worked.
2. Power off; verify PCM1808 mode pins and analog coupling, then wire ADC.
3. Power the ADC and Pico; scope GP13 ~12.288 MHz, GP19 48 kHz,
   GP20 ~3.072 MHz. The bit clock differs from the old 16-bit DAC-only test
   (1.536 MHz) because PCM1808 requires 64 BCK/frame for this mode.
4. With modest low-amplitude ADC input, check DAC L output on the oscilloscope
   before enabling LM386 power.
5. Connect LM386 through attenuation, set volume low, and test sound.

## Tuning and pitfalls
* `RIAA_ENABLED=0` bypasses EQ, useful for comparing sound.
* `OUTPUT_GAIN=0.020f` is intentionally quiet. Increase gradually only after
  verifying clean waveform and avoiding DAC / amplifier clipping.
* `INPUT_LEFT_SECOND=1` selects the second 32-bit RX slot as left, based on
  the PIO LRCK polarity. If the wanted channel is silent, verify and select the
  opposite slot after measuring ADC signals.
* If audio is distorted or static, verify ADC FMT/MD0/MD1, clocks, PCM1808 DOUT,
  and DAC I2S mode. If the ADC returns all zeros, fix ADC/input before increasing gain.
* This proof-of-concept uses CPU polling without DMA. To make a reliable final
  design, use DMA double-buffering, add FIFO-underflow/overflow diagnostics,
  and ideally an engineered low-jitter audio clock.
* Do not use the old Pico ADC ISR or PWM audio output simultaneously.
