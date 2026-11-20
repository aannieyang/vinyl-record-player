"""Host checks of actual source: externally clocked PIO timing and PCM conversion vectors.
Run: python3 adc-dac-passthrough/tests/verify_audio.py
Requires a host C compiler. Does not emulate FIFO stalls or electrical timing.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
program, labels = [], {}
for line in (root / 'i2s_duplex.pio').read_text().splitlines():
    line = line.split(';')[0].strip()
    if not line or line.startswith('.'):
        continue
    if line.endswith(':'):
        labels[line[:-1]] = len(program)
    else:
        program.append(line)

# Synthetic PCM1808 master: 120 MHz PIO, 3 MHz BCK, 32 bits/channel.
# Model 2 cycles of pin synchronization and 5 cycles of ADC output delay.
# Exercise startup across the entire stereo frame, including both LRCK levels.
def adc_word(word_index):
    pcm = (0x123450 + word_index * 0x234567) & 0xffffff
    return pcm << 7

def tx_word(word_index):
    # Initial two silent frames, then distinguishable positive/negative samples.
    return 0 if word_index < 4 else ((0xedcbb0 + word_index * 0x123456) & 0xffffff) << 7

for phase in range(0, 2560, 37):
    pc = x = rx = rx_bits = tx_bits = din = 0
    received, sent, dac_bits = [], [], []
    first_external_word = None
    for cycle in range(2560 * 6):
        t = cycle + phase
        observed_t = t - 2
        bck = int(observed_t % 40 >= 20)
        lrck = (observed_t // 1280) & 1
        # DAC observes physical BCK, without Pico's input synchronization delay.
        if t % 40 == 20 and first_external_word is not None:
            dac_bits.append(din)
        op = program[pc]
        next_pc = (pc + 1) % len(program)
        if op.startswith('wait '):
            _, value, kind, pin = op.split()
            assert kind == 'gpio' and int(pin) in (16, 17)
            actual = bck if int(pin) == 16 else lrck
            if actual != int(value):
                next_pc = pc
        elif op.startswith('set x,'):
            x = int(op.split(',')[1])
        elif op == 'out pins, 1':
            assert t % 40 < 20, 'DAC data changed after the sampling edge'
            external_word, slot = divmod(t // 40, 32)
            if first_external_word is None:
                first_external_word = external_word
                assert not external_word & 1, 'First word must be LEFT'
            assert external_word == first_external_word + tx_bits // 32
            assert slot == tx_bits % 32, 'Skipped or duplicated TX bit'
            word = tx_word(tx_bits // 32)
            din = (word >> (31 - slot)) & 1
            sent.append(din)
            tx_bits += 1
        elif op == 'in pins, 1':
            assert bck, 'ADC sampled before rising BCK'
            external_word, slot = divmod((observed_t - 5) // 40, 32)
            assert external_word == first_external_word + len(received)
            assert slot == rx_bits, 'Skipped or duplicated RX bit'
            bit = (adc_word(external_word) >> (31 - slot)) & 1
            rx = ((rx << 1) | bit) & 0xffffffff
            rx_bits += 1
            if rx_bits == 32:
                assert rx == adc_word(external_word)
                received.append(rx)
                rx = rx_bits = 0
        elif op.startswith('jmp x-- '):
            if x:
                next_pc = labels[op.split()[-1]]
            x = (x - 1) & 0xffffffff
        else:
            raise AssertionError(f'Unsupported instruction: {op}')
        pc = next_pc
    assert len(received) >= 8
    assert dac_bits == sent[:len(dac_bits)], 'DAC rising-edge data differs from intended TX'
print('PASS: external-clock PIO across startup phases; left/right alignment, RX words and DAC bits')

source = (root / 'adc-dac-passthrough.c').read_text()
functions = '\n'.join(re.findall(
    r'static inline (?:int32_t|uint32_t) (?:decode_sample|encode_sample|scale_sample)\([^}]+\}', source))
assert functions.count('static inline') == 3
volume_define = re.search(r'^#define VOLUME_DIVISOR \d+', source, re.M).group(0)
harness = r'''
#include <stdint.h>
#include <assert.h>
FUNCTIONS
int main(void) {
    const struct { uint32_t raw; int32_t decoded; uint32_t tx; } vectors[] = {
        {0x091a2800, 1193040, 0x01234500},
        {0x76e5d800, -1193040, 0x7edcbb00},
        {0, 0, 0}, {0x3fffff80, 8388607, 0x07ffff80},
        {0x40000000, -8388608, 0x78000000},
        {0x7fffff80, -1, 0}
    };
    for (unsigned i = 0; i < sizeof(vectors)/sizeof(vectors[0]); ++i) {
        assert(decode_sample(vectors[i].raw) == vectors[i].decoded);
        assert(decode_sample(vectors[i].raw | 0x8000007f) == vectors[i].decoded);
        int32_t scaled = scale_sample(vectors[i].decoded);
        assert(encode_sample(scaled) == vectors[i].tx);
    }
    for (int32_t s = -8388608; s < 8388608; s += 257) {
        uint32_t tx = encode_sample(s);
        assert(!(tx & 0x8000007f));
        assert(decode_sample(tx) == s);
        // DAC consumes slots 1..31 plus zero slot 0 of the next word.
        uint32_t dac_word = tx << 1;
        assert(dac_word == (uint32_t)s * 256u);
    }
}
'''.replace('FUNCTIONS', volume_define + '\n' + functions)
with tempfile.TemporaryDirectory() as directory:
    c = Path(directory) / 'pcm.c'
    exe = Path(directory) / 'pcm'
    c.write_text(harness)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=undefined', str(c), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('PASS: actual C decode/encode, signed boundaries, gain, ignored bits, DAC 32-bit framing')
