// Board 2.0 audio output: PIO PWM on GPIO45 fed by DMA from a sample buffer in the PSRAM work area.
// Test tone only for now (see audio.h).
#include <math.h>
#include <string.h>
#include "audio.h"
#include "pins.h"
#include "psram.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "pwm.pio.h"

#define RATE_HZ 44100u
#define N_SAMPLES 2048u                        // ring: the DMA read address wraps at 8 KiB (2048 x uint32)
static uint32_t *const buf = (uint32_t *)PSRAM_WORK_AUD;
static PIO pio; static int sm = -1, dma = -1; static uint off; static uint32_t period; static bool ready, running;

bool audio_init(void) {
    if (ready) return true;
    pio = pio1;
    if (!pio_can_add_program(pio, &pwm_program)) return false;
    sm = pio_claim_unused_sm(pio, false); if (sm < 0) return false;
    off = pio_add_program(pio, &pwm_program);
    period = clock_get_hz(clk_sys) / RATE_HZ / 2;                    // two PIO cycles per count (jmp x!=y / jmp y--)
    pio_gpio_init(pio, PIN_AUDIO_PWM);
    pio_sm_set_consecutive_pindirs(pio, (uint)sm, PIN_AUDIO_PWM, 1, true);
    pio_sm_config c = pwm_program_get_default_config(off);
    sm_config_set_sideset_pins(&c, PIN_AUDIO_PWM);
    pio_sm_init(pio, (uint)sm, off, &c);
    pio_sm_put_blocking(pio, (uint)sm, period);
    pio_sm_exec(pio, (uint)sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, (uint)sm, pio_encode_out(pio_isr, 32));
    pio_sm_put_blocking(pio, (uint)sm, period / 2);                  // idle at mid level (the coupling capacitor sits at the DC point)
    pio_sm_set_enabled(pio, (uint)sm, true);
    dma = dma_claim_unused_channel(false); if (dma < 0) return false;
    ready = true;
    return true;
}
bool audio_ready(void) { return ready; }
uint32_t audio_rate_hz(void) { return ready ? clock_get_hz(clk_sys) / (period * 2) : 0; }

void audio_off(void) {
    if (!running) return;
    dma_channel_abort(dma); running = false;
    pio_sm_put_blocking(pio, (uint)sm, period / 2);
}
void audio_tone(uint32_t hz, uint32_t level) {
    if (!ready) return;
    audio_off();
    if (!hz) return;
    if (level > 100) level = 100;
    uint32_t rate = audio_rate_hz();
    uint32_t k = (hz * N_SAMPLES + rate / 2) / rate; if (!k) k = 1;   // whole periods per ring so the wrap is seamless
    double amp = (period / 2.0 - 1) * level / 100.0;
    for (uint32_t i = 0; i < N_SAMPLES; i++) buf[i] = (uint32_t)(period / 2.0 + amp * sin(2 * M_PI * k * i / N_SAMPLES));
    dma_channel_config c = dma_channel_get_default_config(dma);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_ring(&c, false, 13);                          // read address wraps at 8 KiB
    channel_config_set_dreq(&c, pio_get_dreq(pio, (uint)sm, true));
    dma_channel_configure(dma, &c, &pio->txf[sm], buf, 0x0FFFFFFF, true);
    running = true;
}
