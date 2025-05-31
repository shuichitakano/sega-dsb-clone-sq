#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/clocks.h"
#include <cmath>
#include "i2s.pio.h"

namespace
{
    auto *pioI2S = pio0;
    static constexpr int SM_I2S = 0;

    static constexpr int pinLRCK = 13;
    static constexpr int pinBCK = 15;
    static constexpr int pinDATA = 14;
    static constexpr int pinXSMT = 12;

    static constexpr int sampleFreq = 44100;
}

void sendI2S(uint32_t *data, size_t size)
{
    auto dst = (volatile uint32_t *)&pioI2S->txf[SM_I2S];
    auto tail = data + size;
    while (data != tail)
    {
        while (pio_sm_is_tx_fifo_full(pioI2S, SM_I2S))
            ;
        *dst = *data++;
    }
}

void initI2S()
{
    uint offset = pio_add_program(pioI2S, &send_i2s_48fs_program);
    initProgramSendI2S48fs(pioI2S, SM_I2S, offset, pinLRCK, pinBCK, pinDATA);

    auto clock = clock_get_hz(clk_sys);
    // auto divider = clock * 256 / (sampleFreq * 2 * 24 * 2;
    auto divider = clock * 8 / (sampleFreq * 3);
    // 125000000*256/(44100*2*24*2) = 7558.578987150416
    assert(divider < 0x1000000);
    auto divFrac = divider & 0xffu;
    auto divInt = divider >> 8u;
    printf("I2S clock divider: %d %d.%02d\n", divider, divInt, divFrac);
    pio_sm_set_clkdiv_int_frac(pioI2S, SM_I2S, divInt, divFrac);
}

int main()
{
    stdio_init_all();

    puts("Hello, world!");

    constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    gpio_init(pinXSMT);
    gpio_set_dir(pinXSMT, GPIO_OUT);
    gpio_put(pinXSMT, 1); // mute 解除

    // 44100Hz の 440Hz は周期 100サンプル
    uint32_t wave[100];
    for (int i = 0; i < 100; ++i)
    {
        float t = i * (2 * 3.14159265f / 100.0f);
        float st = sin(t);
        float sc = cos(t);
        float scale = 32767.0f / 6;
        uint16_t l = static_cast<int>(st * scale);
        uint16_t r = static_cast<int>(sc * scale);

        wave[i] = (l << 16) | r;
    }

    initI2S();

    while (1)
    {
        sendI2S(wave, 100);
    }

    return 0;
}
