#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/clocks.h"
#include <cmath>
#include "i2s.pio.h"

#include "hw_config.h"
#include "f_util.h"
#include "ff.h"

namespace
{
    auto *pioI2S = pio0;
    constexpr int SM_I2S = 0;

    constexpr int PIN_LRCK = 13;
    constexpr int PIN_BCK = 15;
    constexpr int PIN_DATA = 14;
    constexpr int PIN_XSMT = 12;

    constexpr int sampleFreq = 44100;

    constexpr int DMA_CH_I2S = 0;

    constexpr size_t WAVE_BUFFER_SIZE = 512;
    uint32_t waveBuffer_[2][WAVE_BUFFER_SIZE];

    int i2sDBID_ = 0;
}

namespace
{
    void initLED()
    {
        constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN;
        gpio_init(LED_PIN);
        gpio_set_dir(LED_PIN, GPIO_OUT);
        gpio_put(LED_PIN, 1);
    }

    void setLED(bool on)
    {
        constexpr uint LED_PIN = PICO_DEFAULT_LED_PIN;
        gpio_put(LED_PIN, on ? 1 : 0);
    }

    [[maybe_unused]] void sendI2S(uint32_t *data, size_t size)
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

    void enableIRQI2SDMA(bool f)
    {
        dma_hw->ints1 = 1 << DMA_CH_I2S;
        dma_channel_set_irq1_enabled(DMA_CH_I2S, f);
    }

    void updateWaveBuffer()
    {
        static float phase = {};
        constexpr float phaseStep = 2 * 3.14159265f * 440 / 44100;

        auto *dst = waveBuffer_[i2sDBID_];
        auto *tail = dst + WAVE_BUFFER_SIZE;
        do
        {
            float st = sinf(phase);
            float sc = cosf(phase);
            float scale = 32767.0f / 6;
            uint16_t l = static_cast<int>(st * scale);
            uint16_t r = static_cast<int>(sc * scale);

            *dst++ = (l << 16) | r;
            phase += phaseStep;
        } while (dst != tail);
    }

    void startTransferI2S()
    {
        dma_channel_set_trans_count(DMA_CH_I2S, WAVE_BUFFER_SIZE, false);
        dma_channel_set_read_addr(DMA_CH_I2S, waveBuffer_[i2sDBID_], true);
    }

    void flipI2SBuffer()
    {
        i2sDBID_ ^= 1;
    }

    void __isr __not_in_flash_func(i2sDMAIRQHandler)()
    {
        if (dma_hw->ints1 & (1 << DMA_CH_I2S))
        {
            dma_hw->ints1 = 1 << DMA_CH_I2S;

            static bool led = false;
            setLED(led);
            led ^= true;

            // 更新済みのバッファを転送
            startTransferI2S();

            // 次のバッファを更新
            flipI2SBuffer();
            updateWaveBuffer();
        }
    }

    void initI2S()
    {
        uint offset = pio_add_program(pioI2S, &send_i2s_48fs_program);
        initProgramSendI2S48fs(pioI2S, SM_I2S, offset, PIN_LRCK, PIN_BCK, PIN_DATA);

        auto clock = clock_get_hz(clk_sys);
        // auto divider = clock * 256 / (sampleFreq * 2 * 24 * 2;
        auto divider = clock * 8 / (sampleFreq * 3);
        // 125000000*256/(44100*2*24*2) = 7558.578987150416
        assert(divider < 0x1000000);
        auto divFrac = divider & 0xffu;
        auto divInt = divider >> 8u;
        printf("I2S clock divider: %d %d.%02d\n", (int)divider, (int)divInt, (int)divFrac);
        pio_sm_set_clkdiv_int_frac(pioI2S, SM_I2S, divInt, divFrac);

        // dma
        irq_set_exclusive_handler(DMA_IRQ_1, i2sDMAIRQHandler);
        irq_set_enabled(DMA_IRQ_1, true);

        dma_channel_config config = dma_channel_get_default_config(DMA_CH_I2S);
        channel_config_set_dreq(&config, pio_get_dreq(pioI2S, SM_I2S, true /* tx */));
        channel_config_set_read_increment(&config, true);
        channel_config_set_write_increment(&config, false);
        dma_channel_configure(DMA_CH_I2S, &config, &pioI2S->txf[SM_I2S], nullptr, 0, false);
    }

    [[maybe_unused]] void fsTest()
    {
        FATFS fs;
        FRESULT fr = f_mount(&fs, "", 1);
        if (FR_OK != fr)
        {
            panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        }

        // Open a file and write to it
        FIL fil;
        const char *const filename = "filename.txt";
        fr = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE);
        if (FR_OK != fr && FR_EXIST != fr)
        {
            panic("f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
        }
        if (f_printf(&fil, "Hello, world!\n") < 0)
        {
            printf("f_printf failed\n");
        }

        // Close the file
        fr = f_close(&fil);
        if (FR_OK != fr)
        {
            printf("f_close error: %s (%d)\n", FRESULT_str(fr), fr);
        }

        // Unmount the SD card
        f_unmount("");
    }
}

int main()
{
    stdio_init_all();

    printf("DSB Clone^2\n");

    initLED();

    // fsTest();

    gpio_init(PIN_XSMT);
    gpio_set_dir(PIN_XSMT, GPIO_OUT);
    gpio_put(PIN_XSMT, 1); // mute 解除

    printf("init I2S...\n");
    initI2S();

    printf("update wave buffer...\n");
    updateWaveBuffer();
    flipI2SBuffer();
    updateWaveBuffer();

    enableIRQI2SDMA(true);

    printf("start transfer...\n");
    flipI2SBuffer();
    startTransferI2S();
    flipI2SBuffer();

    while (1)
    {
    }

    return 0;
}
