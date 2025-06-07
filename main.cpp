
#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <hardware/dma.h>
#include <hardware/pio.h>
#include <hardware/interp.h>
#include <hardware/clocks.h>
#include <hardware/vreg.h>
#include <hardware/uart.h>
#include <cmath>
#include "i2s.pio.h"
#include "mutex.h"
#include "midi.h"

#include "hw_config.h"
#include "f_util.h"
#include "ff.h"

extern "C"
{
#include "music_file.h"
}

#include <string>
#include <mutex>
#include <map>
#include <array>
#include <cmath>
#include <algorithm>

namespace
{
    auto *pioI2S = pio0;
    constexpr int SM_I2S = 0;

    constexpr int PIN_LRCK = 13;
    constexpr int PIN_BCK = 15;
    constexpr int PIN_DATA = 14;
    constexpr int PIN_XSMT = 12;

    auto *midiUART = uart1;
    constexpr int MIDI_BAUDRATE = 31250;
    constexpr int MIDI_RX_PIN = 5;

    constexpr int sampleFreq = 44100;
    constexpr int DMA_CH_I2S = 2; // SDで0,1つかう

    constexpr size_t WAVE_BUFFER_SIZE = 3000;
    constexpr size_t WAVE_RING_SIZE = 4;

    struct WaveBufferUnit
    {
        uint32_t data[WAVE_BUFFER_SIZE];
        size_t size = 0;
    };

    WaveBufferUnit waveBuffer_[WAVE_RING_SIZE];
    size_t waveOutRingID_ = 0;
    size_t waveInRingID_ = 0;

    bool dmaTransferInProgress_ = false;

    bool isWaveInFull()
    {
        return (waveInRingID_ + 1) % WAVE_RING_SIZE == waveOutRingID_;
    }

    bool isWaveOutEmpty()
    {
        return waveInRingID_ == waveOutRingID_;
    }

    [[maybe_unused]] void nextWaveIn()
    {
        waveInRingID_ = (waveInRingID_ + 1) % WAVE_RING_SIZE;
    }

    void nextWaveOut()
    {
        waveOutRingID_ = (waveOutRingID_ + 1) % WAVE_RING_SIZE;
    }
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

    bool startTransferI2S()
    {
        if (isWaveOutEmpty())
        {
            return false;
        }

        auto &wb = waveBuffer_[waveOutRingID_];
        dma_channel_set_trans_count(DMA_CH_I2S, wb.size, false);
        dma_channel_set_read_addr(DMA_CH_I2S, wb.data, true);
        nextWaveOut();
        dmaTransferInProgress_ = true;
        return true;
    }

    void __isr __not_in_flash_func(i2sDMAIRQHandler)()
    {
        if (dma_hw->ints1 & (1 << DMA_CH_I2S))
        {
            dma_hw->ints1 = 1 << DMA_CH_I2S;

            static bool led = false;
            setLED(led);
            led ^= true;

            dmaTransferInProgress_ = startTransferI2S();
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

    void initMIDI()
    {
        uart_init(midiUART, MIDI_BAUDRATE);
        gpio_set_function(MIDI_RX_PIN, GPIO_FUNC_UART);
        uart_set_format(midiUART, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(midiUART, true);
    }

    [[maybe_unused]] void fsTest()
    {
        FATFS fs;
        FRESULT fr = f_mount(&fs, "", 1);
        if (FR_OK != fr)
        {
            panic("f_mount error: %d\n", fr);
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

    [[maybe_unused]] void fstest2()
    {
        DIR dir;
        auto fr = f_opendir(&dir, "/");

        if (FR_OK != fr)
        {
            printf("f_opendir error: %d\n", fr);
            return;
        }
        FILINFO fno;
        while (true)
        {
            fr = f_readdir(&dir, &fno);
            if (FR_OK != fr)
            {
                printf("f_readdir error: %d\n", fr);
                return;
            }
            if (fno.fname[0] == 0) // end of directory
            {
                break;
            }

            printf("File: %s, Size: %llu\n", fno.fname, fno.fsize);
        }
        fr = f_closedir(&dir);
        if (FR_OK != fr)
        {
            printf("f_closedir error: %d\n", fr);
            return;
        }
        printf("Directory listing completed\n");
    }

    // main

    class TrackHolder
    {
        std::map<int, std::string> trackMap_;

    public:
        void init(const char *path)
        {
            printf("Parse contents of %s\n", path);
            DIR dir;
            auto fr = f_opendir(&dir, path);
            if (FR_OK != fr)
            {
                printf("f_opendir error: %d\n", fr);
                return;
            }

            while (true)
            {
                FILINFO fno;
                fr = f_readdir(&dir, &fno);
                if (FR_OK != fr)
                {
                    printf("f_readdir error: %d\n", fr);
                    return;
                }
                if (fno.fname[0] == 0) // end of directory
                {
                    break;
                }

                const char *fname = fno.fname;
                printf("File: %s, Size: %llu\n", fname, fno.fsize);

                auto l = strlen(fname);
                if (l >= 4 && strcmp(fname + l - 4, ".mp3") == 0)
                {
                    int trackNum = 0;
                    if (sscanf(fname, "%d_", &trackNum) == 1)
                    {
                        trackMap_[trackNum] = std::string(path) + "/" + fname;
                        printf("  Track %d: %s\n", trackNum, fname);
                    }
                }
            }
            fr = f_closedir(&dir);
            if (FR_OK != fr)
            {
                printf("f_closedir error: %d\n", fr);
                return;
            }
            printf("done.\n");
        }

        const char *getTrack(int trackNum) const
        {
            auto it = trackMap_.find(trackNum);
            if (it != trackMap_.end())
            {
                return it->second.c_str();
            }
            return nullptr;
        }
    };

    struct Request
    {
        std::string loadFile;
        int loadTrack = -1;
        bool repeat = false;
        bool stop = false;
    };

    Mutex mutex_;
    Request request_;
    TrackHolder trackHolder_;

    constexpr int MAX_VOLUME = 32768 / 6;

    // 256 * 0.3 = 76.8 db
    constexpr int VOLTBL_STEPS = 256;
    constexpr float VOLTBL_STEP_DB = 0.3f;                               // 1 step = 0.1 dB
    constexpr float VOLTBL_MAX_DB = (VOLTBL_STEPS - 1) * VOLTBL_STEP_DB; // ≒25.5 dB

    constexpr float db_to_gain(float attenuation_db)
    {
        return std::pow(10.0f, -attenuation_db / 20.0f);
    }

    constexpr std::array<uint16_t, VOLTBL_STEPS> volumeTable_ = []()
    {
        std::array<uint16_t, VOLTBL_STEPS> tbl = {};
        for (int i = 0; i < VOLTBL_STEPS; ++i)
        {
            float attenuation = (VOLTBL_STEPS - 1 - i) * VOLTBL_STEP_DB;
            float gain = db_to_gain(attenuation);
            tbl[i] = i == 0 ? 0 : static_cast<uint16_t>(gain * MAX_VOLUME + 0.5f);
        }
        return tbl;
    }();

    int volumeIndex_ = VOLTBL_STEPS - 1; // MAX
    int fadeSpeed8_ = 0;                 // 1s あたりの volume変化*8
    int fade8_ = 0;

    void setVolume(float db)
    {
        volumeIndex_ = std::clamp(static_cast<int>(VOLTBL_STEPS - 1 + db / VOLTBL_STEP_DB), 0, VOLTBL_STEPS - 1);
    }

    void setFadeSpeed(float fadeDBPerSec)
    {
        fadeSpeed8_ = static_cast<int>(fadeDBPerSec * 8 / VOLTBL_STEP_DB);
    }

    void resetFade()
    {
        fadeSpeed8_ = 0;
        fade8_ = 0;
    }

    void updateFade(size_t sampleCount, size_t sampleRate)
    {
        if (fadeSpeed8_ > 0 && fade8_ < volumeIndex_ * 8)
        {
            fade8_ += fadeSpeed8_ * sampleCount / sampleRate;
        }
    }

    int getLinearVolume()
    {
        int i = std::clamp(volumeIndex_ - (fade8_ >> 3), 0, VOLTBL_STEPS - 1);
        return volumeTable_[i];
    }

    constexpr size_t MP3_WORKING_SIZE = 16000;
    constexpr size_t MP3_DECODE_BUFFER_SIZE = WAVE_BUFFER_SIZE * 2;
    int16_t mp3DecodeBuffer_[MP3_DECODE_BUFFER_SIZE];
    uint8_t mp3Working_[MP3_WORKING_SIZE];

    void setRequest(Request &&req)
    {
        std::lock_guard<Mutex> lock(mutex_);
        request_ = std::move(req);
    }

    Request popRequest()
    {
        std::lock_guard<Mutex> lock(mutex_);
        Request req = std::move(request_);
        request_ = {};
        return req;
    }

    struct MP3Player
    {
        music_file mf_;
        uint32_t sampleRate_ = 0;
        int nChannels_ = 0;
        bool opened_ = false;

        void open(const std::string &filename)
        {
            close();
            resetFade();

            if (!musicFileCreate(&mf_, filename.c_str(), mp3Working_, MP3_WORKING_SIZE))
            {
                printf("mp3FileCreate error: %s\n", filename.c_str());
                return;
            }

            sampleRate_ = musicFileGetSampleRate(&mf_);
            nChannels_ = musicFileGetChannels(&mf_);
            printf("MP3 file: %s, sample rate: %lu, channels: %d\n",
                   filename.c_str(), sampleRate_, nChannels_);

            opened_ = true;
        }

        void close()
        {
            if (opened_)
            {
                musicFileClose(&mf_);
                opened_ = false;
                printf("MP3 file closed\n");
            }
        }

        void tick()
        {
            if (!opened_ || isWaveInFull())
            {
                return;
            }

            uint32_t written = 0;
            if (musicFileRead(&mf_, mp3DecodeBuffer_, MP3_DECODE_BUFFER_SIZE, &written))
            {
                if (written > 0)
                {
                    // printf("MP3 file read %lu samples\n", written);
                    auto &wb = waveBuffer_[waveInRingID_];
                    auto *dst = wb.data;
                    const auto *src = mp3DecodeBuffer_;
                    auto *srcTail = src + written;

                    int scale = getLinearVolume();

                    if (nChannels_ == 2)
                    {
                        // Stereo
                        do
                        {
                            int l = src[0] * scale >> 15;
                            int r = src[1] * scale >> 15;
                            *dst = (l << 16) | (r & 0xffff);
                            ++dst;
                            src += 2;
                        } while (src < srcTail);
                        wb.size = written >> 1;
                    }
                    else
                    {
                        // Mono
                        do
                        {
                            int s = *src++ * scale >> 15;
                            *dst = (s << 16) | (s & 0xffff);
                        } while (src < srcTail);
                        wb.size = written;
                    }

                    updateFade(wb.size, sampleRate_);

                    nextWaveIn();

                    if (!dmaTransferInProgress_)
                    {
                        startTransferI2S();
                    }
                }
                else
                {
                    printf("MP3 file read finished\n");
                    close();
                }
            }
            else
            {
                printf("Error reading MP3 file\n");
                close();
            }
        }
    };

    void __not_in_flash_func(core1_main)()
    {
        MP3Player player;

        FATFS fs;
        FRESULT fr = f_mount(&fs, "", 1);
        if (FR_OK != fr)
        {
            panic("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        }

        // fstest2();
        trackHolder_.init("/dsb");

        while (1)
        {
            auto req = popRequest();
            if (!req.loadFile.empty())
            {
                printf("Request to load file: %s\n", req.loadFile.c_str());
                player.open(req.loadFile);
            }
            else if (req.loadTrack > 0)
            {
                printf("Request to load track: %d\n", req.loadTrack);
                auto *trackFile = trackHolder_.getTrack(req.loadTrack);
                if (trackFile)
                {
                    player.open(trackFile);
                }
                else
                {
                    printf("Track %d not found\n", req.loadTrack);
                }
            }
            else if (req.stop)
            {
                printf("Request to stop playback\n");
                player.close();
            }

            player.tick();
        }
    }

}

void processMIDI(io::MidiMessage &m)
{
    m.dump();

    int statusKind = m.data[0] >> 4;
    int ch = (m.data[0] & 0x0f) + 1;
    switch (statusKind)
    {
    case 0x9: // Note On
        if (m.size >= 3)

        {
            int note = m.data[1];
            if (note > 0x3c)
            {
                int num = note - 0x3c; // C4
                printf("play %d\n", num);
                Request r;
                r.loadTrack = num;
                setRequest(std::move(r));
            }
            else if (note == 0x3c)
            {
                printf("stop\n");
                Request r;
                r.stop = true;
                setRequest(std::move(r));
            }
            else if (note == 0x3b)
            {
                printf("fadeout\n");
                int v = 100;
                int speed = 4000 / v;

                setFadeSpeed(speed);
            }
        }
        break;

    case 0xa: // Aftertouch
    {
        int note = m.data[1];
        int pressure = m.data[2];
        // Play command
        if (m.size >= 3 && ch >= 15)
        {
            if (note == 16 || note == 111)
            {
                if (pressure > 0)
                {
                    Request r;
                    r.loadTrack = pressure;

                    // Activate repeat if channel 16
                    constexpr bool ENABLE_SPIKEOUT = false;
                    if (ch == 16 || ENABLE_SPIKEOUT)
                    {
                        r.repeat = true;
                    }
                    printf("CMD: BGM #%d repeat %d\n", r.loadTrack, r.repeat);
                    setRequest(std::move(r));
                }
                else
                {
                    printf("CMD: BGM #0 STOP\n");
                    Request r;
                    r.stop = true;
                    setRequest(std::move(r));
                }
            }
        }

        // Volume command
        if (note == 1 || note == 2)
        {
            // volume は最大90
            printf("CMD: Volume change. %d\n", pressure);
            setVolume((pressure - 90) / 3.0f); // 全体で-30dbにしてみる
        }

        // Fadeout command
        if (note == 3 || note == 4)
        {
            if (pressure > 0)
            {
                // pressure ms で -4db
                float speed = 4 * 1000 / pressure;
                printf("CMD: Fadeout. %d, %f\n", pressure, speed);
                setFadeSpeed(speed);
            }
        }
    }
    break;
    }
}

void __not_in_flash_func(core0_main)()
{
    if (0)
    {
        Request r;
        r.loadFile = "test.mp3";
        setRequest(std::move(r));
    }

#if 1
    io::MidiMessageMaker midiMaker;

    while (1)
    {
        auto data = uart_getc(midiUART);
        midiMaker.analyze(data, [&](const io::MidiMessage &m)
                          { processMIDI(const_cast<io::MidiMessage &>(m)); });
    }
#else
    std::string str;
    while (1)
    {
        auto ch = getchar_timeout_us(0);
        if (ch == PICO_ERROR_TIMEOUT)
        {
            // No input, continue
            sleep_ms(10);
            continue;
        }
        if (ch == '\r' || ch == '\n')
        {
            printf("enter!!!! %s\n", str.c_str());
            int num = atoi(str.c_str());
            if (num > 0)
            {
                Request r;
                r.loadTrack = num;
                setRequest(std::move(r));
            }
            str.clear();
        }
        else
        {
            str.push_back(ch);
        }
    }
#endif
}

int main()
{
#if 1
    // vreg_set_voltage(VREG_VOLTAGE_1_20);
    sleep_ms(10);
    bool clock_set = set_sys_clock_khz(140000, true);
    if (!clock_set)
    {
        panic("Failed to set system clock");
    }
#endif

    stdio_init_all();

    printf("DSB Clone^2\n");

    initLED();

    // fsTest();

    gpio_init(PIN_XSMT);
    gpio_set_dir(PIN_XSMT, GPIO_OUT);
    gpio_put(PIN_XSMT, 1); // mute 解除

    printf("init I2S...\n");
    initI2S();

    enableIRQI2SDMA(true);

    initMIDI();

    multicore_launch_core1(core1_main);
    core0_main();

    return 0;
}
