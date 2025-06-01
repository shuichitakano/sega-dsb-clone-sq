//

#include <hw_config.h>

extern "C"
{
    size_t sd_get_num() { return 1; }

    sd_card_t *sd_get_by_num(size_t num)
    {
        static spi_t spi = {
            .hw_inst = spi0,
            .miso_gpio = 16,
            .mosi_gpio = 19,
            .sck_gpio = 18,
            .baud_rate = 125 * 1000 * 1000 / 4,
        };

        static sd_spi_if_t spi_if = {
            .spi = &spi,
            .ss_gpio = 17,
        };

        static sd_card_t sd_card = {
            .type = SD_IF_SPI,
            .spi_if_p = &spi_if,
        };

        if (num == 0)
        {
            return &sd_card;
        }
        else
        {
            return {};
        }
    }
}
