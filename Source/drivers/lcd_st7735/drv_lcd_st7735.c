/**
 * Copyright (c) 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrfx_spim.h"
#include "app_error.h"
#include "drv_lcd_st7735.h"
#include "sdk_config.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ST7735_SCK_PIN ARDUINO_13_PIN
#define ST7735_MISO_PIN ARDUINO_12_PIN
#define ST7735_MOSI_PIN ARDUINO_11_PIN
#define ST7735_SS_PIN ARDUINO_10_PIN
#define ST7735_DC_PIN ARDUINO_9_PIN

// ILI9341 LCD pin assignments
#define LCD_RESET   //NRF_GPIO_PIN_MAP(1,  7)
#define LCD_D_C     ST7735_DC_PIN//NRF_GPIO_PIN_MAP(1,  9)
#define LCD_CS      ST7735_SS_PIN//NRF_GPIO_PIN_MAP(0, 26)
#define LCD_SCK     ST7735_SCK_PIN//NRF_GPIO_PIN_MAP(0, 27)
#define LCD_MOSI    ST7735_MOSI_PIN//NRF_GPIO_PIN_MAP(1,  8)
#define LCD_MISO    ST7735_MISO_PIN//NRFX_SPIM_PIN_NOT_USED

#define SPI_INSTANCE  3                                           /**< SPI instance index. */
static const nrfx_spim_t spi_st7735_instance = NRFX_SPIM_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

#define SPI_BUFF_SIZE   16

static uint8_t spi_tx_buff[SPI_BUFF_SIZE];

static drv_st7735_evt_handler_t m_cb_st7735_handler;

/**
 * @brief Structure holding ST7735 controller basic parameters.
 */
typedef struct st7735_s
{
        uint8_t tab_color;  /**< Color of tab attached to the used screen. */
} st7735_t;


/**
 * @brief Enumerator with TFT tab colors.
 */
typedef enum {
        INITR_GREENTAB = 0, /**< Green tab. */
        INITR_REDTAB,       /**< Red tab. */
        INITR_BLACKTAB,     /**< Black tab. */
        INITR_144GREENTAB   /**< Green tab, 1.44" display. */
} st7735_tab_t;

static st7735_t m_st7735;


#define OP_NULL         0x00
#define OP_CMD          0x01
#define OP_DELAY        0x02

typedef struct {
        uint8_t op_type;
        size_t op_size;
        const uint8_t       *p_data;
} lcd_op_t;

static const uint8_t lcd_init_st7735_SWRESET[] = { ST7735_SWRESET };
static const uint8_t lcd_init_st7735_SLPOUT[]  = { ST7735_SLPOUT };
static const uint8_t lcd_init_st7735_FRMCTR1[]  = { ST7735_FRMCTR1, 0x01, 0x2C, 0x2D };
static const uint8_t lcd_init_st7735_FRMCTR2[]  = { ST7735_FRMCTR2, 0x01, 0x2C, 0x2D };
static const uint8_t lcd_init_st7735_FRMCTR3[]  = { ST7735_FRMCTR3, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D };
static const uint8_t lcd_init_st7735_INVCTR[]   = { ST7735_INVCTR, 0x07 };

static const uint8_t lcd_init_st7735_PWCTR1[]  = { ST7735_PWCTR1, 0xA2, 0x02, 0x84 };
static const uint8_t lcd_init_st7735_PWCTR2[]  = { ST7735_PWCTR2, 0xC5 };
static const uint8_t lcd_init_st7735_PWCTR3A[]  = { ST7735_PWCTR3, 0xA2, 0x00 };
static const uint8_t lcd_init_st7735_PWCTR3B[]  = { ST7735_PWCTR3, 0x8A, 0x2A };
static const uint8_t lcd_init_st7735_PWCTR5[]  = { ST7735_PWCTR5, 0x8A, 0xEE, 0x0E };

static const uint8_t lcd_init_st7735_INVOFF[]  = { ST7735_INVOFF };
static const uint8_t lcd_init_st7735_MADCT[]  = { ST7735_MADCTL, 0xC8 };

static const uint8_t lcd_init_st7735_COLMOD[]  = { ST7735_COLMOD, 0x05 };

static const uint8_t lcd_init_st7735_CASET_GREENTAB[]  = { ST7735_CASET, 0x00, 0x02, 0x00, 0x81};
static const uint8_t lcd_init_st7735_RASET_GREENTAB[]  = { ST7735_RASET, 0x00, 0x01, 0x00, 0xA0};

static const uint8_t lcd_init_st7735_CASET_144GREENTAB[]  = { ST7735_CASET, 0x00, 0x00, 0x00, 0x7F};
static const uint8_t lcd_init_st7735_RASET_144GREENTAB[]  = { ST7735_RASET, 0x00, 0x00, 0x00, 0x7F};

static const uint8_t lcd_init_st7735_CASET_REDTAB[]  = { ST7735_CASET, 0x00, 0x00, 0x00, 0x7F};
static const uint8_t lcd_init_st7735_RASET_REDTAB[]  = { ST7735_RASET, 0x00, 0x00, 0x00, 0x9F};

static const uint8_t lcd_init_st7735_GMCTRP1[]  = { ST7735_GMCTRP1, 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2b, 0x39, 0x00, 0x01, 0x03, 0x10};
static const uint8_t lcd_init_st7735_GMCTRN1[]  = { ST7735_GMCTRN1, 0x03, 0x1d, 0x07, 0x06, 0x2e, 0x2c, 0x29, 0x2d, 0x2e, 0x2e, 0x37, 0x3f, 0x00, 0x00, 0x02, 0x10};

static const uint8_t lcd_init_st7735_NORON[] = { ST7735_NORON };
static const uint8_t lcd_init_st7735_DISPON[] = { ST7735_DISPON };

static const uint8_t lcd_init_st7735_MADCTL[] = { ST7735_MADCTL, 0xC0 };

static const uint8_t lcd_init_st7735_INVON[] = { ST7789_INVON };

static const lcd_op_t lcd_init_st7735_seq[] =
{
        {OP_CMD, sizeof(lcd_init_st7735_SWRESET ), lcd_init_st7735_SWRESET },
        {OP_DELAY, 150,                 NULL       },
        {OP_CMD, sizeof(lcd_init_st7735_SLPOUT ),  lcd_init_st7735_SLPOUT },
        {OP_DELAY, 500,                 NULL       },
        {OP_CMD, sizeof(lcd_init_st7735_FRMCTR1 ), lcd_init_st7735_FRMCTR1 },
        {OP_CMD, sizeof(lcd_init_st7735_FRMCTR2 ), lcd_init_st7735_FRMCTR2 },
        {OP_CMD, sizeof(lcd_init_st7735_FRMCTR3 ), lcd_init_st7735_FRMCTR3 },
        {OP_CMD, sizeof(lcd_init_st7735_FRMCTR1 ), lcd_init_st7735_FRMCTR1 },
        {OP_CMD, sizeof(lcd_init_st7735_PWCTR1 ), lcd_init_st7735_PWCTR1 },
        {OP_CMD, sizeof(lcd_init_st7735_PWCTR2 ), lcd_init_st7735_PWCTR2 },
        {OP_CMD, sizeof(lcd_init_st7735_PWCTR3A ), lcd_init_st7735_PWCTR3A },
        {OP_CMD, sizeof(lcd_init_st7735_PWCTR3B ), lcd_init_st7735_PWCTR3B },
        {OP_CMD, sizeof(lcd_init_st7735_PWCTR5 ), lcd_init_st7735_PWCTR5 },

        {OP_CMD, sizeof(lcd_init_st7735_INVOFF ), lcd_init_st7735_INVOFF },
        {OP_CMD, sizeof(lcd_init_st7735_MADCT ), lcd_init_st7735_MADCT },

        #if (ST7735_TAB_COLOR == INITR_GREENTAB)

        {OP_CMD, sizeof(lcd_init_st7735_CASET_GREENTAB), lcd_init_st7735_CASET_GREENTAB},
        {OP_CMD, sizeof(lcd_init_st7735_RASET_GREENTAB), lcd_init_st7735_RASET_GREENTAB},

        #elif (ST7735_TAB_COLOR == INITR_144GREENTAB)

        {OP_CMD, sizeof(lcd_init_st7735_CASET_144GREENTAB), lcd_init_st7735_CASET_144GREENTAB},
        {OP_CMD, sizeof(lcd_init_st7735_RASET_144GREENTAB), lcd_init_st7735_RASET_144GREENTAB},

        #elif (ST7735_TAB_COLOR == INITR_REDTAB)

        {OP_CMD, sizeof(lcd_init_st7735_CASET_REDTAB), lcd_init_st7735_CASET_REDTAB},
        {OP_CMD, sizeof(lcd_init_st7735_RASET_REDTAB), lcd_init_st7735_RASET_REDTAB},

        #endif

        {OP_CMD, sizeof(lcd_init_st7735_GMCTRP1), lcd_init_st7735_GMCTRP1},
        {OP_CMD, sizeof(lcd_init_st7735_GMCTRN1), lcd_init_st7735_GMCTRN1},

        {OP_CMD, sizeof(lcd_init_st7735_NORON), lcd_init_st7735_NORON},
        {OP_DELAY, 10,                 NULL       },
        {OP_CMD, sizeof(lcd_init_st7735_DISPON), lcd_init_st7735_DISPON},
        {OP_DELAY, 100,                 NULL       },

        #if (ST7735_TAB_COLOR == INITR_BLACKTAB)
        {OP_CMD, sizeof(lcd_init_st7735_MADCTL), lcd_init_st7735_MADCTL},
        #endif

        {OP_CMD, sizeof(lcd_init_st7735_INVON), lcd_init_st7735_INVON},
        {OP_CMD, sizeof(lcd_init_st7735_DISPON), lcd_init_st7735_DISPON},


};

static inline void write_command(uint8_t c)
{
        nrfx_err_t err_code;
        uint8_t tx_buff[1];
        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buff, 1);
        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;
        tx_buff[0] = c;
        err_code = nrfx_spim_xfer_dcx(&spi_st7735_instance, &xfer_desc, flags, 1);
        APP_ERROR_CHECK(err_code);
}

static inline void write_data(uint8_t c)
{
        nrfx_err_t err_code;
        uint8_t tx_buff[1];
        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buff, 1);

        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;

        tx_buff[0] = c;
        err_code = nrfx_spim_xfer_dcx(&spi_st7735_instance, &xfer_desc, flags, 0);
        APP_ERROR_CHECK(err_code);
}




void drv_st7735_proc_op_list(const lcd_op_t *p_op_list)
{
        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(spi_tx_buff, p_op_list->op_size);//1);

        while (p_op_list->op_type != OP_NULL)
        {
                if (p_op_list->op_type == OP_CMD)
                {
                        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;
                        // send LCD command
                        memcpy(spi_tx_buff, p_op_list->p_data, p_op_list->op_size);
                        xfer_desc.tx_length = p_op_list->op_size;
                        nrfx_err_t err_code = nrfx_spim_xfer_dcx(&spi_st7735_instance, &xfer_desc, flags, 1);
                        APP_ERROR_CHECK(err_code);
                }
                else if (p_op_list->op_type == OP_DELAY)
                {
                        // wait awhile
                        nrf_delay_ms(p_op_list->op_size);
                }
                else
                {
                        // something goes wrong!
                        break;
                }
                p_op_list++;
        }
}

static void drv_spim_st7735_event_handler(nrfx_spim_evt_t * p_event)
{
        if (p_event->type == NRFX_SPIM_EVENT_DONE)
        {
                drv_st7735_evt_t ST7735_evt;
                ST7735_evt.evt_type = eLCD_st7735_EVT_TX_DONE;
                m_cb_st7735_handler(&ST7735_evt);
        }
}

/**@brief Function to initiate LCD.
 */
void drv_st7735_init(drv_st7735_init_t *p_drv_st7735)
{
        nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
        spi_config.frequency      = NRF_SPIM_FREQ_32M;
        spi_config.mode           = NRF_SPIM_MODE_3;
        spi_config.ss_pin         = LCD_CS;
        spi_config.miso_pin       = LCD_MISO;
        spi_config.mosi_pin       = LCD_MOSI;
        spi_config.sck_pin        = LCD_SCK;
        spi_config.dcx_pin        = LCD_D_C;
        spi_config.use_hw_ss      = true;
        spi_config.ss_active_high = false;
        spi_config.ss_duration    = 8;
        APP_ERROR_CHECK(nrfx_spim_init(&spi_st7735_instance, &spi_config, NULL, NULL));

        m_st7735.tab_color = ST7735_TAB_COLOR;

        // set SCK to high drive
        nrf_gpio_cfg(
                LCD_SCK,
                NRF_GPIO_PIN_DIR_OUTPUT,
                NRF_GPIO_PIN_INPUT_DISCONNECT,
                NRF_GPIO_PIN_NOPULL,
                NRF_GPIO_PIN_H0H1,
                NRF_GPIO_PIN_NOSENSE);

        // set MOSI to high drive
        nrf_gpio_cfg(
                LCD_MOSI,
                NRF_GPIO_PIN_DIR_OUTPUT,
                NRF_GPIO_PIN_INPUT_DISCONNECT,
                NRF_GPIO_PIN_NOPULL,
                NRF_GPIO_PIN_H0H1,
                NRF_GPIO_PIN_NOSENSE);

        m_cb_st7735_handler = p_drv_st7735->evt_handler;

        // send LCD init sequence
        drv_st7735_proc_op_list(lcd_init_st7735_seq);
}




static void drv_st7735_display_invert(bool invert)
{
        write_command(invert ? ST7735_INVON : ST7735_INVOFF);
}

static void drv_st7735_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
        uint8_t tx_buff[5];
        nrfx_err_t err_code;
        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buff, 5);
        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;
        tx_buff[0] = 0x2A;
        tx_buff[1] = ( x          >> 8) & 0xFF;
        tx_buff[2] = ( x              ) & 0xFF;
        tx_buff[3] = ((x + w - 1) >> 8) & 0xFF;
        tx_buff[4] = ((x + w - 1)     ) & 0xFF;
        err_code = nrfx_spim_xfer_dcx(&spi_st7735_instance, &xfer_desc, flags, 1);
        APP_ERROR_CHECK(err_code);

        tx_buff[0] = 0x2B;
        tx_buff[1] = ( y          >> 8) & 0xFF;
        tx_buff[2] = ( y              ) & 0xFF;
        tx_buff[3] = ((y + h - 1) >> 8) & 0xFF;
        tx_buff[4] = ((y + h - 1)     ) & 0xFF;
        err_code = nrfx_spim_xfer_dcx(&spi_st7735_instance, &xfer_desc, flags, 1);
        APP_ERROR_CHECK(err_code);
}


void drv_st7735_put_partial_update(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t const *p_lcd_data)
{
        drv_st7735_set_window(x, y, w, h);

        uint8_t tx_buff[1];
        nrfx_err_t err_code;

        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buff, 1);
        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;

        // send LCD command
        tx_buff[0] = ST7735_RAMWR;
        err_code = nrfx_spim_xfer_dcx(&spi_st7735_instance, &xfer_desc, flags, 1);
        APP_ERROR_CHECK(err_code);

        //nrf_delay_ms(1);
        uint32_t len = (uint32_t)w * (uint32_t)h * 2;
        uint32_t pos = 0, stp = 0;

        // send LCD data
        for (pos = 0; pos < len; pos += stp)
        {
                stp = MIN((len - pos), 65534);
                //NRF_LOG_INFO("gfx %d", stp);
                xfer_desc.tx_length = stp;
                xfer_desc.p_tx_buffer = p_lcd_data + pos;
                err_code = nrfx_spim_xfer_dcx(&spi_st7735_instance, &xfer_desc, 0, 0);
                APP_ERROR_CHECK(err_code);
        }

}

void drv_st7735_put_gfx(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t const *p_lcd_data)
{
        drv_st7735_set_window(x, y, w, h);

        uint8_t tx_buff[1];
        nrfx_err_t err_code;

        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buff, 1);
        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;

        // send LCD command
        tx_buff[0] = ST7735_RAMWR;
        err_code = nrfx_spim_xfer_dcx(&spi_st7735_instance, &xfer_desc, flags, 1);
        APP_ERROR_CHECK(err_code);

        //nrf_delay_ms(1);
        uint32_t len = (uint32_t)w * (uint32_t)h * 2;
        uint32_t pos = 0, stp = 0;

        // send LCD data
        for (pos = 0; pos < len; pos += stp)
        {
                stp = MIN((len - pos), 65534);
                //NRF_LOG_INFO("gfx %d", stp);
                xfer_desc.tx_length = stp;
                xfer_desc.p_tx_buffer = p_lcd_data + pos;
                err_code = nrfx_spim_xfer_dcx(&spi_st7735_instance, &xfer_desc, 0, 0);
                APP_ERROR_CHECK(err_code);
        }

}
