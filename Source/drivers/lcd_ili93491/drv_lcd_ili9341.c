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
#include "drv_lcd_ili9341.h"
#include "sdk_config.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ILI9341_SCK_PIN ARDUINO_13_PIN
#define ILI9341_MISO_PIN ARDUINO_12_PIN
#define ILI9341_MOSI_PIN ARDUINO_11_PIN
#define ILI9341_SS_PIN ARDUINO_10_PIN
#define ILI9341_DC_PIN ARDUINO_9_PIN

// ILI9341 LCD pin assignments
#define LCD_RESET   //NRF_GPIO_PIN_MAP(1,  7)
#define LCD_D_C     ILI9341_DC_PIN//NRF_GPIO_PIN_MAP(1,  9)
#define LCD_CS      ILI9341_SS_PIN//NRF_GPIO_PIN_MAP(0, 26)
#define LCD_SCK     ILI9341_SCK_PIN//NRF_GPIO_PIN_MAP(0, 27)
#define LCD_MOSI    ILI9341_MOSI_PIN//NRF_GPIO_PIN_MAP(1,  8)
#define LCD_MISO    ILI9341_MISO_PIN//NRFX_SPIM_PIN_NOT_USED

#define SPI_INSTANCE  3                                           /**< SPI instance index. */
static const nrfx_spim_t spi_ili9341_instance = NRFX_SPIM_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

#define SPI_BUFF_SIZE   16

static uint8_t spi_tx_buff[SPI_BUFF_SIZE];

static drv_ili9341_evt_handler_t m_cb_ili9341_handler;

#define OP_NULL         0x00
#define OP_CMD          0x01
#define OP_DELAY        0x02

typedef struct {
        uint8_t op_type;
        size_t op_size;
        const uint8_t       *p_data;
} lcd_op_t;

static const uint8_t lcd_init_ILI9341_SWRESET[] = { ILI9341_SWRESET };
static const uint8_t lcd_init_ILI9341_DISPOFF[] = { ILI9341_DISPOFF };
static const uint8_t lcd_init_ILI9341_PWCTRB[]  = { ILI9341_PWCTRB, 0x00, 0xC1, 0x30 };
static const uint8_t lcd_init_ILI9341_TIMCTRA[] = { ILI9341_TIMCTRA, 0x85, 0x00, 0x78 };
static const uint8_t lcd_init_ILI9341_PWCTRSEQ[] = { ILI9341_PWCTRSEQ, 0x39, 0x2C, 0x00, 0x34, 0x02 };
static const uint8_t lcd_init_ILI9341_PUMP[] = { ILI9341_PUMP, 0x20 };
static const uint8_t lcd_init_ILI9341_TIMCTRB[] = { ILI9341_TIMCTRB, 0x00, 0x00};
static const uint8_t lcd_init_ILI9341_PWCTR1[] = { ILI9341_PWCTR1, 0x23 };
static const uint8_t lcd_init_ILI9341_PWCTR2[] = { ILI9341_PWCTR2, 0x10 };
static const uint8_t lcd_init_ILI9341_VMCTR1[] = { ILI9341_VMCTR1, 0x3E, 0x28 };
static const uint8_t lcd_init_ILI9341_VMCTR2[] = { ILI9341_VMCTR2, 0x86 };
static const uint8_t lcd_init_ILI9341_MADCTL[] = { ILI9341_MADCTL, 0x48 };
static const uint8_t lcd_init_ILI9341_PIXFMT[] = { ILI9341_PIXFMT, 0x55 };
static const uint8_t lcd_init_ILI9341_FRMCTR1[] = { ILI9341_FRMCTR1, 0x00, 0x18 };
static const uint8_t lcd_init_ILI9341_DFUNCTR[] = { ILI9341_DFUNCTR, 0x08, 0x82, 0x27 };
static const uint8_t lcd_init_ILI9341_ENGMCTR[] = { ILI9341_ENGMCTR, 0x00 };
static const uint8_t lcd_init_ILI9341_GAMMASET[] = { ILI9341_GAMMASET, 0x01 };
static const uint8_t lcd_init_ILI9341_GMCTRP1[] = { ILI9341_GMCTRP1, \
                                                    0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, \
                                                    0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,};
static const uint8_t lcd_init_ILI9341_GMCTRN1[] = { ILI9341_GMCTRN1, \
                                                    0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,     \
                                                    0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,};
static const uint8_t lcd_init_ILI9341_SLPOUT[] = { ILI9341_SLPOUT };
static const uint8_t lcd_init_ILI9341_DISPON[] = { ILI9341_DISPON };

static const lcd_op_t lcd_init_ili9341_seq[] =
{
        {OP_CMD, sizeof(lcd_init_ILI9341_SWRESET ), lcd_init_ILI9341_SWRESET },
        {OP_DELAY, 120,                 NULL       },
        {OP_CMD, sizeof(lcd_init_ILI9341_DISPOFF ), lcd_init_ILI9341_DISPOFF },
        {OP_DELAY, 120,                 NULL       },
        {OP_CMD, sizeof(lcd_init_ILI9341_PWCTRB  ), lcd_init_ILI9341_PWCTRB  },
        {OP_CMD, sizeof(lcd_init_ILI9341_TIMCTRA ), lcd_init_ILI9341_TIMCTRA },
        {OP_CMD, sizeof(lcd_init_ILI9341_PWCTRSEQ), lcd_init_ILI9341_PWCTRSEQ},
        {OP_CMD, sizeof(lcd_init_ILI9341_PUMP    ), lcd_init_ILI9341_PUMP    },
        {OP_CMD, sizeof(lcd_init_ILI9341_TIMCTRB ), lcd_init_ILI9341_TIMCTRB },
        {OP_CMD, sizeof(lcd_init_ILI9341_PWCTR1  ), lcd_init_ILI9341_PWCTR1  },
        {OP_CMD, sizeof(lcd_init_ILI9341_PWCTR2  ), lcd_init_ILI9341_PWCTR2  },
        {OP_CMD, sizeof(lcd_init_ILI9341_VMCTR1  ), lcd_init_ILI9341_VMCTR1  },
        {OP_CMD, sizeof(lcd_init_ILI9341_VMCTR2  ), lcd_init_ILI9341_VMCTR2  },
        {OP_CMD, sizeof(lcd_init_ILI9341_MADCTL  ), lcd_init_ILI9341_MADCTL  },
        {OP_CMD, sizeof(lcd_init_ILI9341_PIXFMT  ), lcd_init_ILI9341_PIXFMT  },
        {OP_CMD, sizeof(lcd_init_ILI9341_FRMCTR1 ), lcd_init_ILI9341_FRMCTR1 },
        {OP_CMD, sizeof(lcd_init_ILI9341_DFUNCTR ), lcd_init_ILI9341_DFUNCTR },
        {OP_CMD, sizeof(lcd_init_ILI9341_ENGMCTR ), lcd_init_ILI9341_ENGMCTR },
        {OP_CMD, sizeof(lcd_init_ILI9341_GAMMASET), lcd_init_ILI9341_GAMMASET},
        {OP_CMD, sizeof(lcd_init_ILI9341_GMCTRP1 ), lcd_init_ILI9341_GMCTRP1 },
        {OP_CMD, sizeof(lcd_init_ILI9341_GMCTRN1 ), lcd_init_ILI9341_GMCTRN1 },
        {OP_CMD, sizeof(lcd_init_ILI9341_SLPOUT  ), lcd_init_ILI9341_SLPOUT  },
        {OP_DELAY, 120,                 NULL       },
        {OP_CMD, sizeof(lcd_init_ILI9341_DISPON  ), lcd_init_ILI9341_DISPON  },
};

static inline void write_command(uint8_t c)
{
        nrfx_err_t err_code;
        uint8_t tx_buff[1];
        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buff, 1);
        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;
        tx_buff[0] = c;
        err_code = nrfx_spim_xfer_dcx(&spi_ili9341_instance, &xfer_desc, flags, 1);
        APP_ERROR_CHECK(err_code);
}

static inline void write_data(uint8_t c)
{
        nrfx_err_t err_code;
        uint8_t tx_buff[1];
        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buff, 1);

        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;

        tx_buff[0] = c;
        err_code = nrfx_spim_xfer_dcx(&spi_ili9341_instance, &xfer_desc, flags, 0);
        APP_ERROR_CHECK(err_code);
}




void drv_ili9341_proc_op_list(const lcd_op_t *p_op_list)
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
                        nrfx_err_t err_code = nrfx_spim_xfer_dcx(&spi_ili9341_instance, &xfer_desc, flags, 1);
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

static void drv_spim_ili9341_event_handler(nrfx_spim_evt_t * p_event)
{
        if (p_event->type == NRFX_SPIM_EVENT_DONE)
        {
                drv_ili9341_evt_t ili9341_evt;
                ili9341_evt.evt_type = eLCD_ILI9341_EVT_TX_DONE;
                m_cb_ili9341_handler(&ili9341_evt);
        }
}

/**@brief Function to initiate LCD.
 */
void drv_ili9341_init(drv_ili9341_init_t *p_drv_ili9341)
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
        APP_ERROR_CHECK(nrfx_spim_init(&spi_ili9341_instance, &spi_config, NULL, NULL));

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

        m_cb_ili9341_handler = p_drv_ili9341->evt_handler;

        // send LCD init sequence
        drv_ili9341_proc_op_list(lcd_init_ili9341_seq);
}




static void drv_ili9341_display_invert(bool invert)
{
        write_command(invert ? ILI9341_INVON : ILI9341_INVOFF);
}

static void drv_ili9341_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
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
        err_code = nrfx_spim_xfer_dcx(&spi_ili9341_instance, &xfer_desc, flags, 1);
        APP_ERROR_CHECK(err_code);

        tx_buff[0] = 0x2B;
        tx_buff[1] = ( y          >> 8) & 0xFF;
        tx_buff[2] = ( y              ) & 0xFF;
        tx_buff[3] = ((y + h - 1) >> 8) & 0xFF;
        tx_buff[4] = ((y + h - 1)     ) & 0xFF;
        err_code = nrfx_spim_xfer_dcx(&spi_ili9341_instance, &xfer_desc, flags, 1);
        APP_ERROR_CHECK(err_code);
}


void drv_ili9341_put_partial_update(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t const *p_lcd_data)
{
        drv_ili9341_set_window(x, y, w, h);

        uint8_t tx_buff[1];
        nrfx_err_t err_code;

        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buff, 1);
        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;

        // send LCD command
        tx_buff[0] = ILI9341_RAMWR;
        err_code = nrfx_spim_xfer_dcx(&spi_ili9341_instance, &xfer_desc, flags, 1);
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
                err_code = nrfx_spim_xfer_dcx(&spi_ili9341_instance, &xfer_desc, 0, 0);
                APP_ERROR_CHECK(err_code);
        }

}

void drv_ili9341_put_gfx(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t const *p_lcd_data)
{
        drv_ili9341_set_window(x, y, w, h);

        uint8_t tx_buff[1];
        nrfx_err_t err_code;

        nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(tx_buff, 1);
        uint32_t flags = NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;

        // send LCD command
        tx_buff[0] = ILI9341_RAMWR;
        err_code = nrfx_spim_xfer_dcx(&spi_ili9341_instance, &xfer_desc, flags, 1);
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
                err_code = nrfx_spim_xfer_dcx(&spi_ili9341_instance, &xfer_desc, 0, 0);
                APP_ERROR_CHECK(err_code);
        }

}
