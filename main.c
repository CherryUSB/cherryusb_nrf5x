/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_drv_power.h"
#include "app_uart.h"
#include "bsp_cli.h"
#include "nrf_cli_uart.h"

#define CDC_DEMO
/**
 * @brief CLI interface over UART
 */
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            4);

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

void uart_error_handle(app_uart_evt_t *p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

void usb_dc_low_level_post_init(void)
{
    /* Enable interrupt globally */
    NRFX_IRQ_PRIORITY_SET(USBD_IRQn, NRFX_USBD_CONFIG_IRQ_PRIORITY);
    NRFX_IRQ_ENABLE(USBD_IRQn);
}

extern void cherry_usb_hal_nrf_power_event(uint32_t event);
static void power_event_handler(nrfx_power_usb_evt_t event)
{
    cherry_usb_hal_nrf_power_event((uint32_t)event);
}

void usb_dc_low_level_pre_init(void)
{
    uint32_t usb_reg;
    const nrfx_power_usbevt_config_t config = {.handler = power_event_handler};
    nrfx_power_usbevt_init(&config);
    nrfx_power_usbevt_enable();
    usb_reg = NRF_POWER->USBREGSTATUS;

    if (usb_reg & POWER_USBREGSTATUS_VBUSDETECT_Msk)
    {
        cherry_usb_hal_nrf_power_event(NRFX_POWER_USB_EVT_DETECTED);
    }

    volatile uint32_t count = 10000;
    while (count--)
    {
    }
		
    if (usb_reg & POWER_USBREGSTATUS_OUTPUTRDY_Msk)
    {
        cherry_usb_hal_nrf_power_event(NRFX_POWER_USB_EVT_READY);
    }
}

void usb_clear_pending_irq(void)
{
    NVIC_ClearPendingIRQ(USBD_IRQn);
}

void usb_disable_irq(void)
{
    NVIC_DisableIRQ(USBD_IRQn);
}

int main(void)
{
    uint32_t err_code;
    UNUSED_RETURN_VALUE(NRF_LOG_INIT(NULL));
    nrf_drv_clock_init();
    nrf_drv_power_init(NULL);
    app_timer_init();

    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        UART_HWFC,
        false,
#if defined(UART_PRESENT)
        NRF_UART_BAUDRATE_115200
#else
        NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);

    printf("USBD example started. \r\n");

#ifdef CDC_DEMO
    extern void cdc_acm_init(void);
    cdc_acm_init();
    printf("cdc acm example started. \r\n");
    nrf_delay_ms(5000);
#elif defined MSC_DEMO
    extern void msc_ram_init(void);
    msc_ram_init();
    printf("msc ram example started. \r\n");
    nrf_delay_ms(5000);
#elif defined HID_DEMO
    extern void hid_keyboard_init(void);
    hid_keyboard_init();
    printf("hid keyboard example started. \r\n");
    nrf_delay_ms(5000);
#elif defined CDC_MULTI_DEMO
    extern void cdc_acm_multi_init(void);
    cdc_acm_multi_init();
    nrf_delay_ms(5000);
#endif

    while (true)
    {
#ifdef CDC_DEMO
        extern void cdc_acm_data_send_with_dtr_test(void);
        cdc_acm_data_send_with_dtr_test();
#elif defined(HID_DEMO)
        extern void hid_keyboard_test(void);
        hid_keyboard_test();
        nrf_delay_ms(2000);
#endif
    }
}
