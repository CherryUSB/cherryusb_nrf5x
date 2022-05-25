/**
 ******************************************************************************
 * @file          cdc_dfu.c
 * @brief
 * @author        Li Guo
 *                1570139720@qq.com
 * @version       1.0
 * @date          2022.05.10
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright 2021 Li Guo.
 * All rights reserved.</center></h2>
 *
 * @htmlonly
 * <span style='font-weight: bold'>History</span>
 * @endhtmlonly
 * 版本|作者|时间|描述
 * ----|----|----|----
 * 1.0|Li Guo|2022.05.10|创建文件
 ******************************************************************************
 */

/* include -------------------------------------------------------------------*/
#include <stdint.h>
#include "usbd_core.h"
/* marco ---------------------------------------------------------------------*/
#define NRF_APP_START_ADDRESS 0
#define NRF_APP_LENGTH 0
#define NRF_DFU_IN_EP 0

/* typedef -------------------------------------------------------------------*/

/* declare -------------------------------------------------------------------*/

/* variable ------------------------------------------------------------------*/
uint8_t data_buff[64];
volatile bool need_handle = false;
volatile bool is_jump = false;
uint32_t start_add = NRF_APP_START_ADDRESS;
uint8_t ack_data = {0, 9, 0, 3};
/* code ----------------------------------------------------------------------*/
static void nrf_flash_write_buff(uint32_t address, uint8_t *data, uint32_t length)
{
}

static void nrf_flash_write_erase(uint32_t address, uint32_t length)
{
}

static inline void nrf_jump_to_app(void)
{
}

void nrf_cdc_acm_out(uint8_t ep)
{
    /*!< receive data from pc */
    if (usbd_ep_read(ep, data_buff, 64, NULL))
    {
        uint32_t *data = data_buff;
        /*!< receive data successfully */
        if (*data == 0x0309)
        {
            is_jump = true;
            return;
        }
        need_handle = true;
    }
}

void nrf_dfu_init(void)
{
    /*!< Erase APP */
    nrf_flash_write_erase(NRF_APP_START_ADDRESS, NRF_APP_LENGTH);
}

void nrf_dfu_task_process(void)
{
    while (true)
    {
        if (need_handle)
        {
            /*!< Storge the data to flash and send ack to host */
            nrf_flash_write_buff(start_add, data_buff, 64);
            start_add += 64;

            usbd_ep_write(NRF_DFU_IN_EP, ack_data, 4, NULL);
            need_handle = false;
        }
        if (is_jump == true)
        {
            break;
        }
    }
    /*!< disable interrupt and jump */
    nrf_jump_to_app();
}

/************************ (C) COPYRIGHT 2021 Li Guo *****END OF FILE*************/
