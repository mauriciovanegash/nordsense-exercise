/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
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
/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "driver_rv8803.h"

// TWI instance ID.
#define TWI_INSTANCE_ID     0

// TWI instance.
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
static volatile bool m_gpio_rise = false;

/**
 * @brief GPIO events handler.
 */
void gpio_interrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    m_gpio_rise = true;
}

/**
 * @brief GPIO configuration
 * This function configures a GPIO as input and enables
 * the interrupt.
 * @return 0 on success
 */
int gpio_init(nrfx_gpiote_pin_t pin)
{
    ret_code_t err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
    if (err_code != 0)
        return err_code;

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(pin, &in_config, gpio_interrupt_handler);
    APP_ERROR_CHECK(err_code);
    if (err_code != 0)
        return err_code;

    nrf_drv_gpiote_in_event_enable(pin, true);
    return 0;
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    twi_peripheral_t i2c;
    i2c.twi_instance = (nrf_drv_twi_t *)&m_twi;
    i2c.scl_pin = ARDUINO_SCL_PIN;
    i2c.sda_pin = ARDUINO_SDA_PIN;
    if (rv8803_twi_init(&i2c) != 0) {
        NRF_LOG_ERROR("\r\nMAIN: TWI initialisation fail!");
    }
    else {
        NRF_LOG_INFO("\r\nTWI driver initialised!");
    }
    NRF_LOG_FLUSH();

    // Configure board.
    bsp_board_init(BSP_INIT_LEDS);
    NRF_LOG_INFO("\r\nLEDs initialised.");
    NRF_LOG_FLUSH();

    // Test unix-time writing
    int32_t unix_time = 1593985469;
    if (rv8803_set_unix_time(&i2c, unix_time) != 0)
    {
        NRF_LOG_ERROR("\r\nMAIN: RV8803 test set unix-time fail!");
        NRF_LOG_FLUSH();
    }

    // Test unix-time readings
    unix_time = 0;
    if (rv8803_get_unix_time(&i2c, &unix_time) == 0)
    {
        NRF_LOG_INFO("\r\nMAIN: RV8803 get unix-time\r\n %s", ctime((time_t *)&unix_time));
    }
    else
    {
        NRF_LOG_ERROR("\r\nMAIN: RV8803 test get unix-time fail!");
    }
    NRF_LOG_FLUSH();

    while (true)
    {
        // do
        // {
        //     __WFE();
        // }while (m_xfer_done == false);

        for (int i = 0; i < LEDS_NUMBER; i++)
        {
            bsp_board_led_invert(i);
            nrf_delay_ms(500);
        }
        // Test time evolution
        if (rv8803_test_time(&i2c) != 0)
        {
            NRF_LOG_ERROR("\r\nMAIN: RV8803 test time-evolution fail!");
        }
        NRF_LOG_FLUSH();
    }
}

/** @} */
