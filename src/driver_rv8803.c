#include "driver_rv8803.h"

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample;

twi_peripheral_t *ptrToTWI = NULL;

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */
void LM75B_set_mode(void)
{
    // ret_code_t err_code;

    // /* Writing to LM75B_REG_CONF "0" set temperature sensor in NORMAL mode. */
    // uint8_t reg[2] = {LM75B_REG_CONF, NORMAL_MODE};
    // err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, sizeof(reg), false);
    // APP_ERROR_CHECK(err_code);
    // while (m_xfer_done == false);

    // /* Writing to pointer byte. */
    // reg[0] = LM75B_REG_TEMP;
    // m_xfer_done = false;
    // err_code = nrf_drv_twi_tx(&m_twi, LM75B_ADDR, reg, 1, false);
    // APP_ERROR_CHECK(err_code);
    // while (m_xfer_done == false);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief I2C initialization.
 */
int twi_init(nrf_drv_twi_t *twi_instance, uint32_t scl_pin, uint32_t sda_pin)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = scl_pin,
       .sda                = sda_pin,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(twi_instance, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(twi_instance);

    return err_code;
}

/**
 * @brief TWI driver initialisation
 * This function configures peripheral TWI and takes a copy of the pointer
 * to TWI instance.
 * @param[in]   pointer to structure containing the necessary
 *              information to configure peripheral
 * @return      O on success
 */
int rv8803_twi_init(twi_peripheral_t *twi_instance)
{
    ptrToTWI = twi_instance;
    return twi_init(ptrToTWI->twi_instance, ptrToTWI->scl_pin, ptrToTWI->sda_pin);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
// static void read_sensor_data()
// {
//     m_xfer_done = false;

//     /* Read 1 byte from the specified address - skip 3 bits dedicated for fractional part of temperature. */
//     ret_code_t err_code = nrf_drv_twi_rx(&m_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
//     APP_ERROR_CHECK(err_code);
// }
