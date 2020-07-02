#include "driver_rv8803.h"

// Indicates if operation on TWI has ended.
static volatile bool m_tx_done = false;
static volatile bool m_rx_done = false;
// Pointer to TWI instance
twi_peripheral_t *ptrToTWI = NULL;
// 64 bytes are enough to read all internal registers in one TWI transfer
#define BUFFER_LENGTH   66
// Internal buffer for TWI transaction
uint8_t twi_buffer[BUFFER_LENGTH];

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (!m_tx_done)
                m_tx_done = (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX);
            if (!m_rx_done)
                m_rx_done = (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX);
            break;
        default:
            break;
    }
}

/**
 * @brief I2C initialization.
 */
static int twi_init(nrf_drv_twi_t *twi_instance, uint32_t scl_pin, uint32_t sda_pin)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = scl_pin,
       .sda                = sda_pin,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(twi_instance, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(twi_instance);

    return err_code;
}

/**
 * @brief TWI_read
 * This function reads through the TWI interface.
 * @param addr[in]:     Internal register address
 * @param data[out]:    Pointer to buffer
 * @param length:   Number of bytes to be transferred
 * @return      0 on success.
 */
static int TWI_read(uint8_t addr, uint8_t *data, uint8_t length)
{
    if (length > BUFFER_LENGTH-2) {
        NRF_LOG_ERROR("DRIVER-RV8803: Requested transfer length higher than %d at %s:%d.", BUFFER_LENGTH-2, __FILE__, __LINE__);
        return RV8803_UNEXPECTED_LENGTH;
    }
    int ret = 0;
    uint32_t timeout = 1000000; // Huge value to deal with worst scenario
    // Request read
    twi_buffer[0] = addr;
    m_tx_done = false;
    ret_code_t err_code = nrf_drv_twi_tx(ptrToTWI->twi_instance, CHIP_ADDR, twi_buffer, 1, false);
    APP_ERROR_CHECK(err_code);
    if (err_code != 0) {
        m_tx_done = true;
        return err_code;
    }
    // Wait for transfer to be finished
    while (!m_tx_done && (--timeout > 0));
    if (timeout == 0) {
        NRF_LOG_ERROR("DRIVER-RV8803: TWI timeout at %s:%d.", __FILE__, __LINE__);
        m_tx_done = true;
        return RV8803_TWI_TIMEOUT;
    }
    // Actual read
    timeout = 1000000;
    m_rx_done = false;
    err_code = nrf_drv_twi_rx(ptrToTWI->twi_instance, CHIP_ADDR, twi_buffer, length);
    APP_ERROR_CHECK(err_code);
    if (err_code != 0) {
        m_rx_done = true;
        return err_code;
    }
    // Wait for transfer to be finished
    while (!m_rx_done && (--timeout > 0));
    if (timeout == 0) {
        NRF_LOG_ERROR("DRIVER-RV8803: TWI timeout at %s:%d.", __FILE__, __LINE__);
        ret = RV8803_TWI_TIMEOUT;
        m_rx_done = true;
    }
    return ret;
}

/**
 * @brief TWI_write
 * This function writes through the TWI interface.
 * @param addr[in]:     Internal register address
 * @param data[in]:     Pointer to buffer
 * @param length:       Number of bytes to be transferred
 * @return      0 on success.
 */
static int TWI_write(uint8_t addr, uint8_t *data, uint8_t length)
{
    if (length > BUFFER_LENGTH-2) {
        NRF_LOG_ERROR("DRIVER-RV8803: Requested transfer length higher than %d at %s:%d.", BUFFER_LENGTH-2, __FILE__, __LINE__);
        return RV8803_UNEXPECTED_LENGTH;
    }
    int ret = 0;
    uint32_t timeout = 1000000; // Huge value to deal with worst scenario
    // Define buffer to be sent
    twi_buffer[0] = addr;
    memcpy(&twi_buffer[1], data, length);
    m_tx_done = false;
    ret_code_t err_code = nrf_drv_twi_tx(ptrToTWI->twi_instance, CHIP_ADDR, twi_buffer, length+1, false);
    APP_ERROR_CHECK(err_code);
    if (err_code != 0) {
        m_tx_done = true;
        return err_code;
    }
    // Wait for transfer to be finished
    while (!m_tx_done && (--timeout > 0));
    if (timeout == 0) {
        NRF_LOG_ERROR("DRIVER-RV8803: TWI timeout at %s:%d.", __FILE__, __LINE__);
        ret = RV8803_TWI_TIMEOUT;
        m_tx_done = true;
    }
    return ret;
}

/**
 * @brief Function to read chip-ID - a test function.
 */
int rv8803_read_ID(void)
{
    uint8_t reg = 0;
    int ret = TWI_read(CHIP_ID, &reg, 1);
    NRF_LOG_INFO("DRIVER-RV8803: CHIP-ID - %d", reg);
    return ret;
}

/**
 * @brief Set UNIX time.
 * This function set UNIX time on external micro crystal
 * @param unix_time:    UNIX epoch
 * @returns     0 on success.
 */
int rv8803_set_unix_time(uint32_t unix_time)
{
    uint8_t reg[4];
    reg[0] = (uint8_t)((unix_time >> 0) & 0xff);
    reg[1] = (uint8_t)((unix_time >> 8) & 0xff);
    reg[2] = (uint8_t)((unix_time >> 16) & 0xff);
    reg[3] = (uint8_t)((unix_time >> 24) & 0xff);
    return TWI_write(UNIX_TIME_0, reg, 4);
}

/**
 * @brief Set UNIX time.
 * This function set UNIX time on external micro crystal
 * @param unix_time:    UNIX epoch
 * @returns     0 on success.
 */
int rv8803_get_unix_time(uint32_t *unix_time)
{
    uint8_t reg[4];
    reg[0] = 0;
    reg[1] = 0;
    reg[2] = 0;
    reg[3] = 0;
    int ret = TWI_read(CHIP_ID, reg, 4);
    *unix_time = (uint32_t)((reg[3] >> 0) & 0xff) | ((reg[2] >> 8) & 0xff) | ((reg[1] >> 16) & 0xff) | ((reg[0] >> 24) & 0xff);
    return ret;
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
//     ret_code_t err_code = nrf_drv_twi_rx(ptrToTWI->twi_instance, CHIP_ADDR, &m_sample, sizeof(m_sample));
//     APP_ERROR_CHECK(err_code);
// }
