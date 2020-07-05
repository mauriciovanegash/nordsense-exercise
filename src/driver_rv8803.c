#include "driver_rv8803.h"

// Indicates if operation on TWI has ended.
static volatile bool m_tx_done = false;
static volatile bool m_rx_done = false;

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
 * @param twi_instance[in]: Instance of the TWI peripheral
 * @param addr:             Internal register address
 * @param data[out]:        Pointer to buffer
 * @param length:           Number of bytes to be transferred
 * @return      0 on success.
 */
static int TWI_read(nrf_drv_twi_t *twi_instance, uint8_t addr, uint8_t *data, uint8_t length)
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
    ret_code_t err_code = nrf_drv_twi_tx(twi_instance, CHIP_ADDR, twi_buffer, 1, false);
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
    err_code = nrf_drv_twi_rx(twi_instance, CHIP_ADDR, data, length+1);
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
 * @param twi_instance[in]: Instance of the TWI peripheral
 * @param addr:             Internal register address
 * @param data[in]:         Pointer to buffer
 * @param length:           Number of bytes to be transferred
 * @return      0 on success.
 */
static int TWI_write(nrf_drv_twi_t *twi_instance, uint8_t addr, uint8_t *data, uint8_t length)
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
    ret_code_t err_code = nrf_drv_twi_tx(twi_instance, CHIP_ADDR, twi_buffer, length+1, false);
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
 * @brief Function to read min and seconds - a test function.
 * @param ptrToTWI: Pointer to peripheral information
 * @return      0 on success
 */
int rv8803_test_time(twi_peripheral_t *ptrToTWI)
{
    uint8_t reg[3];
    int ret = TWI_read(ptrToTWI->twi_instance, TIME_SEC, reg, 3);
    NRF_LOG_INFO("DRIVER-RV8803: hour:min:sec - %02x:%02x:%02x", reg[2], reg[1], reg[0]);
    NRF_LOG_FLUSH();
    return ret;
}

/**
 * @brief Set UNIX time.
 * This function set UNIX time on external micro crystal
 * @param ptrToTWI[in]: Pointer to peripheral information
 * @param unix_time:    UNIX epoch
 * @returns     0 on success.
 */
int rv8803_set_unix_time(twi_peripheral_t *ptrToTWI, int32_t unix_time)
{
    uint8_t reg[8];
    time_t utime = unix_time;
    struct tm *timeinfo = gmtime(&utime);
    // Convert "tm" structure into register values
    reg[0] = DEC2BCD(timeinfo->tm_sec);
    reg[1] = DEC2BCD(timeinfo->tm_min);
    reg[2] = DEC2BCD(timeinfo->tm_hour);
    reg[3] = DEC2ONEHOT(timeinfo->tm_wday);
    reg[4] = DEC2BCD(timeinfo->tm_mday);
    reg[5] = DEC2BCD(timeinfo->tm_mon);
    // Function gmtime returns years starting from 1900.
    // The chip stores years from 00 to 99 - it start counting from 2000
    // The actual acount for the years is "tm_year + 1900 - 2000"
    reg[6] = DEC2BCD(timeinfo->tm_year - 100);
    return TWI_write(ptrToTWI->twi_instance, TIME_SEC, reg, 8);
}

/**
 * @brief Set UNIX time.
 * This function set UNIX time on external micro crystal
 * @param ptrToTWI[in]:     Pointer to peripheral information
 * @param unix_time[out]:   UNIX epoch
 * @returns     0 on success.
 */
int rv8803_get_unix_time(twi_peripheral_t *ptrToTWI, int32_t *unix_time)
{
    uint8_t reg[8];
    memset(reg, 0, 8);
    int ret = TWI_read(ptrToTWI->twi_instance, TIME_SEC, reg, 8);
    struct tm timeinfo;
    // Convert register values into "tm" structure
    timeinfo.tm_sec = BCD2DEC(reg[0]);
    timeinfo.tm_min = BCD2DEC(reg[1]);
    timeinfo.tm_hour = BCD2DEC(reg[2]);
    timeinfo.tm_wday = ONEHOT2DEC(reg[3]);
    timeinfo.tm_mday = BCD2DEC(reg[4]);
    timeinfo.tm_mon = BCD2DEC(reg[5]);
    timeinfo.tm_year = BCD2DEC(reg[6]) + 100;
    timeinfo.tm_isdst = 0;
    *unix_time = (int32_t)mktime(&timeinfo);
    return ret;
}

/**
 * @brief TWI driver initialisation
 * This function configures peripheral TWI and takes a copy of the pointer
 * to TWI instance.
 * @param ptrToTWI[in]:     pointer to structure containing the necessary
 *                          information to configure peripheral
 * @return      O on success
 */
int rv8803_twi_init(twi_peripheral_t *ptrToTWI)
{
    return twi_init(ptrToTWI->twi_instance, ptrToTWI->scl_pin, ptrToTWI->sda_pin);
}
