#include "driver_rv8803.h"

/// @TODO: Global variables are a problem for the scalability of this
///        driver.

#define BUFFER_LENGTH   66 ///< 64 bytes are enough to read all internal registers in one TWI transfer
static volatile bool m_tx_done = false;///< Indicates if TX operation on TWI has ended.
static volatile bool m_rx_done = false;///< Indicates if RX operation on TWI has ended.
uint8_t twi_buffer[BUFFER_LENGTH]; ///< Internal buffer for TWI transaction.

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
 * This function initialise the TWI interface.
 * @param twi_instance[in]: Instance of the TWI peripheral
 * @param scl_pin:          Pin number for SCL
 * @param sda_pin:          Pin number for SDA
 * @return          0 on success
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
 * @param ptrToTWI[in]: Pointer to peripheral information
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
 * @return      0 on success.
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
 * @return      0 on success.
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
 * @param ptrToTWI[in]:     Pointer to peripheral information
 * @return      O on success
 */
int rv8803_twi_init(twi_peripheral_t *ptrToTWI)
{
    int ret = twi_init(ptrToTWI->twi_instance, ptrToTWI->scl_pin, ptrToTWI->sda_pin);
    if (ret != 0)
    {
        NRF_LOG_ERROR("DRIVER-RV8803: TWI initialisation fail at %s:%d.", __FILE__, __LINE__);
        return ret;
    }
    for (int i=1; i<5; i++)
    {
        ret = rv8803_unset_alarm(ptrToTWI, (enum ALARM_TYPE)i, 0);
        if (ret != 0)
        {
            NRF_LOG_ERROR("DRIVER-RV8803: Alarm disable fail for type %d at %s:%d.", i, __FILE__, __LINE__);
            return ret;
        }
    }

    // Clear and disable interrupt generation
    uint8_t value = 0;
    ret = TWI_write(ptrToTWI->twi_instance, FLAG_REGISTER, &value, 1);
    if (ret != 0)
        return ret;
    return TWI_write(ptrToTWI->twi_instance, CONTROL_REGISTER, &value, 1);
}

/**
 * @brief Set alarm
 * This function enables one of the four different
 * alarms available on the chip.
 * @param ptrToTWI[in]:     Pointer to peripheral information
 * @param alarm:            Type of alarm (enum type)
 * @param value:            Value to be set
 * @return          0 on success
 */
int rv8803_set_alarm(twi_peripheral_t *ptrToTWI, enum ALARM_TYPE alarm, uint8_t value)
{
    uint8_t reg = 0;
    // Enable interrupt generation
    uint8_t reg_value = 0x08;
    int ret = TWI_write(ptrToTWI->twi_instance, CONTROL_REGISTER, &reg_value, 1);
    if (ret != 0)
        return ret;
    switch (alarm)
    {
        case MIN:
            reg = ALARM_MINUTES;
            reg_value = DEC2BCD((value & 0x3f));
            break;
        case HOUR:
            reg = ALARM_HOURS;
            reg_value = DEC2BCD((value & 0x3f));
            break;
        case WDAY:
            reg = ALARM_WDAYS;
            reg_value = DEC2ONEHOT(value);
            break;
        case DATE:
            reg = ALARM_DATE;
            reg_value = DEC2BCD((value & 0x3f));
            break;
        default:
            break;
    }
    return TWI_write(ptrToTWI->twi_instance, reg, &reg_value, 1);
}

/**
 * @brief Unset alarm
 * This function disables one of the four different
 * alarms available on the chip.
 * @param ptrToTWI[in]:     Pointer to peripheral information
 * @param alarm:            Type of alarm (enum type)
 * @param value:            Value to be set
 * @return          0 on success
 */
int rv8803_unset_alarm(twi_peripheral_t *ptrToTWI, enum ALARM_TYPE alarm, uint8_t value)
{
    uint8_t reg = 0;
    // Disable interrupt generation
    uint8_t reg_value = 0x0;
    int ret = TWI_write(ptrToTWI->twi_instance, CONTROL_REGISTER, &reg_value, 1);
    if (ret != 0)
        return ret;
    switch (alarm)
    {
        case MIN:
            reg = ALARM_MINUTES;
            break;
        case HOUR:
            reg = ALARM_HOURS;
            break;
        case WDAY:
            reg = ALARM_WDAYS;
            break;
        case DATE:
            reg = ALARM_DATE;
            break;
        default:
            break;
    }
    reg_value = 0x80;
    return TWI_write(ptrToTWI->twi_instance, reg, &reg_value, 1);
}

/**
 * @brief Get alarm
 * This function reads one of the four different
 * alarms available on the chip.
 * @param ptrToTWI[in]:     Pointer to peripheral information
 * @param alarm:            Type of alarm (enum type)
 * @param value[out]:       Value to be set
 * @return          0 on success
 */
int rv8803_get_alarm(twi_peripheral_t *ptrToTWI, enum ALARM_TYPE alarm, uint8_t *value)
{
    uint8_t reg = 0;
    uint8_t reg_value = 0;
    switch (alarm)
    {
        case MIN:
            reg = ALARM_MINUTES;
            break;
        case HOUR:
            reg = ALARM_HOURS;
            break;
        case WDAY:
            reg = ALARM_WDAYS;
            break;
        case DATE:
            reg = ALARM_DATE;
            break;
        default:
            break;
    }
    int ret = TWI_read(ptrToTWI->twi_instance, reg, &reg_value, 1);
    if (alarm == ALARM_WDAYS)
        *value = ONEHOT2DEC(reg_value);
    else
        *value = BCD2DEC(reg_value);

    return ret;
}

/**
 * @brief Get interrupt status
 * This function reads FLAG register.
 * @param ptrToTWI[in]:     Pointer to peripheral information
 * @param value[out]:       Value of the register
 * @return          0 on success
 */
int rv8803_get_interrupt_status(twi_peripheral_t *ptrToTWI, uint8_t *value)
{
    return TWI_read(ptrToTWI->twi_instance, FLAG_REGISTER, value, 1);
}
