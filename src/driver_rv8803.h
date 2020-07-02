#ifndef DRIVER_RV8803_H
#define DRIVER_RV8803_H

#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#ifdef __cplusplus
extern "C" {
#endif

// CHIP slave address
#define CHIP_ADDR       (0x52)

// Internal registers
#define UNIX_TIME_0     (0x1b)
#define UNIX_TIME_1     (0x1c)
#define UNIX_TIME_2     (0x1d)
#define UNIX_TIME_3     (0x1e)
#define CHIP_ID         (0x28)

// Error definition
#define RV8803_TWI_TIMEOUT          -1
#define RV8803_UNEXPECTED_LENGTH    -2

typedef struct _twi_peipheral
{
    nrf_drv_twi_t *twi_instance;
    uint32_t scl_pin;
    uint32_t sda_pin;
}twi_peripheral_t;

int rv8803_twi_init(twi_peripheral_t *twi_instance);
int rv8803_read_ID(void);
int rv8803_set_unix_time(uint32_t unix_time);
int rv8803_get_unix_time(uint32_t *unix_time);

#ifdef __cplusplus
}
#endif

#endif //DRIVER_RV8803_H
