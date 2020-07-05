#ifndef DRIVER_RV8803_H
#define DRIVER_RV8803_H

#include <time.h>
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
#define CHIP_ADDR       (0x32)

// Internal registers
#define TIME_MIN        (0x01)
#define TIME_SEC        (0x00)
#define TIME_HOUR       (0x02)
#define TIME_WEEKDAY    (0x03)
#define TIME_DATE       (0x04)
#define TIME_MONTH      (0x05)
#define TIME_YEAR       (0x06)

// Error definition
#define RV8803_TWI_TIMEOUT          -1
#define RV8803_UNEXPECTED_LENGTH    -2

// Macros
#define BCD2DEC(value)              (((value) >> 4) & 0xf)*10 + ((value) & 0xf)
#define DEC2BCD(value)              (((((value) - (value)%10)/10) << 4) + ((value)%10 & 0xf))
#define SHIFTONEHOT(value, pos)     (((value) >> pos) & 0x1)
#define ONEHOT2DEC(value)           ((SHIFTONEHOT((value), 0))*1 + (SHIFTONEHOT((value), 1))*2 + (SHIFTONEHOT((value), 2))*3 + \
                                     (SHIFTONEHOT((value), 3))*4 + (SHIFTONEHOT((value), 5))*6 + (SHIFTONEHOT((value), 6))*7 - 1)
#define DEC2ONEHOT(value)           (0x1 << (value))

typedef struct _twi_peipheral
{
    nrf_drv_twi_t *twi_instance;
    uint32_t scl_pin;
    uint32_t sda_pin;
}twi_peripheral_t;

int rv8803_twi_init(twi_peripheral_t *ptrToTWI);
int rv8803_test_time(twi_peripheral_t *ptrToTWI);
int rv8803_set_unix_time(twi_peripheral_t *ptrToTWI, int32_t unix_time);
int rv8803_get_unix_time(twi_peripheral_t *ptrToTWI, int32_t *unix_time);

#ifdef __cplusplus
}
#endif

#endif //DRIVER_RV8803_H
