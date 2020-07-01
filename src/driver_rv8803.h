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

typedef struct _twi_peipheral
{
    nrf_drv_twi_t *twi_instance;
    uint32_t scl_pin;
    uint32_t sda_pin;
}twi_peripheral_t;

int rv8803_twi_init(twi_peripheral_t *twi_instance);

#ifdef __cplusplus
}
#endif

#endif //DRIVER_RV8803_H
