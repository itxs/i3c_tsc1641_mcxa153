/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright Timur Khasanshin 2024
 * Modifications: Temperature sensor from example has been replaced by TSC1641 sensor, improved debug logging
 */

/*  SDK Included Files */
#include "board.h"
#include "clock_config.h"
#include "drivers/external/tsc1641.h"
#include "drivers/peripheral/i3c.h"
#include "fsl_debug_console.h"
#include "fsl_i3c.h"
#include "pin_mux.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SENSOR_READOUT_PERIOD_US 20000U /**< Period in us for reading sensor data in the main loop */
#define SENSOR_ADDR              0x08U  /**< Dynamic address of the sensor device */
#define EXT_SHUNT_RES_OHMS       0.005f /**< External shunt resistance in ohms for current value calculation */

static tsc1641_handle_t g_tsc1641_handle = { .dbg_log_handler = i3c_err_dbg_log };

/*!
 * @brief Main function
 */
int main (void)
{
    /* Attach clock to I3C 24MHZ */
    CLOCK_SetClockDiv(kCLOCK_DivI3C0_FCLK, 2U);
    CLOCK_AttachClk(kFRO_HF_DIV_to_I3C0FCLK);

    BOARD_InitBootPins();
    BOARD_BootClockFRO48M();
    BOARD_InitDebugConsole();

    PRINTF("\r\n\r\n\r\n\r\n===============================================================\r\n");
    PRINTF("FW Start: I3C master read sensor data example.\r\n");

    i3c_init();

    status_t result = i3c_set_dynamic_address(I3C0, I2C_TSC1641_ADDR, SENSOR_ADDR);
    if (result != kStatus_Success)
    {
        PRINTF("TSC1641 set dynamic address failed.\r\n");
        assert(0);
    }

    tsc1641_config_t tsc1641_cfg;
    tsc1641_cfg.write_handler      = i3c_write;
    tsc1641_cfg.read_handler       = i3c_read;
    tsc1641_cfg.addr               = SENSOR_ADDR;
    tsc1641_cfg.shunt_val          = EXT_SHUNT_RES_OHMS;
    tsc1641_cfg.mode               = TSC1641_Mode_VshloadCont;
    tsc1641_cfg.conversion_time    = TSC1641_Conf_CT_1024;
    tsc1641_cfg.reset_state        = false;
    tsc1641_cfg.temp_sensor_enable = true;
    result                         = tsc1641_init(&g_tsc1641_handle, &tsc1641_cfg);
    if (result != kStatus_Success)
    {
        PRINTF("TSC1641 init failed.\r\n");
        assert(0);
    }

    float voltage = 0, current = 0, power = 0, temperature = 0;

    while (1)
    {
        result = tsc1641_read_all_data(&g_tsc1641_handle, &voltage, &current, &power, &temperature);
        if (result != kStatus_Success)
        {
            PRINTF("TSC1641 read data failed.\r\n");
        }
        else
        {
            PRINTF("\rVoltage:%6.3fV Current:%6.3fA Power:%6.3fW Temp:%5.1f°C", voltage, current, power, temperature);
        }
        SDK_DelayAtLeastUs(SENSOR_READOUT_PERIOD_US, CLOCK_GetCoreSysClkFreq());
    }
}
