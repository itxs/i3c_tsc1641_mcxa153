/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  Standard C Included Files */
#include <string.h>
/*  SDK Included Files */
#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_i3c.h"
#include "pin_mux.h"
#include "tsc1641.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define SENSOR_READOUT_PERIOD_US 20000U
#define SENSOR_ADDR              0x08U

#define I3C_MASTER_CLOCK_FREQUENCY CLOCK_GetI3CFClkFreq()
#define I3C_TIME_OUT_INDEX         100000000U

#define R_SHUNT_RESISTANCE 0.005f

#define I3C_BROADCAST_ADDR 0x7EU
#define CCC_RSTDAA         0x06U
#define CCC_SETDASA        0x87U

#ifndef I2C_BAUDRATE
#define I2C_BAUDRATE 400000U
#endif
#ifndef I3C_OD_BAUDRATE
#define I3C_OD_BAUDRATE 1000000U
#endif
#ifndef I3C_PP_BAUDRATE
#define I3C_PP_BAUDRATE 1000000U
#endif

static void i3c_master_callback (I3C_Type * base, i3c_master_handle_t * handle, status_t status, void * userData);
static void i3c_err_dbg_log (status_t err);

static volatile status_t g_completion_status;
static volatile bool g_master_completion_flag;
static i3c_master_handle_t g_i3c_m_handle;
static tsc1641_handle_t g_tsc1641_handle                           = { .dbg_log_handler = i3c_err_dbg_log };
static const i3c_master_transfer_callback_t global_master_callback = { NULL, NULL, i3c_master_callback };
static const char * g_i3c_err_arr[]                                = {
    "The master is already performing a transfer",
    "The slave driver is idle",
    "The slave device sent a NAK in response to an address",
    "The slave device sent a NAK in response to a write",
    "The master terminates slave read",
    "Parity error from DDR read",
    "CRC error from DDR read",
    "Read from M/SRDATAB register when FIFO empty",
    "Write to M/SWDATAB register when FIFO full",
    "Message SDR/DDR mismatch or read/write message in wrong state",
    "Invalid use of request",
    "The module has stalled too long in a frame",
    "The I3C slave count has exceed the definition in I3C_MAX_DEVCNT",
    "The I3C slave event IBI or MR or HJ won the arbitration on a header address",
    "Slave internal from-bus buffer/FIFO overrun",
    "Slave internal to-bus buffer/FIFO underrun",
    "Slave internal from-bus buffer/FIFO underrun and NACK error",
    "Slave invalid start flag",
    "SDR parity error",
    "S0 or S1 error",
};

static void i3c_master_callback (I3C_Type * base, i3c_master_handle_t * handle, status_t status, void * userData)
{
    if (status == kStatus_Success)
    {
        g_master_completion_flag = true;
    }

    g_completion_status = status;
}

static void i3c_err_dbg_log (status_t err)
{
    if (err != kStatus_Success)
    {
        PRINTF("I3C Error: %s.\r\n", g_i3c_err_arr[err - kStatus_I3C_Busy]);
    }
}

status_t I3C_WriteSensor (uint8_t dev_addr, uint32_t reg_addr, uint8_t * data, size_t len)
{
    i3c_master_transfer_t masterXfer = { 0 };
    masterXfer.slaveAddress          = dev_addr;
    masterXfer.direction             = kI3C_Write;
    masterXfer.busType               = kI3C_TypeI3CSdr;
    masterXfer.subaddress            = reg_addr;
    masterXfer.subaddressSize        = 1;
    masterXfer.data                  = data;
    masterXfer.dataSize              = len;
    masterXfer.flags                 = kI3C_TransferDefaultFlag;

    g_master_completion_flag = false;
    g_completion_status      = kStatus_Success;
    status_t result          = I3C_MasterTransferNonBlocking(I3C0, &g_i3c_m_handle, &masterXfer);
    i3c_err_dbg_log(result);
    if (kStatus_Success != result)
    {
        return result;
    }

    uint32_t timeout = 0U;
    while (!g_master_completion_flag)
    {
        timeout++;
        if ((g_completion_status != kStatus_Success) || (timeout > I3C_TIME_OUT_INDEX))
        {
            break;
        }
    }

    return (timeout == I3C_TIME_OUT_INDEX) ? kStatus_Timeout : g_completion_status;
}

status_t I3C_ReadSensor (uint8_t dev_addr, uint32_t reg_addr, uint8_t * data, size_t len)
{
    i3c_master_transfer_t masterXfer = { 0 };
    masterXfer.slaveAddress          = dev_addr;
    masterXfer.direction             = kI3C_Read;
    masterXfer.busType               = kI3C_TypeI3CSdr;
    masterXfer.subaddress            = reg_addr;
    masterXfer.subaddressSize        = 1;
    masterXfer.data                  = data;
    masterXfer.dataSize              = len;
    masterXfer.flags                 = kI3C_TransferDefaultFlag;

    g_master_completion_flag = false;
    g_completion_status      = kStatus_Success;
    status_t result          = I3C_MasterTransferNonBlocking(I3C0, &g_i3c_m_handle, &masterXfer);
    i3c_err_dbg_log(result);
    if (kStatus_Success != result)
    {
        return result;
    }

    uint32_t timeout = 0U;
    while (!g_master_completion_flag)
    {
        timeout++;
        if ((g_completion_status != kStatus_Success) || (timeout > I3C_TIME_OUT_INDEX))
        {
            break;
        }
    }

    return (timeout == I3C_TIME_OUT_INDEX) ? kStatus_Timeout : g_completion_status;
}

status_t i3c_set_dynamic_address (I3C_Type * i3c, uint8_t staticAddr, uint8_t dynamicAddr)
{
    i3c_master_transfer_t masterXfer = { 0 };
    memset(&masterXfer, 0, sizeof(masterXfer));
    uint8_t txBuff[1]       = { CCC_RSTDAA };
    masterXfer.slaveAddress = I3C_BROADCAST_ADDR;
    masterXfer.data         = txBuff;
    masterXfer.dataSize     = 1;
    masterXfer.direction    = kI3C_Write;
    masterXfer.busType      = kI3C_TypeI3CSdr;
    masterXfer.flags        = kI3C_TransferDefaultFlag;
    status_t result         = I3C_MasterTransferBlocking(i3c, &masterXfer);
    i3c_err_dbg_log(result);
    if (result != kStatus_Success)
    {
        return result;
    }

    memset(&masterXfer, 0, sizeof(masterXfer));
    txBuff[0]               = CCC_SETDASA;
    masterXfer.slaveAddress = I3C_BROADCAST_ADDR;
    masterXfer.data         = txBuff;
    masterXfer.dataSize     = 1;
    masterXfer.direction    = kI3C_Write;
    masterXfer.busType      = kI3C_TypeI3CSdr;
    masterXfer.flags        = kI3C_TransferNoStopFlag;
    result                  = I3C_MasterTransferBlocking(i3c, &masterXfer);
    i3c_err_dbg_log(result);
    if (result != kStatus_Success)
    {
        return result;
    }

    memset(&masterXfer, 0, sizeof(masterXfer));
    txBuff[0]               = dynamicAddr << 1U;
    masterXfer.slaveAddress = staticAddr;
    masterXfer.data         = txBuff;
    masterXfer.dataSize     = 1;
    masterXfer.direction    = kI3C_Write;
    masterXfer.busType      = kI3C_TypeI3CSdr;
    masterXfer.flags        = kI3C_TransferDefaultFlag;
    result                  = I3C_MasterTransferBlocking(i3c, &masterXfer);
    i3c_err_dbg_log(result);
    return result;
}

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

    i3c_master_config_t masterConfig;
    I3C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz.i2cBaud          = I2C_BAUDRATE;
    masterConfig.baudRate_Hz.i3cPushPullBaud  = I3C_PP_BAUDRATE;
    masterConfig.baudRate_Hz.i3cOpenDrainBaud = I3C_OD_BAUDRATE;
    masterConfig.enableOpenDrainStop          = false;
    masterConfig.enableOpenDrainHigh          = false;
    masterConfig.disableTimeout               = true;
    masterConfig.hKeep                        = kI3C_MasterPassiveSDASCL;
    masterConfig.enableMaster                 = kI3C_MasterOn;
    PRINTF("I3C_MASTER_CLOCK_FREQUENCY: %d\r\n", I3C_MASTER_CLOCK_FREQUENCY);
    I3C_MasterInit(I3C0, &masterConfig, I3C_MASTER_CLOCK_FREQUENCY);
    I3C_MasterTransferCreateHandle(I3C0, &g_i3c_m_handle, &global_master_callback, NULL);

    status_t result = i3c_set_dynamic_address(I3C0, I2C_TSC1641_ADDR, SENSOR_ADDR);
    if (result != kStatus_Success)
    {
        PRINTF("TSC1641 set dynamic address failed.\r\n");
        assert(0);
    }

    tsc1641_config_t tsc1641Config;
    tsc1641Config.write_handler      = I3C_WriteSensor;
    tsc1641Config.read_handler       = I3C_ReadSensor;
    tsc1641Config.addr               = SENSOR_ADDR;
    tsc1641Config.shunt_val          = R_SHUNT_RESISTANCE;
    tsc1641Config.mode               = TSC1641_Mode_VshloadCont;
    tsc1641Config.conversion_time    = TSC1641_Conf_CT_1024;
    tsc1641Config.reset_state        = false;
    tsc1641Config.temp_sensor_enable = true;
    result                           = tsc1641_init(&g_tsc1641_handle, &tsc1641Config);
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
