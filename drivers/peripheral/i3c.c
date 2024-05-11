/*
 * Copyright Timur Khasanshin 2024
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "i3c.h"
#include "MCXA153.h"
#include "fsl_debug_console.h"
#include "fsl_i3c.h"

#define I3C_MASTER_CLOCK_FREQUENCY CLOCK_GetI3CFClkFreq()
#define I3C_TIME_OUT_INDEX         100000000U

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

static void i3c_master_callback (I3C_Type * base, i3c_master_handle_t * handle, status_t status, void * user_data);

static volatile status_t g_completion_status;  /**< Buffer to store the I3C transfer completion status */
static volatile bool g_master_completion_flag; /**< Flag indicating that started I3C transfer is completed */
static i3c_master_handle_t g_i3c_m_handle;     /**< Handle for I3C master instance */
/**< Structure with calback functions for I3C master events */
static const i3c_master_transfer_callback_t global_master_callback = { NULL, NULL, i3c_master_callback };

/**
 * @brief Initializes the I3C master module.
 */
void i3c_init (void)
{
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
}

/**
 * @brief Writes data to a specified register of a device on the I3C bus.
 *
 * This function initiates a non-blocking write transfer to the specified device
 * at the given register address. It waits for the transfer to complete or timeout.
 *
 * @param dev_addr  The 7-bit I2C device address.
 * @param reg_addr  The register address to write to.
 * @param data      Pointer to the data to be written.
 * @param len       The number of bytes to write.
 *
 * @return status_t  Returns kStatus_Success if the transfer is successful,
 *                   kStatus_Timeout if the transfer times out, or an error code
 *                   from the I3C driver if the transfer fails.
 */
status_t i3c_write (uint8_t dev_addr, uint32_t reg_addr, uint8_t * data, size_t len)
{
    i3c_master_transfer_t master_xfer = { 0 };
    master_xfer.slaveAddress          = dev_addr;
    master_xfer.direction             = kI3C_Write;
    master_xfer.busType               = kI3C_TypeI3CSdr;
    master_xfer.subaddress            = reg_addr;
    master_xfer.subaddressSize        = 1;
    master_xfer.data                  = data;
    master_xfer.dataSize              = len;
    master_xfer.flags                 = kI3C_TransferDefaultFlag;

    g_master_completion_flag = false;
    g_completion_status      = kStatus_Success;
    status_t result          = I3C_MasterTransferNonBlocking(I3C0, &g_i3c_m_handle, &master_xfer);
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

/**
 * @brief Reads data from a specified register of a device on the I3C bus.
 *
 * This function initiates a non-blocking read transfer to the specified device
 * at the given register address. It waits for the transfer to complete or timeout.
 *
 * @param dev_addr  The 7-bit I2C device address.
 * @param reg_addr  The register address to read from.
 * @param data      Pointer to the buffer where the read data will be stored.
 * @param len       The number of bytes to read.
 *
 * @return status_t  Returns kStatus_Success if the transfer is successful,
 *                   kStatus_Timeout if the transfer times out, or an error code
 *                   from the I3C driver if the transfer fails.
 */
status_t i3c_read (uint8_t dev_addr, uint32_t reg_addr, uint8_t * data, size_t len)
{
    i3c_master_transfer_t master_xfer = { 0 };
    master_xfer.slaveAddress          = dev_addr;
    master_xfer.direction             = kI3C_Read;
    master_xfer.busType               = kI3C_TypeI3CSdr;
    master_xfer.subaddress            = reg_addr;
    master_xfer.subaddressSize        = 1;
    master_xfer.data                  = data;
    master_xfer.dataSize              = len;
    master_xfer.flags                 = kI3C_TransferDefaultFlag;

    g_master_completion_flag = false;
    g_completion_status      = kStatus_Success;
    status_t result          = I3C_MasterTransferNonBlocking(I3C0, &g_i3c_m_handle, &master_xfer);
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

/**
 * @brief Sets the dynamic address of a device on the I3C bus.
 *
 * This function performs a sequence of I3C transactions to set the dynamic
 * address of a device. It first sends a Reset Device Address (RSTDAA) command
 * to all devices on the bus, then sends a Set Device Address (SETDASA) command
 * to the device with the specified static address, and finally writes the new
 * dynamic address to the device.
 *
 * @param i3c       Pointer to the I3C peripheral instance.
 * @param static_addr  The static address of the device.
 * @param dynamic_addr The new dynamic address to be set for the device.
 *
 * @return status_t  Returns kStatus_Success if the dynamic address is set
 *                   successfully, or an error code from the I3C driver if
 *                   the operation fails.
 */
status_t i3c_set_dynamic_address (I3C_Type * i3c, uint8_t static_addr, uint8_t dynamic_addr)
{
    i3c_master_transfer_t master_xfer = { 0 };
    memset(&master_xfer, 0, sizeof(master_xfer));
    uint8_t txBuff[1]        = { CCC_RSTDAA };
    master_xfer.slaveAddress = I3C_BROADCAST_ADDR;
    master_xfer.data         = txBuff;
    master_xfer.dataSize     = 1;
    master_xfer.direction    = kI3C_Write;
    master_xfer.busType      = kI3C_TypeI3CSdr;
    master_xfer.flags        = kI3C_TransferDefaultFlag;
    status_t result          = I3C_MasterTransferBlocking(i3c, &master_xfer);
    i3c_err_dbg_log(result);
    if (result != kStatus_Success)
    {
        return result;
    }

    memset(&master_xfer, 0, sizeof(master_xfer));
    txBuff[0]                = CCC_SETDASA;
    master_xfer.slaveAddress = I3C_BROADCAST_ADDR;
    master_xfer.data         = txBuff;
    master_xfer.dataSize     = 1;
    master_xfer.direction    = kI3C_Write;
    master_xfer.busType      = kI3C_TypeI3CSdr;
    master_xfer.flags        = kI3C_TransferNoStopFlag;
    result                   = I3C_MasterTransferBlocking(i3c, &master_xfer);
    i3c_err_dbg_log(result);
    if (result != kStatus_Success)
    {
        return result;
    }

    memset(&master_xfer, 0, sizeof(master_xfer));
    txBuff[0]                = dynamic_addr << 1U;
    master_xfer.slaveAddress = static_addr;
    master_xfer.data         = txBuff;
    master_xfer.dataSize     = 1;
    master_xfer.direction    = kI3C_Write;
    master_xfer.busType      = kI3C_TypeI3CSdr;
    master_xfer.flags        = kI3C_TransferDefaultFlag;
    result                   = I3C_MasterTransferBlocking(i3c, &master_xfer);
    i3c_err_dbg_log(result);
    return result;
}

/**
 * @brief Prints an error message based on the provided I3C status code.
 *
 * This function takes an I3C status code as input and prints an error message
 * corresponding to the status code. The error messages are stored in a static
 * array and accessed using the status code as an index.
 *
 * @param err  The I3C status code to be logged.
 */
void i3c_err_dbg_log (status_t err)
{
    static const char * g_i3c_err_arr[] = {
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

    if (err != kStatus_Success)
    {
        PRINTF("I3C Error: %s.\r\n", g_i3c_err_arr[err - kStatus_I3C_Busy]);
    }
}

/**
 * @brief I3C master callback function.
 *
 * This function is called by the I3C master driver when a transfer is completed or an error occurs.
 * It sets the completion flag and stores the transfer status.
 *
 * @param base       Pointer to the I3C peripheral instance.
 * @param handle     Pointer to the I3C master handle.
 * @param status     The status of the transfer.
 * @param user_data  Pointer to user-defined data.
 */
static void i3c_master_callback (I3C_Type * base, i3c_master_handle_t * handle, status_t status, void * user_data)
{
    (void)base;
    (void)user_data;
    (void)handle;

    if (status == kStatus_Success)
    {
        g_master_completion_flag = true;
    }

    g_completion_status = status;
}
