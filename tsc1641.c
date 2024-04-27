/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "tsc1641.h"
#include <math.h>

#define U16_REV(x) __REV16(x)

#define ERR_DBG_LOG(err)                 \
    if (handle->errDebugHandler != NULL) \
    {                                    \
        handle->errDebugHandler(err);    \
    }

#define TSC1641_RShunt_c  1e-5f
#define TSC1641_ShuntV_c  2.5e-6f
#define TSC1641_LoadV_c   2.0e-3f
#define TSC1641_Power_c   2.5e-3f
#define TSC1641_Current_c 2.5e-6f
#define TSC1641_Temp_c    0.5f

status_t TSC1641_Init (tsc1641_handle_t * handle, const tsc1641_config_t * config)
{
    assert(handle != NULL);
    assert(config != NULL);
    assert(config->writeTransfer != NULL);
    assert(config->readTransfer != NULL);
    assert(config->shuntResistance != 0);

    handle->writeTransfer   = config->writeTransfer;
    handle->readTransfer    = config->readTransfer;
    handle->sensorAddress   = config->sensorAddress;
    handle->shuntResistance = config->shuntResistance;

    uint16_t reg    = (int32_t)roundf(handle->shuntResistance / TSC1641_RShunt_c);
    reg             = U16_REV(reg);
    status_t result = TSC1641_WriteReg(handle, TSC1641_RegAdd_RShunt, (uint8_t *)&reg, sizeof(reg));
    ERR_DBG_LOG(result);
    assert(result == kStatus_Success);

    reg = config->resetState ? (1U << TSC1641_Reg_Conf_RST_Offset) : 0U;
    reg |= config->mode | (config->convTime << TSC1641_Reg_Conf_CT_Offset)
         | (config->enableTempSensor ? (1U << TSC1641_Reg_Conf_T_EN_Offset) : 0U);
    reg    = U16_REV(reg);
    result = TSC1641_WriteReg(handle, TSC1641_RegAdd_Conf, (uint8_t *)&reg, sizeof(reg));
    ERR_DBG_LOG(result);
    assert(result == kStatus_Success);

    return result;
}

status_t TSC1641_WriteReg (tsc1641_handle_t * handle, uint32_t regAddress, uint8_t * regData, size_t dataSize)
{
    return handle->writeTransfer(handle->sensorAddress, regAddress, regData, dataSize);
}

status_t TSC1641_ReadReg (tsc1641_handle_t * handle, uint32_t regAddress, uint8_t * regData, size_t dataSize)
{
    return handle->readTransfer(handle->sensorAddress, regAddress, regData, dataSize);
}

status_t TSC1641_ReadAllData (tsc1641_handle_t * handle, float * voltage, float * current, float * power, float * temp)
{
    uint16_t data[4] = { 0, 0, 0, 0 };
    status_t result  = TSC1641_ReadReg(handle, TSC1641_RegAdd_LoadV, (uint8_t *)&data[0], 2);
    ERR_DBG_LOG(result);
    result = TSC1641_ReadReg(handle, TSC1641_RegAdd_Current, (uint8_t *)&data[1], 2);
    ERR_DBG_LOG(result);
    result = TSC1641_ReadReg(handle, TSC1641_RegAdd_Power, (uint8_t *)&data[2], 2);
    ERR_DBG_LOG(result);
    result = TSC1641_ReadReg(handle, TSC1641_RegAdd_Temp, (uint8_t *)&data[3], 2);
    ERR_DBG_LOG(result);
    if (result == kStatus_Success)
    {
        *voltage = (float)(int16_t)U16_REV(data[0]) * TSC1641_LoadV_c;
        *current = (float)(int16_t)U16_REV(data[1]) * TSC1641_Current_c / handle->shuntResistance;
        *power   = (float)(int16_t)U16_REV(data[2]) * TSC1641_Power_c;
        *temp    = (float)(int16_t)U16_REV(data[3]) * TSC1641_Temp_c;
    }
    return result;
}
