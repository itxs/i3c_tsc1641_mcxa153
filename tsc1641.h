#ifndef TSC1641_H_
#define TSC1641_H_

#include "fsl_common.h"

/* Registers. */
#define TSC1641_REG_SIZE 8U

/*------------------I2C adress ------------------------------------*/
#define I2C_TSC1641_ADDR 0x40U // Value if A1 and A0 are connected to GND. Otherwise please rfer to the datasheet

/*------------------register map-----------------------------------*/
#define TSC1641_RegAdd_Conf     0x00U // configuration register
#define TSC1641_RegAdd_ShuntV   0x01U // read only. LSB=2.5µV
#define TSC1641_RegAdd_LoadV    0x02U // read only. LSB=2mV
#define TSC1641_RegAdd_Power    0x03U // read only. LSB=2.5mW
#define TSC1641_RegAdd_Current  0x04U // read only. LSB=2.5µV/Rshunt
#define TSC1641_RegAdd_Temp     0x05U // read only. LSB=0.5°C
#define TSC1641_RegAdd_MaskAl   0x06U // Mask register
#define TSC1641_RegAdd_Alert    0x07U // read only
#define TSC1641_RegAdd_RShunt   0x08U // shunt register. LSB=10µOhm
#define TSC1641_RegAdd_VshuntOV 0x09U // Alert threshold. LSB=2.5µV
#define TSC1641_RegAdd_VshuntUV 0x0AU // Alert threshold. LSB=2.5µV
#define TSC1641_RegAdd_VloadOV  0x0BU // Alert threshold. LSB=2mV
#define TSC1641_RegAdd_VloadUV  0x0CU // Alert threshold. LSB=2mV
#define TSC1641_RegAdd_PowerOL  0x0DU // Alert threshold. LSB=25mW. Works only if VLOAD>0
#define TSC1641_RegAdd_TempOL   0x0EU // Alert threshold. LSB=0.5°C
#define TSC1641_RegAdd_ManufID  0xFEU
#define TSC1641_RegAdd_DieID    0xFFU

#define TSC1641_Reg_Conf_RST_Offset  15U
#define TSC1641_Reg_Conf_T_EN_Offset 3U
#define TSC1641_Reg_Conf_CT_Offset   4U

/*! @brief Define sensor access function. */
typedef status_t (*sensor_write_transfer_func_t)(uint8_t deviceAddress,
                                                 uint32_t regAddress,
                                                 uint8_t * regData,
                                                 size_t dataSize);
typedef status_t (*sensor_read_transfer_func_t)(uint8_t deviceAddress,
                                                uint32_t regAddress,
                                                uint8_t * regData,
                                                size_t dataSize);

typedef struct _tsc1641_handle
{
    sensor_write_transfer_func_t writeTransfer;
    sensor_read_transfer_func_t readTransfer;
    uint8_t sensorAddress;
    float shuntResistance;
    void (*errDebugHandler)(status_t err);
} tsc1641_handle_t;

typedef enum
{
    TSC1641_Mode_ShutDown     = 0x00, /*mode shutdown                */
    TSC1641_Mode_Vsh_Trig     = 0x01, /*mode trigger Vshunt only     */
    TSC1641_Mode_Vload_Trig   = 0x02, /*mode trigger Vload only      */
    TSC1641_Mode_Vshload_Trig = 0x03, /*mode trigger Vshunt and Vload*/
    TSC1641_Mode_Idle         = 0x04, /*mode Idle                    */
    TSC1641_Mode_VshCont      = 0x05, /*mode continuous Vshunt only  */
    TSC1641_Mode_VloadCont    = 0x06, /*mode continuous Vload only   */
    TSC1641_Mode_VshloadCont  = 0x07  /*mode continuous Vshunt and Vload*/
} tsc1641_mode_t;

typedef enum
{
    TSC1641_Conf_CT_128   = 0x00, /*conversion time of 128µs*/
    TSC1641_Conf_CT_256   = 0x01,
    TSC1641_Conf_CT_512   = 0x02,
    TSC1641_Conf_CT_1024  = 0x03,
    TSC1641_Conf_CT_2048  = 0x04,
    TSC1641_Conf_CT_4096  = 0x05,
    TSC1641_Conf_CT_8192  = 0x06,
    TSC1641_Conf_CT_16384 = 0x07,
    TSC1641_Conf_CT_32768 = 0x08, /*conversion time of 32ms*/
} tsc1641_conv_time_t;

typedef struct _tsc1641_config
{
    float shuntResistance;
    sensor_write_transfer_func_t writeTransfer;
    sensor_read_transfer_func_t readTransfer;
    tsc1641_conv_time_t convTime;
    tsc1641_mode_t mode;
    uint8_t sensorAddress;
    bool resetState;
    bool enableTempSensor;
} tsc1641_config_t;

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Create handle for TSC1641, reset the sensor per user configuration.
 *
 * @param handle The pointer to #tsc1641_handle_t.
 * @param config The pointer to #tsc1641_config_t.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t TSC1641_Init (tsc1641_handle_t * handle, const tsc1641_config_t * config);

/*!
 * @brief Write Register with register data buffer.
 *
 * @param handle The pointer to #tsc1641_handle_t.
 * @param regAddress register address to write.
 * @param regData The pointer to data buffer to be write to the reg.
 * @param dataSize Size of the regData.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t TSC1641_WriteReg (tsc1641_handle_t * handle, uint32_t regAddress, uint8_t * regData, size_t dataSize);

/*!
 * @brief Read Register to speficied data buffer.
 *
 * @param handle The pointer to #tsc1641_handle_t.
 * @param regAddress register address to read.
 * @param regData The pointer to data buffer to store the read out data.
 * @param dataSize Size of the regData to be read.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t TSC1641_ReadReg (tsc1641_handle_t * handle, uint32_t regAddress, uint8_t * regData, size_t dataSize);

/*!
 * @brief Read all data from sensor.
 *
 * @param handle The pointer to #tsc1641_handle_t.
 * @param voltage The pointer to voltage data.
 * @param current The pointer to current data.
 * @param power The pointer to power data.
 * @param temp The pointer to temperature data.
 *
 * @return kStatus_Success if success or kStatus_Fail if error.
 */
status_t TSC1641_ReadAllData (tsc1641_handle_t * handle, float * voltage, float * current, float * power, float * temp);
#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* TSC1641_H_ */
