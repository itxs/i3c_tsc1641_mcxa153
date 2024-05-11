/*
 * Copyright Timur Khasanshin 2024
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef I3C_H
#define I3C_H

#include "fsl_common.h"
#include <stdint.h>

void i3c_init (void);
void i3c_err_dbg_log (status_t err);
status_t i3c_write (uint8_t dev_addr, uint32_t reg_addr, uint8_t * data, size_t len);
status_t i3c_read (uint8_t dev_addr, uint32_t reg_addr, uint8_t * data, size_t len);
status_t i3c_set_dynamic_address (I3C_Type * i3c, uint8_t static_addr, uint8_t dynamic_addr);

#endif // I3C_H