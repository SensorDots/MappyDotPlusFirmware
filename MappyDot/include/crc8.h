/* Copyright (c) 2011 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Very simple 8-bit CRC function.
 */
#ifndef CRC8_H_
#define CRC8_H_
#include <stdint.h>
uint8_t Crc8(const void* data, int len);
#endif /* CRC8_H_ */