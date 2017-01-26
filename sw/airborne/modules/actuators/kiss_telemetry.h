/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/actuators/kiss_telemetry.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Telemetry from the kiss ESCs
 */

#ifndef KISS_TELEMETRY_H
#define KISS_TELEMETRY_H

#include "pprzlink/pprzlink_device.h"

struct kiss_telemetry_t {
  uint8_t servo_idx;              ///< Servo index communicated with
  struct link_device *dev;        ///< Device used for communication
  uint8_t buf_idx;                ///< Buffer index
  uint8_t buffer[10];             ///< Buffer with the data packet
  uint8_t crc;                    ///< The calculated CRC
  uint16_t energy[4];              ///< Accumilated energy
};

struct kiss_message_t {
  uint8_t temperature;
  uint16_t voltage;
  uint16_t current;
  uint16_t consumption;
  uint16_t rpm;
  uint8_t crc;
};

extern float rpm_feedback_log;

extern void kiss_telemetry_init(void);
extern void kiss_telemetry_periodic(void);
extern void kiss_telemetry_event(void);

#endif

