/*
 * Copyright (C) 2005-2013 The Paparazzi Team
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef HIGH_SPEED_LOGGER_SPI_LINK_H_
#define HIGH_SPEED_LOGGER_SPI_LINK_H_

#include "std.h"

extern void high_speed_logger_spi_link_init(void);
extern void high_speed_logger_spi_link_periodic(void);

#define PACKED __attribute__((__packed__))

struct PACKED high_speed_logger_spi_link_data {
  int32_t id;         // 1
  int32_t thrust;     // 2
  int32_t airspeed;
  int32_t alt;
  int32_t alpha;      // 5
  int32_t gps_x;
  int32_t ref_qi;
  int32_t ref_qx;      // 8
  int32_t ref_qy;
  int32_t ref_qz;
  int32_t att_qi;        // 11
  int32_t att_qx;
  int32_t att_qy;
  int32_t att_qz;     // 14
  int32_t gps_speedx;     // 15
  int32_t gps_speedy;     // 16
};

struct PACKED high_speed_logger_spi_link_data2 {
  int32_t id;         // 1
  int32_t acc_x;     // 2
  int32_t acc_y;
  int32_t acc_z;
  int32_t gyro_p;      // 5
  int32_t gyro_q;
  int32_t gyro_r;
  int32_t mag_x;       // 8
  int32_t mag_y;
  int32_t mag_z;
  int32_t gps_y;        // 11
  int32_t cmd_roll;
  int32_t cmd_pitch;
  int32_t cmd_yaw;     // 14
  int32_t gps_speedx;     // 15
  int32_t gps_speedy;     // 16
};

#endif /* HIGH_SPEED_LOGGER_SPI_LINK_H_ */
