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

#include "high_speed_logger_spi_link.h"

#include "subsystems/imu.h"
#include "mcu_periph/spi.h"
#include "subsystems/actuators/motor_mixing.h"
#include "state.h"

struct high_speed_logger_spi_link_data high_speed_logger_spi_link_data;
struct spi_transaction high_speed_logger_spi_link_transaction;

static volatile bool_t high_speed_logger_spi_link_ready = TRUE;

static void high_speed_logger_spi_link_trans_cb( struct spi_transaction *trans );

void high_speed_logger_spi_link_init(void) {
  high_speed_logger_spi_link_data.id = 0;

  high_speed_logger_spi_link_transaction.select        = SPISelectUnselect;
  high_speed_logger_spi_link_transaction.cpol          = SPICpolIdleHigh;
  high_speed_logger_spi_link_transaction.cpha          = SPICphaEdge2;
  high_speed_logger_spi_link_transaction.dss           = SPIDss8bit;
  high_speed_logger_spi_link_transaction.bitorder      = SPIMSBFirst;
  high_speed_logger_spi_link_transaction.cdiv          = SPIDiv64;
  high_speed_logger_spi_link_transaction.slave_idx     = HIGH_SPEED_LOGGER_SPI_LINK_SLAVE_NUMBER;
  high_speed_logger_spi_link_transaction.output_length = sizeof(high_speed_logger_spi_link_data);
  high_speed_logger_spi_link_transaction.output_buf    = (uint8_t*) &high_speed_logger_spi_link_data;
  high_speed_logger_spi_link_transaction.input_length  = 0;
  high_speed_logger_spi_link_transaction.input_buf     = NULL;
  high_speed_logger_spi_link_transaction.after_cb      = high_speed_logger_spi_link_trans_cb;
}


void high_speed_logger_spi_link_periodic(void)
{
  if (high_speed_logger_spi_link_ready)
  {
    struct FloatRates* body_rates = stateGetBodyRates_f();
    high_speed_logger_spi_link_ready = FALSE;
    high_speed_logger_spi_link_data.gyro_p     = body_rates->p;
    high_speed_logger_spi_link_data.gyro_q     = body_rates->q;
    high_speed_logger_spi_link_data.gyro_r     = body_rates->r;
    high_speed_logger_spi_link_data.acc_x      = motor_mixing.commands[0];
    high_speed_logger_spi_link_data.acc_y      = motor_mixing.commands[1];
    high_speed_logger_spi_link_data.acc_z      = motor_mixing.commands[2];
    high_speed_logger_spi_link_data.mag_x      = motor_mixing.commands[3];

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);
  }

  high_speed_logger_spi_link_data.id++;
}

static void high_speed_logger_spi_link_trans_cb( struct spi_transaction *trans __attribute__ ((unused)) ) {
  high_speed_logger_spi_link_ready = TRUE;
}


