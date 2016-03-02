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

struct high_speed_logger_spi_link_data high_speed_logger_spi_link_data;
struct spi_transaction high_speed_logger_spi_link_transaction;

static volatile bool_t high_speed_logger_spi_link_ready = TRUE;

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans);

void high_speed_logger_spi_link_init(void)
{
  high_speed_logger_spi_link_data.id = 0;

  high_speed_logger_spi_link_transaction.select        = SPISelectUnselect;
  high_speed_logger_spi_link_transaction.cpol          = SPICpolIdleHigh;
  high_speed_logger_spi_link_transaction.cpha          = SPICphaEdge2;
  high_speed_logger_spi_link_transaction.dss           = SPIDss8bit;
  high_speed_logger_spi_link_transaction.bitorder      = SPIMSBFirst;
  high_speed_logger_spi_link_transaction.cdiv          = SPIDiv64;
  high_speed_logger_spi_link_transaction.slave_idx     = HIGH_SPEED_LOGGER_SPI_LINK_SLAVE_NUMBER;
  high_speed_logger_spi_link_transaction.output_length = sizeof(high_speed_logger_spi_link_data);
  high_speed_logger_spi_link_transaction.output_buf    = (uint8_t *) &high_speed_logger_spi_link_data;
  high_speed_logger_spi_link_transaction.input_length  = 0;
  high_speed_logger_spi_link_transaction.input_buf     = NULL;
  high_speed_logger_spi_link_transaction.after_cb      = high_speed_logger_spi_link_trans_cb;
}

#include "state.h"
#include "modules/helicopter/throttle_curve.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "modules/adcs/adc_generic.h"

void high_speed_logger_spi_link_periodic(void)
{
  if (high_speed_logger_spi_link_ready) {
    struct Int32Quat *att_quat = stateGetNedToBodyQuat_i();
    high_speed_logger_spi_link_ready = FALSE;
    high_speed_logger_spi_link_data.gyro_p     = imu.gyro.p;
    high_speed_logger_spi_link_data.gyro_q     = imu.gyro.q;
    high_speed_logger_spi_link_data.gyro_r     = imu.gyro.r;
    high_speed_logger_spi_link_data.acc_x      = stab_att_sp_quat.qi;
    high_speed_logger_spi_link_data.acc_y      = stab_att_sp_quat.qx;
    high_speed_logger_spi_link_data.acc_z      = stab_att_sp_quat.qy;
    high_speed_logger_spi_link_data.mag_x      = att_quat->qi;
    high_speed_logger_spi_link_data.mag_y      = att_quat->qx;
    high_speed_logger_spi_link_data.mag_z      = att_quat->qy;
    high_speed_logger_spi_link_data.phi        = att_quat->qz;
    high_speed_logger_spi_link_data.theta      = adc_generic_val1;
    high_speed_logger_spi_link_data.psi        = stab_att_sp_quat.qz;
    high_speed_logger_spi_link_data.extra1     = stabilization_cmd[COMMAND_ROLL];
    high_speed_logger_spi_link_data.extra2     = stabilization_cmd[COMMAND_PITCH];
    high_speed_logger_spi_link_data.extra3     = stabilization_cmd[COMMAND_YAW];

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);
  }

  high_speed_logger_spi_link_data.id++;
}

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  high_speed_logger_spi_link_ready = TRUE;
}


