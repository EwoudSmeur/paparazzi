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
struct high_speed_logger_spi_link_data high_speed_logger_spi_link_data2;
struct spi_transaction high_speed_logger_spi_link_transaction;
struct spi_transaction high_speed_logger_spi_link_transaction2;

static volatile bool high_speed_logger_spi_link_ready = true;

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans);

void high_speed_logger_spi_link_init(void)
{
  high_speed_logger_spi_link_data.id = 0;
  high_speed_logger_spi_link_data2.id = 1;

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

  high_speed_logger_spi_link_transaction2.select        = SPISelectUnselect;
  high_speed_logger_spi_link_transaction2.cpol          = SPICpolIdleHigh;
  high_speed_logger_spi_link_transaction2.cpha          = SPICphaEdge2;
  high_speed_logger_spi_link_transaction2.dss           = SPIDss8bit;
  high_speed_logger_spi_link_transaction2.bitorder      = SPIMSBFirst;
  high_speed_logger_spi_link_transaction2.cdiv          = SPIDiv64;
  high_speed_logger_spi_link_transaction2.slave_idx     = HIGH_SPEED_LOGGER_SPI_LINK_SLAVE_NUMBER;
  high_speed_logger_spi_link_transaction2.output_length = sizeof(high_speed_logger_spi_link_data2);
  high_speed_logger_spi_link_transaction2.output_buf    = (uint8_t*) &high_speed_logger_spi_link_data2;
  high_speed_logger_spi_link_transaction2.input_length  = 0;
  high_speed_logger_spi_link_transaction2.input_buf     = NULL;
high_speed_logger_spi_link_transaction2.after_cb = high_speed_logger_spi_link_trans_cb;
}

#include "subsystems/actuators.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "modules/actuators/kiss_telemetry.h"
#include "state.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"

void high_speed_logger_spi_link_periodic(void)
{
  // Static counter to identify missing samples
  static int32_t counter = 0;

  // count all periodic steps
  counter ++;

  // only send a new message if the previous was completely sent
  if (high_speed_logger_spi_link_ready) {
    // copy the counter into the SPI datablock
    high_speed_logger_spi_link_data.id = counter;

    struct Int32Rates *rates = stateGetBodyRates_i();
    struct Int32Quat *quat = stateGetNedToBodyQuat_i();

    high_speed_logger_spi_link_ready = false;
    high_speed_logger_spi_link_data.gyro_p     = rates->p;
    high_speed_logger_spi_link_data.gyro_q     = rates->q;
    high_speed_logger_spi_link_data.gyro_r     = rates->r;
    high_speed_logger_spi_link_data.act1       = indi_du[0];
    high_speed_logger_spi_link_data.act2       = indi_du[1];
    high_speed_logger_spi_link_data.act3       = indi_du[2];
    high_speed_logger_spi_link_data.act4       = indi_du[3];
    high_speed_logger_spi_link_data.v1         = indi_v[0];
    high_speed_logger_spi_link_data.v2         = indi_v[1];
    high_speed_logger_spi_link_data.v3         = indi_v[2];
    high_speed_logger_spi_link_data.v4         = indi_v[3];
    high_speed_logger_spi_link_data.quati      = quat->qi;
    high_speed_logger_spi_link_data.quatx      = quat->qx;
    high_speed_logger_spi_link_data.quaty      = quat->qy;
    high_speed_logger_spi_link_data.quatz      = quat->qz;

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);

    counter++;
    high_speed_logger_spi_link_data2.gyro_p     = rates->p;
    high_speed_logger_spi_link_data2.gyro_q     = rates->q;
    high_speed_logger_spi_link_data2.gyro_r     = rates->r;
    high_speed_logger_spi_link_data2.act1       = actuator_state_filt_vect[0];
    high_speed_logger_spi_link_data2.act2       = actuator_state_filt_vect[1];
    high_speed_logger_spi_link_data2.act3       = actuator_state_filt_vect[2];
    high_speed_logger_spi_link_data2.act4       = actuator_state_filt_vect[3];
    high_speed_logger_spi_link_data2.v1         = indi_v[0];
    high_speed_logger_spi_link_data2.v2         = indi_v[1];
    high_speed_logger_spi_link_data2.v3         = indi_v[2];
    high_speed_logger_spi_link_data2.v4         = indi_v[3];
    high_speed_logger_spi_link_data2.quati      = stab_att_sp_quat.qi;
    high_speed_logger_spi_link_data2.quatx      = stab_att_sp_quat.qx;
    high_speed_logger_spi_link_data2.quaty      = stab_att_sp_quat.qy;
    high_speed_logger_spi_link_data2.quatz      = stab_att_sp_quat.qz;

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction2);
  }
}

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  high_speed_logger_spi_link_ready = true;
}


