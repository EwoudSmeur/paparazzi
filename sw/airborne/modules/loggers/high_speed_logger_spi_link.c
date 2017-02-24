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
struct high_speed_logger_spi_link_data2 high_speed_logger_spi_link_data2;
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
  high_speed_logger_spi_link_transaction2.after_cb      = high_speed_logger_spi_link_trans_cb;
}

#include "state.h"
#include "stabilization.h"
#include "subsystems/actuators.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"

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
    struct Int32Vect3 *accel = stateGetAccelBody_i();
    struct Int32Quat *quat = stateGetNedToBodyQuat_i();

    int32_t v_int[4];
    v_int[0] = indi_v[0]*10000;
    v_int[1] = indi_v[1]*10000;
    v_int[2] = indi_v[2]*10000;
    v_int[3] = indi_v[3]*10000;

    high_speed_logger_spi_link_ready = false;
    high_speed_logger_spi_link_data.gyro_p     = rates->p;
    high_speed_logger_spi_link_data.gyro_q     = rates->q;
    high_speed_logger_spi_link_data.gyro_r     = rates->r;
    high_speed_logger_spi_link_data.acc_x      = actuators_pprz[0];
    high_speed_logger_spi_link_data.acc_y      = actuators_pprz[1];
    high_speed_logger_spi_link_data.acc_z      = actuators_pprz[2];
    high_speed_logger_spi_link_data.mag_x      = actuators_pprz[3];
    high_speed_logger_spi_link_data.mag_y      = accel->x;
    high_speed_logger_spi_link_data.mag_z      = accel->y;
    high_speed_logger_spi_link_data.phi        = accel->z;
    high_speed_logger_spi_link_data.theta      = quat->qi;
    high_speed_logger_spi_link_data.psi        = quat->qx;
    high_speed_logger_spi_link_data.extra1     = quat->qy;
    high_speed_logger_spi_link_data.extra2     = quat->qz;
    high_speed_logger_spi_link_data.extra3     = transition_theta_offset;

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);

    counter ++;
    high_speed_logger_spi_link_data2.id         = counter;
    high_speed_logger_spi_link_data2.acc_x      = stab_att_sp_quat.qi;
    high_speed_logger_spi_link_data2.acc_y      = stab_att_sp_quat.qx;
    high_speed_logger_spi_link_data2.acc_z      = stab_att_sp_quat.qy;
    high_speed_logger_spi_link_data2.gyro_p     = stab_att_sp_quat.qz;
    high_speed_logger_spi_link_data2.gyro_q     = v_int[0];
    high_speed_logger_spi_link_data2.gyro_r     = v_int[1];
    high_speed_logger_spi_link_data2.mag_x      = v_int[2];
    high_speed_logger_spi_link_data2.mag_y      = v_int[3];
    high_speed_logger_spi_link_data2.mag_z      = stateGetPositionNed_i()->x;
    high_speed_logger_spi_link_data2.gps_y      = stateGetPositionNed_i()->y;
    high_speed_logger_spi_link_data2.cmd_roll   = stateGetPositionNed_i()->z;
    high_speed_logger_spi_link_data2.cmd_pitch  = stateGetSpeedNed_i()->x;
    high_speed_logger_spi_link_data2.cmd_yaw    = stateGetSpeedNed_i()->y;
    high_speed_logger_spi_link_data2.gps_speedx = stateGetSpeedNed_i()->z;
    high_speed_logger_spi_link_data2.gps_speedy = 0;

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction2);
  }
}

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  high_speed_logger_spi_link_ready = true;
}


