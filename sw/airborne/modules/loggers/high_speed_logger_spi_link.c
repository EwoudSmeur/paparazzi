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
#include "state.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "sensors/airspeed_ets.h"
#include "sensors/aoa_adc.h"
#include "subsystems/actuators/motor_mixing.h"

struct high_speed_logger_spi_link_data high_speed_logger_spi_link_data;
struct high_speed_logger_spi_link_data2 high_speed_logger_spi_link_data2;
struct spi_transaction high_speed_logger_spi_link_transaction;
struct spi_transaction high_speed_logger_spi_link_transaction2;

static volatile bool_t high_speed_logger_spi_link_ready = TRUE;

static void high_speed_logger_spi_link_trans_cb( struct spi_transaction *trans );

void high_speed_logger_spi_link_init(void) {
  high_speed_logger_spi_link_data.id = -2;
  high_speed_logger_spi_link_data2.id = -1;

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


void high_speed_logger_spi_link_periodic(void)
{
  if (high_speed_logger_spi_link_ready)
  {
    struct Int32Quat* att_quaternion = stateGetNedToBodyQuat_i();

    high_speed_logger_spi_link_ready = FALSE;
    high_speed_logger_spi_link_data.id = high_speed_logger_spi_link_data.id + 2;
    high_speed_logger_spi_link_data.thrust     = stabilization_cmd[COMMAND_THRUST];
//     high_speed_logger_spi_link_data.airspeed   = stabilization_cmd[COMMAND_THRUST];
//     high_speed_logger_spi_link_data.alt        = stateGetPositionNed_i()->z;
//     high_speed_logger_spi_link_data.alpha      = stabilization_cmd[COMMAND_THRUST];
//     high_speed_logger_spi_link_data.gps_x      = stateGetPositionNed_i()->x;
//     high_speed_logger_spi_link_data.ref_qi     = stab_att_ref_quat.qi;
//     high_speed_logger_spi_link_data.ref_qx     = stab_att_ref_quat.qx;
//     high_speed_logger_spi_link_data.ref_qy     = stab_att_ref_quat.qy;
//     high_speed_logger_spi_link_data.ref_qz     = stab_att_ref_quat.qz;
//     high_speed_logger_spi_link_data.att_qi     = att_quaternion->qi;
//     high_speed_logger_spi_link_data.att_qx     = att_quaternion->qx;
//     high_speed_logger_spi_link_data.att_qy     = att_quaternion->qy;
//     high_speed_logger_spi_link_data.att_qz     = att_quaternion->qz;
//     high_speed_logger_spi_link_data.gps_speedx = stateGetSpeedNed_i()->x;
//     high_speed_logger_spi_link_data.gps_speedy = stateGetSpeedNed_i()->y;

    high_speed_logger_spi_link_data.airspeed   = RATE_BFP_OF_REAL(filtered_rate.r);
    high_speed_logger_spi_link_data.alt        = RATE_BFP_OF_REAL(filtered_rate_deriv.r);
    high_speed_logger_spi_link_data.alpha      = RATE_BFP_OF_REAL(angular_accel_ref.r);
    high_speed_logger_spi_link_data.gps_x      = RATE_BFP_OF_REAL(indi_u.r);
    high_speed_logger_spi_link_data.ref_qi     = RATE_BFP_OF_REAL(indi_du.r);
    high_speed_logger_spi_link_data.ref_qx     = RATE_BFP_OF_REAL(filtered_rate.p);
    high_speed_logger_spi_link_data.ref_qy     = RATE_BFP_OF_REAL(filtered_rate_deriv.p);
    high_speed_logger_spi_link_data.ref_qz     = RATE_BFP_OF_REAL(angular_accel_ref.p);
    high_speed_logger_spi_link_data.att_qi     = RATE_BFP_OF_REAL(indi_u.p);
    high_speed_logger_spi_link_data.att_qx     = RATE_BFP_OF_REAL(indi_du.p);
    high_speed_logger_spi_link_data.att_qy     = RATE_BFP_OF_REAL(filtered_rate.q);
    high_speed_logger_spi_link_data.att_qz     = RATE_BFP_OF_REAL(filtered_rate_deriv.q);
    high_speed_logger_spi_link_data.gps_speedx = RATE_BFP_OF_REAL(angular_accel_ref.q);
    high_speed_logger_spi_link_data.gps_speedy = RATE_BFP_OF_REAL(indi_u.q);

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);

    high_speed_logger_spi_link_data2.id = high_speed_logger_spi_link_data2.id + 2;
    high_speed_logger_spi_link_data2.acc_x      = imu.accel_unscaled.x;
    high_speed_logger_spi_link_data2.acc_y      = imu.accel_unscaled.y;
    high_speed_logger_spi_link_data2.acc_z      = imu.accel_unscaled.z;
    high_speed_logger_spi_link_data2.gyro_p     = imu.gyro_unscaled.p;
    high_speed_logger_spi_link_data2.gyro_q     = imu.gyro_unscaled.q;
    high_speed_logger_spi_link_data2.gyro_r     = imu.gyro_unscaled.r;
    high_speed_logger_spi_link_data2.mag_x      = imu.mag_unscaled.x;
    high_speed_logger_spi_link_data2.mag_y      = imu.mag_unscaled.y;
    high_speed_logger_spi_link_data2.mag_z      = imu.mag_unscaled.z;
    high_speed_logger_spi_link_data2.gps_y      = stateGetPositionNed_i()->y;;
    high_speed_logger_spi_link_data2.cmd_roll   = stabilization_cmd[COMMAND_ROLL];
    high_speed_logger_spi_link_data2.cmd_pitch  = stabilization_cmd[COMMAND_PITCH];
    high_speed_logger_spi_link_data2.cmd_yaw    = stabilization_cmd[COMMAND_YAW];
    high_speed_logger_spi_link_data2.gps_speedx = stateGetSpeedNed_i()->x;
    high_speed_logger_spi_link_data2.gps_speedy = RATE_BFP_OF_REAL(indi_du.q);

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction2);
  }

}

static void high_speed_logger_spi_link_trans_cb( struct spi_transaction *trans __attribute__ ((unused)) ) {
  high_speed_logger_spi_link_ready = TRUE;
}