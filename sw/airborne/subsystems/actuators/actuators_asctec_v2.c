/*
 * Copyright (C) 2013 The Paparazzi Team
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
 */

/** @file actuators_asctec_v2.c
 *  Actuators driver for Asctec v2 motor controllers.
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_asctec_v2.h"

#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

#define ACTUATORS_ASCTEC_V2_SLAVE_ADDR 0x02

PRINT_CONFIG_VAR(ACTUATORS_ASCTEC_V2_I2C_DEV)

struct ActuatorsAsctecV2 actuators_asctec_v2;
static uint8_t len_w;
uint8_t data_in[2] = {0,0};

void actuators_asctec_v2_init(void)
{
  actuators_asctec_v2.cmd = NONE;
  actuators_asctec_v2.cur_addr = FRONT;
  actuators_asctec_v2.new_addr = FRONT;
  //write
  actuators_asctec_v2.i2c_trans.status = I2CTransSuccess;
  actuators_asctec_v2.i2c_trans.type = I2CTransTx;
  actuators_asctec_v2.i2c_trans.slave_addr = ACTUATORS_ASCTEC_V2_SLAVE_ADDR;
  actuators_asctec_v2.i2c_trans.len_w = 5;
  actuators_asctec_v2.nb_err = 0;
  //read
  actuators_asctec_v2.i2c_read.status = I2CTransSuccess;
  actuators_asctec_v2.i2c_read.type = I2CTransRx;
  actuators_asctec_v2.i2c_read.slave_addr = ACTUATORS_ASCTEC_V2_SLAVE_ADDR;
  actuators_asctec_v2.i2c_read.len_r = 5;
}


void actuators_asctec_v2_set(void)
{
#if defined ACTUATORS_START_DELAY && ! defined SITL
  if (!actuators_delay_done) {
    if (SysTimeTimer(actuators_delay_time) < USEC_OF_SEC(ACTUATORS_START_DELAY)) {
#ifdef USE_I2C_ACTUATORS_REBOOT_HACK
      //Lisa-L with Asctech v2 motors only start after reflashing when a bus error was sensed on stm32-i2c.
      //multiple re-init solves the problem.
      i2c1_init();
#endif
      return;
    } else { actuators_delay_done = true; }
  }
#endif

  switch (actuators_asctec_v2.i2c_trans.status) {
    case I2CTransFailed:
      actuators_asctec_v2.nb_err++;
      actuators_asctec_v2.i2c_trans.status = I2CTransDone;
      break;
    case I2CTransSuccess:
    case I2CTransDone:
      actuators_asctec_v2.i2c_trans.status = I2CTransDone;
      break;
    default:
      actuators_asctec_v2.nb_err++;
      return;
  }

#ifdef KILL_MOTORS
  actuators_asctec_v2.i2c_trans.buf[0] = 0;
  actuators_asctec_v2.i2c_trans.buf[1] = 0;
  actuators_asctec_v2.i2c_trans.buf[2] = 0;
  actuators_asctec_v2.i2c_trans.buf[3] = 0;
  actuators_asctec_v2.i2c_trans.buf[4] = 0xAA;
  len_w = 5;
#else
  switch (actuators_asctec_v2.cmd) {
    case TEST:
      actuators_asctec_v2.i2c_trans.buf[0] = 251;
      actuators_asctec_v2.i2c_trans.buf[1] = actuators_asctec_v2.cur_addr;
      actuators_asctec_v2.i2c_trans.buf[2] = 0;
      actuators_asctec_v2.i2c_trans.buf[3] = 231 + actuators_asctec_v2.cur_addr;
      len_w = 4;
      break;
    case REVERSE:
      actuators_asctec_v2.i2c_trans.buf[0] = 254;
      actuators_asctec_v2.i2c_trans.buf[1] = actuators_asctec_v2.cur_addr;
      actuators_asctec_v2.i2c_trans.buf[2] = 0;
      actuators_asctec_v2.i2c_trans.buf[3] = 234 + actuators_asctec_v2.cur_addr;
      len_w = 4;
      break;
    case SET_ADDR:
      actuators_asctec_v2.i2c_trans.buf[0] = 250;
      actuators_asctec_v2.i2c_trans.buf[1] = actuators_asctec_v2.cur_addr;
      actuators_asctec_v2.i2c_trans.buf[2] = actuators_asctec_v2.new_addr;
      actuators_asctec_v2.i2c_trans.buf[3] = 230 + actuators_asctec_v2.cur_addr +
                                             actuators_asctec_v2.new_addr;
      actuators_asctec_v2.cur_addr = actuators_asctec_v2.new_addr;
      len_w = 4;
      break;
    case NONE:
      actuators_asctec_v2.i2c_trans.buf[0] = actuators_asctec_v2.cmds[SERVO_FRONT];
      actuators_asctec_v2.i2c_trans.buf[1] = actuators_asctec_v2.cmds[SERVO_BACK];
      actuators_asctec_v2.i2c_trans.buf[2] = actuators_asctec_v2.cmds[SERVO_LEFT];
      actuators_asctec_v2.i2c_trans.buf[3] = actuators_asctec_v2.cmds[SERVO_RIGHT];
      actuators_asctec_v2.i2c_trans.buf[4] = 0xAA + actuators_asctec_v2.i2c_trans.buf[0] +
                                             actuators_asctec_v2.i2c_trans.buf[1] +
                                             actuators_asctec_v2.i2c_trans.buf[2] +
                                             actuators_asctec_v2.i2c_trans.buf[3];
      len_w = 5;
      break;
    default:
      break;
  }
  actuators_asctec_v2.cmd = NONE;
#endif

  i2c_transmit(&ACTUATORS_ASCTEC_V2_I2C_DEV, &actuators_asctec_v2.i2c_trans, ACTUATORS_ASCTEC_V2_SLAVE_ADDR, len_w);

  // process previous read
  data_in[0] = actuators_asctec_v2.i2c_read.buf[0];
  /*data_in[1] = actuators_asctec_v2.i2c_read.buf[1];*/
  uint8_t motor_num = 3;
  uint8_t read_addr = ((motor_num+3)<<1)|0x01;
  data_in[1] = read_addr;
  /*i2c_receive(&ACTUATORS_ASCTEC_V2_I2C_DEV, &actuators_asctec_v2.i2c_read, read_addr, 3);*/
}

