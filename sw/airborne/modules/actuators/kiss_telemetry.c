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
 * @file "modules/actuators/kiss_telemetry.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Telemetry from the kiss ESCs
 */

#include "modules/actuators/kiss_telemetry.h"
#include "subsystems/actuators/actuators_pwm.h"
#include "mcu_periph/uart.h"
#include "led.h"
#include <libopencm3/stm32/timer.h>
#include "subsystems/datalink/telemetry.h"
#include "subsystems/electrical.h"

static struct kiss_telemetry_t kiss_telemetry;
static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);

float rpm_feedback_log = 0.0;

void kiss_telemetry_init(void) {
  kiss_telemetry.dev = &((KISS_TELEMETRY_DEV).device);
  kiss_telemetry.servo_idx = 0;
  kiss_telemetry.buf_idx = 0;
}

void kiss_telemetry_periodic(void) {
  uint32_t cnt;
  /* Check if we are allowed to send the pulse */
#if PWM_USE_TIM1
  cnt = timer_get_counter(TIM1);
  if(cnt <= 2400) {
    return;
  }
#endif
#if PWM_USE_TIM2
  cnt = timer_get_counter(TIM2);
  if(cnt <= 2400) {
    return;
  }
#endif
#if PWM_USE_TIM3
  cnt = timer_get_counter(TIM3);
  if(cnt <= 2400) {
    return;
  }
#endif
#if PWM_USE_TIM4
  cnt = timer_get_counter(TIM4);
  if(cnt <= 2400) {
    return;
  }
#endif
#if PWM_USE_TIM5
  cnt = timer_get_counter(TIM5);
  if(cnt <= 2400) {
    return;
  }
#endif
#if PWM_USE_TIM8
  cnt = timer_get_counter(TIM8);
  if(cnt <= 2400) {
    return;
  }
#endif
#if PWM_USE_TIM9
  cnt = timer_get_counter(TIM9);
  if(cnt <= 2400) {
    return;
  }
#endif
#if PWM_USE_TIM12
  cnt = timer_get_counter(TIM12);
  if(cnt <= 2400) {
    return;
  }
#endif

  ActuatorPwmSet(kiss_telemetry.servo_idx, 840);
  ActuatorsPwmCommit();
  /*kiss_telemetry.servo_idx = (kiss_telemetry.servo_idx + 1) % 4;*/
}

void kiss_telemetry_event(void) {
  if (kiss_telemetry.dev->char_available(kiss_telemetry.dev->periph)) {
    while (kiss_telemetry.dev->char_available(kiss_telemetry.dev->periph)) {
      uint8_t c = kiss_telemetry.dev->get_byte(kiss_telemetry.dev->periph);
      kiss_telemetry.buffer[kiss_telemetry.buf_idx] = c;

      /* Update CRC if needed */
      if(kiss_telemetry.buf_idx < 9)
        kiss_telemetry.crc = update_crc8(c, kiss_telemetry.crc);
      kiss_telemetry.buf_idx++;

      /* We received a full telemetry message */
      if(kiss_telemetry.buf_idx >= 10) {
        // If the CRC matches parse the message
        if(kiss_telemetry.crc == kiss_telemetry.buffer[9]) {
          //struct kiss_message_t *msg = (struct kiss_message_t *)&kiss_telemetry.buffer[0];
          float amps = ((kiss_telemetry.buffer[3] << 8) + kiss_telemetry.buffer[4]) / 100.0;
          float bat_volts = electrical.vsupply / 10.0;
          float power = ((kiss_telemetry.buffer[5] << 8) + kiss_telemetry.buffer[6]) / 1000.0;
          float rpm = ((kiss_telemetry.buffer[7] << 8) + kiss_telemetry.buffer[8]) * 100 / 7.0;
          kiss_telemetry.energy[kiss_telemetry.servo_idx] = ((kiss_telemetry.buffer[5] << 8) + kiss_telemetry.buffer[6]);
          float motor_volts = ((kiss_telemetry.buffer[1] << 8) + kiss_telemetry.buffer[2]) / 100.0;
          float energy = (kiss_telemetry.energy[0] + kiss_telemetry.energy[1] + kiss_telemetry.energy[2] + kiss_telemetry.energy[3]) / 1000.0;

          RunOnceEvery(PERIODIC_FREQUENCY,
            pprz_msg_send_ESC(&(DefaultChannel).trans_tx, &(DefaultDevice).device, AC_ID,
              &amps, &bat_volts, &power, &rpm, &motor_volts, &energy, &kiss_telemetry.servo_idx);
          );
          rpm_feedback_log = rpm;
        }
        kiss_telemetry.buf_idx = 0;
        kiss_telemetry.crc = 0;
      }
    }
  }
}

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}
