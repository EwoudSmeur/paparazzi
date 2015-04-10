/*
 * Copyright (C) 2014 Felix Ruess <felix.ruess@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file test_actuators_pwm.c
 *
 * Simple test prog for PWM actuators.
 * Directly control actuators_pwm_values via settings.
 */


#define DATALINK_C
#define PERIODIC_C_MAIN

#include "generated/airframe.h"
#include "generated/settings.h"
#include "generated/periodic_telemetry.h"

#include "subsystems/datalink/datalink.h"
#include "subsystems/datalink/downlink.h"

#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/adc.h"
#include "led.h"
#include BOARD_CONFIG

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/f1/nvic.h>

#include "subsystems/actuators/actuators_pwm.h"

#include "modules/loggers/high_speed_logger_spi_link.h"

static struct adc_buf adc0_buf;
static struct adc_buf adc1_buf;


static inline void main_init( void );
static inline void main_periodic( void );
static inline void main_event(void);

uint32_t rpm_counter = 0;
uint32_t time = 0;
bool_t start_sequence = 0;
uint32_t rpm = 0;

void init_rpm_counter(void) {
rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN);
    gpio_set_mode(GPIOC, GPIO_MODE_INPUT,
            GPIO_CNF_INPUT_FLOAT, GPIO5);

    exti_select_source(EXTI5, GPIOC);
    exti_set_trigger(EXTI5, EXTI_TRIGGER_FALLING);
    exti_enable_request(EXTI5);

    nvic_set_priority(NVIC_EXTI9_5_IRQ, 0x0f);
    nvic_enable_irq(NVIC_EXTI9_5_IRQ);
}

void exti9_5_isr(void) {
  exti_reset_request(EXTI5);
  rpm_counter++;
  rpm = (get_sys_time_usec() - time);
  time = get_sys_time_usec();
}


int main(void) {

  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic();
    main_event();
  };
  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  ActuatorsPwmInit();
  adc_init();
  adc_buf_channel(ADC_1, &adc0_buf, 16);
  adc_buf_channel(ADC_3, &adc1_buf, 16);

  init_rpm_counter();

  high_speed_logger_spi_link_init();
  mcu_int_enable();

}
uint8_t id = 42;
bool_t time_set = false;
uint32_t timestart = 0;
uint32_t timediff = 0;

static inline void main_periodic( void ) {
  if(start_sequence == 1) {
    if(!time_set) {
      timestart = get_sys_time_usec();
      time_set = true;
    }
    timediff = get_sys_time_usec() - timestart;

    if(timediff<1000000){
      actuators_pwm_values[0] = 1000;
    }
    else if(timediff<2000000){
      actuators_pwm_values[0] = 1100;
    }
    else if(timediff<3000000){
      actuators_pwm_values[0] = 1200;
    }
    else if(timediff<4000000){
      actuators_pwm_values[0] = 1300;
    }
    else if(timediff<5000000){
      actuators_pwm_values[0] = 1400;
    }
    else if(timediff<6000000){
      actuators_pwm_values[0] = 1500;
    }
    else if(timediff<7000000){
      actuators_pwm_values[0] = 1400;
    }
    else if(timediff<8000000){
      actuators_pwm_values[0] = 1300;
    }
    else if(timediff<9000000){
      actuators_pwm_values[0] = 1200;
    }
    else if(timediff<10000000){
      actuators_pwm_values[0] = 1100;
    }
    else {
      actuators_pwm_values[0] = 1000;
      timediff =0;
      timestart = 0;
      time_set = false;
      start_sequence = false;
    }
  }

  ActuatorsPwmCommit();

  LED_PERIODIC();
  RunOnceEvery(100, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice,  16, MD5SUM);});

  uint16_t values[2];
  values[0] = adc0_buf.sum/16;
  values[1] = adc1_buf.sum/16;
  RunOnceEvery(50, {DOWNLINK_SEND_ADC(DefaultChannel, DefaultDevice, &id, 2, values);});

   RunOnceEvery(50, {
//      rpm_counter = rpm_counter *1000000/ 12 *3 / (get_sys_time_usec() - time)/2;
//      time = get_sys_time_usec();
     DOWNLINK_SEND_RPM(DefaultChannel, DefaultDevice, &rpm);
//      rpm_counter = 0;
  });

    high_speed_logger_spi_link_periodic();
//     rpm_counter = 0;

}

static inline void main_event(void) {
  DatalinkEvent();
}



#define IdOfMsg(x) (x[1])

void dl_parse_msg( void ) {
  id=2;
  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {
    case  DL_PING:
      {
        DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
      }
      break;

    case DL_SET_ACTUATOR:
      {
        uint8_t servo_no = DL_SET_ACTUATOR_no(dl_buffer);
        uint16_t servo_value = DL_SET_ACTUATOR_value(dl_buffer);
        LED_TOGGLE(2);
        if (servo_no < ACTUATORS_PWM_NB) {
          ActuatorPwmSet(servo_no, servo_value);
        }
      }
      break;

    case DL_SETTING:
      {
        if (DL_SETTING_ac_id(dl_buffer) != AC_ID) break;
        uint8_t i = DL_SETTING_index(dl_buffer);
        float var = DL_SETTING_value(dl_buffer);
        DlSetting(i, var);
        DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
      }
      break;

    case DL_GET_SETTING :
      {
        id = 10;
        if (DL_GET_SETTING_ac_id(dl_buffer) != AC_ID) break;
        uint8_t i = DL_GET_SETTING_index(dl_buffer);
        float val = settings_get_value(i);
        DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &val);
      }
      break;

    default:
      break;
  }
}
