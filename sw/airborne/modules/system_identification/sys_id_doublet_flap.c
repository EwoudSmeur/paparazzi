#include "std.h"

#include "sys_id_doublet.h"
#include "pprz_doublet.h"

#include "subsystems/datalink/telemetry.h"
#include "generated/airframe.h"
#include "mcu_periph/sys_time.h"

#ifndef DOUBLET_ENABLED
#define DOUBLET_ENABLED TRUE
#endif

static struct doublet_t doublet
uint8_t doublet_active = false;
pprz_t doublet_amplitude = 0;
float doublet_time_step_s = 2;
float doublet_time_offset_s = 1;

static void set_current_doublet_value(void)
{
    if (doublet_active) {
        current_doublet_value = doublet_amplitude * doublet.current_value;
    }
    else
    {
        current_doublet_value = 0;
    }
    
}

static void start_doublet(void)
{
    doublet_reset(&doublet, get_sys_time_float());
    doublet_active = true;
    set_current_doublet_value();
}

static void stop_doublet(void)
{
    doublet_reset(&doublet, get_sys_time_float());
    doublet_active = false;
    set_current_doublet_value();
}

void sys_id_doublet_activate_handler(uint8_t activate)
{
  doublet_active = activate;
  if (doublet_active) {
    doublet_init(&doublet, time_step_s, time_offset_s, current_time_s)
    start_doublet();
  } else {
    stop_doublet();
  }
}

void sys_id_chirp_init(void)
{
    doublet_init(&doublet, time_step_s, time_offset_s, current_time_s)
    set_current_doublet_value()
}

void sys_id_doublet_periodic(void)
{
#if DOUBLET_ENABLED

  if (doublet_active) {
    if (!doublet_is_running(&doublet, get_sys_time_float())) {
      stop_doublet();
    } else {
      doublet_update(&doublet, get_sys_time_float());
      set_current_doublet_value();
    }
  }

#endif
}