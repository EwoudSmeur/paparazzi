#ifndef SYS_ID_DOUBLET_H
#define SYS_ID_DOUBLET_H

#include "paparazzi.h"

extern uint8_t doublet_active;
extern pprz_t doublet_amplitude;
extern float doublet_time_step_s;
extern float doublet_time_offset_s;

extern void sys_id_doublet_init(void);

extern void sys_id_doublet_periodic(void)

extern void sys_id_doublet_activate_handler(uint8_t activate)

#endif