#ifndef PPRZ_DOUBLET_H
#define PPRZ_DOUBLET_H

#include "std.h"

struct doublet_t
{
  /* data */
  float time_step_s;
  float time_offset_s;
  float start_time_s;
  float time_array_s[3];
  int8_t current_value;
  float current_time_s;
};

void doublet_init(struct doublet_t *doublet, float time_step_s, float time_offset_s,
                  float current_time_s);

void doublet_reset(struct doublet_t *doublet, float current_time_s);

bool doublet_is_running(struct doublet_t *doublet, float current_time_s);

int8_t doublet_update(struct doublet_t *doublet, float current_time_s)

#endif