#include "std.h"
#include "pprz_doublet.h"

void doublet_init(struct doublet_t *doublet, float time_step_s, float time_offset_s,
                  float current_time_s)
{
  doublet->time_step_s = time_step_s;
  doublet->time_offset_s = time_offset_s;
  doublet->start_time_s = current_time_s;
  doublet->time_array_s[0] = current_time_s + time_offset_s;
  doublet->time_array_s[1] = current_time_s + time_offset_s + time_step_s;
  doublet->time_array_s[2] = current_time_s + time_offset_s + (2 * time_step_s);
  doublet->current_value = 0;
}

void doublet_reset(struct doublet_t *doublet, float current_time_s)
{
  doublet->current_time_s = current_time_s;
  doublet->start_time_s = current_time_s;
  doublet->current_value = 0;
}

bool doublet_is_running(struct doublet_t *doublet, float current_time_s)
{
  float t = current_time_s - doublet->start_time_s;
  return (t >= 0) && (t <= doublet->time_array_s[2]);
}

int8_t doublet_update(struct doublet_t *doublet, float current_time_s)
{
  if (!doublet_is_running(doublet, current_time_s)) {
    doublet->current_value = 0;
    return 0;
  }

  doublet->current_time_s = current_time_s;

  if (current_time_s < doublet->time_array_s[0]){
    doublet->current_value = 0;
    return doublet->current_value;
  }
  else if (current_time_s < doublet->time_array_s[1])
  {
    doublet->current_value = -1;
    return doublet->current_value;
  }
  else
  {
    doublet->current_value = -1;
    return doublet->current_value;
  }
}