#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "param.h"

int param_flash_load(union param_value_u* active_array, bool* dirty_array, uint16_t count);
int param_flash_save(const union param_value_u* active_array, const bool* dirty_array, uint16_t count);
