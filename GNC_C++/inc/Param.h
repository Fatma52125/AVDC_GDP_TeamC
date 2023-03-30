#pragma once
#include <stdint.h>

void save_read_param_float(char name[], uint16_t index, float parameters[],uint16_t num, uint8_t type);

void save_read_param_int16(char name[], uint16_t index, int16_t parameters[],uint16_t num, uint8_t type);

void save_read_param_uint16(char name[], uint16_t index, uint16_t parameters[],uint16_t num, uint8_t type);