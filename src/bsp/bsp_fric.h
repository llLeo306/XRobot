#pragma once

#include <stdint.h>

#include "bsp.h"

int8_t BSP_Fric_Start(void);
int8_t BSP_Fric_Set(float duty_cycle);
int8_t BSP_Fric_Stop(void);
