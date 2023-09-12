//
// Created by ura on 6/11/23.
//

#ifndef TOUCHMATRIX_PICO_COMMON_H
#define TOUCHMATRIX_PICO_COMMON_H

// operation mode (0 or 4)
#define OP_MODES    0

// util
#define MHZ 1000000

// IR LED Mode
#define LED_NORTH       1
#define LED_WEST        2
#define LED_EAST        4
#define LED_SOUTH       8

// switch config by board-version
#if defined(TM_2)
#include "tm2_config.h"
#elif defined(TM_3_DISCOVERY)
#include "tm3dis_config.h"
#elif defined(TM_4)
#include "tm4_config.h"
#endif

#endif //TOUCHMATRIX_PICO_COMMON_H
