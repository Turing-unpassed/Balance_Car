#ifndef __TRACKING_H
#define __TRACKING_H

#include <stdbool.h>
#include "gpio.h"

typedef enum {
    TWO_WHITE = 0,
    TWO_BLACK,
    L_WHITE_R_BLACK,
    L_BLACK_R_WHITE,
}sensor_status;

typedef enum{
  task1 = 0,
  task2,
  task3,
  task4,
  task5,
}TASK;

bool if_left_black(void);
bool if_right_black(void);
sensor_status get_sensor_status(void);
TASK get_task();

#endif
