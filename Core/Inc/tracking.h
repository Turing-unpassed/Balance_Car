#include <stdbool.h>

typedef enum {
    TWO_WHITE = 0,
    TWO_BLACK,
    L_WHITE_R_BLACK,
    L_BLACK_R_WHITE,
}sensor_status;

bool if_left_black(void);
bool if_right_black(void);
sensor_status get_sensor_status(void);