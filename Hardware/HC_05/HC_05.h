#include "usart.h"

typedef enum {
    WAITING_HEADER = 0,
    RECEIVING_DATA,
    WATING_TAIL,
} HC_05_State_t;

typedef struct {
    float Velocity_x;
    float Velocity_y;
    float Omega;
}Velocity_t;

void HC_05_Data_Handle(uint8_t RxBuffer);
void StartUart1ReceiveIT(void);