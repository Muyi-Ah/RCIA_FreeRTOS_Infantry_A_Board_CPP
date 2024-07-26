#include "init.hpp"
#include "can.hpp"
#include "uart.hpp"
#include "tim.h"

void init() {
    UartInit();
    CanInit();
    HAL_TIM_Base_Start_IT(&htim7);
}
