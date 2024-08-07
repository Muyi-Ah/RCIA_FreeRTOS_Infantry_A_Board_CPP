#include "vision_task.hpp"
#include "cmsis_os2.h"
#include "variables.hpp"

void VisionTask(void* argument) {
    for (;;) {
        vision.Send();
        osDelay(5);
    }
}