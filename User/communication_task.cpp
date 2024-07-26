#include "communication_task.hpp"
#include "variables.hpp"
#include "cmsis_os2.h"

void CommunicationTask(void* argument) {
    for (;;) {
        comm.Send();
        osDelay(2);
    }
}