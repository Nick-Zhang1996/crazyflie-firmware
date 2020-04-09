// handle direct motor command
#include "crtp_motor.h"
#include "power_distribution.h"
#include "stabilizer_types.h"
#include "FreeRTOS.h"
#include "static_mem.h"

static bool isInit = false;
static uint32_t last_update_timestamp = 0;

STATIC_MEM_TASK_ALLOC(crtpMotorFailsafeTask, CRTP_MOTOR_FAILSAFE_TASK_STACKSIZE);

void crtpMotorInit(void)
{
  if (isInit){
    return;
  }

  crtpRegisterPortCB(CRTP_PORT_MOTOR,crtpMotorHandler);

  STATIC_MEM_TASK_CREATE(crtpMotorFailsafeTask, crtpMotorFailsafeTask, CRTP_MOTOR_FAILSAFE_TASK_STACKSIZE, NULL, CRTP_MOTOR_FAILSAFE_TASK_PRI);

  isInit = true;

}

void crtpMotorHandler(CRTPPacket *p)
{
  // channel 1 : command
  if (p->channel == 1){
    last_update_timestamp = T2M(xTaskGetTickCount());
    const CrtpMotor* values = p->data;
    directMotor(values);
  }
}


static void crtpMotorFailsafeTask(void* param)
{
  while (1){
    uint32_t lastWakeTime;

    lastWakeTime = xTaskGetTickCount();

    uint32_t now_ms = T2M(xTaskGetTickCount());
    if (now_ms - last_update_timestamp > CRTP_MOTOR_FAILSAFE_TIMEOUT_MS){
      powerStop();
    }

    vTaskDelayUntil(&lastWakeTime,F2T(CRTP_MOTOR_FAILSAFE_RATE));
  }
}



