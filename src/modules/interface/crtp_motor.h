// define packet structure for CRTP port MOTOR
// for reporting motor thrust and battery voltage
#ifndef CRTP_MOTOR_H
#define CRTP_MOTOR_H

#include "crtp.h"

#define CRTP_MOTOR_FAILSAFE_TIMEOUT_MS 100
#define CRTP_MOTOR_FAILSAFE_RATE 50

typedef struct _CrtpMotor
{
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
  float voltage;
} __attribute__((packed)) CrtpMotor;

void crtpMotorHandler(CRTPPacket *p);
void crtpMotorInit(void);


#endif /*CRTP_MOTOR_H*/
