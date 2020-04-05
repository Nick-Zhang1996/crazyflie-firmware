// define packet structure for CRTP port MOTOR
// for reporting motor thrust and battery voltage
#ifndef CRTP_MOTOR_H
#define CRTP_MOTOR_H
typedef struct _CrtpMotor
{
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
  float voltage;
} __attribute__((packed)) CrtpMotor;


#endif /*CRTP_MOTOR_H*/
