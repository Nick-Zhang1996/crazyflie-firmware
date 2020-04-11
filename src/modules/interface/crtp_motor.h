// define packet structure for CRTP port MOTOR
// for reporting motor thrust and battery voltage
#ifndef CRTP_MOTOR_H
#define CRTP_MOTOR_H
typedef union _CrtpMotor
{
  struct {
      // 2 Bytes
      uint8_t m1 : 4;
      uint8_t m2 : 4;
      uint8_t m3 : 4;
      uint8_t m4 : 4;
  };
  uint16_t voltage; // voltage * 1000
} __attribute__((packed)) CrtpMotor;


#endif /*CRTP_MOTOR_H*/
