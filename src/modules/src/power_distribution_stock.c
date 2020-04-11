/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"

#include "pm.h"
#include "crtp.h"
#include "crtp_motor.h"

static bool motorSetEnable = false;

static struct {
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motorPower;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void)
{
  motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerStop()
{
  motorsSetRatio(MOTOR_M1, 0);
  motorsSetRatio(MOTOR_M2, 0);
  motorsSetRatio(MOTOR_M3, 0);
  motorsSetRatio(MOTOR_M4, 0);
}

void powerDistribution(const control_t *control)
{
  #ifdef QUAD_FORMATION_X
    int16_t r = control->roll / 2.0f;
    int16_t p = control->pitch / 2.0f;
    motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
    motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
    motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);
  #endif

  static int counter = 0;
  static bool update = false;
  static CRTPPacket pkt;
  pkt.size = sizeof(CrtpMotor);
  CrtpMotor *cm = (CrtpMotor *)pkt.data;
  // stabilizer runs at 1kHz by default, which is too fast for radio to keep up

  // update thrust, reduce frequency to 30Hz
  if (++counter%33 == 0)
  { 
    update = true;
    // report motor thrust and voltage
    pkt.port = CRTP_PORT_MOTOR;
    pkt.channel = 2;
    //cm->voltage = pmGetBatteryVoltage();
    // FIXME , DEBUG only, for seeing whether Tx can keep up
    //cm->voltage = (float)crtpGetFreeTxQueuePackets();
  }
  else
  {
    update = false;
  }



  if (motorSetEnable)
  {
    // user override motor power from cfclient
    motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
    motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
    motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
    motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
    // reduce to 16 bit
    cm->m1 = motorPowerSet.m1 >> 12;
    cm->m2 = motorPowerSet.m2 >> 12;
    cm->m3 = motorPowerSet.m3 >> 12;
    cm->m4 = motorPowerSet.m4 >> 12;
  }
  else
  {
    motorsSetRatio(MOTOR_M1, motorPower.m1);
    motorsSetRatio(MOTOR_M2, motorPower.m2);
    motorsSetRatio(MOTOR_M3, motorPower.m3);
    motorsSetRatio(MOTOR_M4, motorPower.m4);
    cm->m1 = motorPower.m1 >> 12;
    cm->m2 = motorPower.m2 >> 12;
    cm->m3 = motorPower.m3 >> 12;
    cm->m4 = motorPower.m4 >> 12;
  }

  //send packet
  if (update)
  {
    crtpSendPacket(&pkt);
    
  }

  // update voltage at a lower freq, 10Hz
  // NOTE should be ok to recycle pkt
  if (counter%100 == 0){
    pkt.port = CRTP_PORT_MOTOR;
    pkt.channel = 3;
    cm->voltage = (uint16_t)(1000*pmGetBatteryVoltage());
    // FIXME , DEBUG only, for seeing whether Tx can keep up
    //cm->voltage = (uint16_t)crtpGetFreeTxQueuePackets();
    crtpSendPacket(&pkt);
  }


}

PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(motorPowerSet)

LOG_GROUP_START(motor)
LOG_ADD(LOG_UINT32, m1, &motorPower.m1)
LOG_ADD(LOG_UINT32, m2, &motorPower.m2)
LOG_ADD(LOG_UINT32, m3, &motorPower.m3)
LOG_ADD(LOG_UINT32, m4, &motorPower.m4)
LOG_GROUP_STOP(motor)
