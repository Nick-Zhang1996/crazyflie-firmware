// handle direct motor command
#include "FreeRTOS.h"
#include "crtp_motor.h"
#include "power_distribution.h"
#include "stabilizer_types.h"
#include "static_mem.h"
#include "estimator.h"
#include "controller.h"

#include "task.h"
#include "console.h"

static bool isInit = false;
// TODO use emergencyTimeStopTimeout for failsafe
static uint32_t last_update_timestamp = 0;

static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

#define PROPTEST_NBR_OF_VARIANCE_VALUES   100
static bool startPropTest = false;

uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static StateEstimatorType estimatorType;
static ControllerType controllerType;


// angular velocity - milliradians / sec
static int16_t rateRoll;
static int16_t ratePitch;
static int16_t rateYaw;

static struct {
  uint16_t m1;
  uint16_t m2;
  uint16_t m3;
  uint16_t m4;
} motorThrust;

typedef enum { configureAcc, measureNoiseFloor, measureProp, testBattery, restartBatTest, evaluateResult, testDone } TestState;
#ifdef RUN_PROP_TEST_AT_STARTUP
  static TestState testState = configureAcc;
#else
  static TestState testState = testDone;
#endif


static void crtpMotorTask(void* param);
static void testProps(sensorData_t *sensors);

STATIC_MEM_TASK_ALLOC(crtpMotorTask, CRTP_MOTOR_TASK_STACKSIZE);

void crtpMotorInit(StateEstimatorType estimator)
{
  if (isInit){
    return;
  }
  //consolePuts("crtpMotorInit... \n");
  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();

  estimatorType = getStateEstimator();
  controllerType = getControllerType();

  crtpRegisterPortCB(CRTP_PORT_MOTOR,crtpMotorHandler);

  STATIC_MEM_TASK_CREATE(crtpMotorTask, crtpMotorTask, CRTP_MOTOR_TASK_NAME, NULL, CRTP_MOTOR_TASK_PRI);

  isInit = true;
  //consolePuts("crtpMotorInit...success \n");

}

void crtpMotorHandler(CRTPPacket *p)
{
  //consolePuts("crtpMotorHandler called\n");
  // channel 1 : command
  if (p->channel == 1){
    last_update_timestamp = T2M(xTaskGetTickCount());
    const CrtpMotor* values = (CrtpMotor*)p->data;
    //consolePrintf("motor 1 = %u\n",values->m1);
    motorThrust.m1 = values->m1;
    motorThrust.m2 = values->m2;
    motorThrust.m3 = values->m3;
    motorThrust.m4 = values->m4;
    directMotor(values);

    // reverse calculate control for use with state estimator
    control.thrust = m1/4 + m2/4 + m3/4 + m4/4;
    control.roll = m3/2 + m4/2 - m1/2 - m2/2;
    control.pitch = m1/2 + m4/2 - m2/2 - m3/2;
    control.yaw = m1/4 + m3/4 - m2/4 - m4/4;
  }
}


static void crtpMotorTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_CRTP_MOTOR_ID_NBR);

  //Wait for the system to be fully started
  systemWaitStart();

  DEBUG_PRINT("Wait for sensor calibration...\n");

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount ();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  DEBUG_PRINT("Ready to fly.\n");
  while (1){
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();

    if (startPropTest != false) {
      // TODO: What happens with estimator when we run tests after startup?
      testState = configureAcc;
      startPropTest = false;
    }

    if (testState != testDone) {
      sensorsAcquire(&sensorData, tick);
      //testProps(&sensorData);
    } else {
      // allow to update estimator dynamically
      if (getStateEstimator() != estimatorType) {
        stateEstimatorSwitchTo(estimatorType);
        estimatorType = getStateEstimator();
      }
      // allow to update controller dynamically
      if (getControllerType() != controllerType) {
        controllerInit(controllerType);
        controllerType = getControllerType();
      }

      stateEstimator(&state, &sensorData, &control, tick);

      float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
      rateRoll = sensorData.gyro.x * deg2millirad;
      ratePitch = sensorData.gyro.y * deg2millirad;
      rateYaw = sensorData.gyro.z * deg2millirad;


      // failsafe
      uint32_t now_ms = T2M(xTaskGetTickCount());
      if (now_ms - last_update_timestamp > CRTP_MOTOR_FAILSAFE_TIMEOUT_MS){
        powerStop();
      }
    }

  }
}

static void testProps(sensorData_t *sensors)
{
  static uint32_t i = 0;
  static float accX[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accY[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accZ[PROPTEST_NBR_OF_VARIANCE_VALUES];
  static float accVarXnf;
  static float accVarYnf;
  static float accVarZnf;
  static int motorToTest = 0;
  static uint8_t nrFailedTests = 0;
  static float idleVoltage;
  static float minSingleLoadedVoltage[NBR_OF_MOTORS];
  static float minLoadedVoltage;

  if (testState == configureAcc)
  {
    motorPass = 0;
    sensorsSetAccMode(ACC_MODE_PROPTEST);
    testState = measureNoiseFloor;
    minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    minSingleLoadedVoltage[MOTOR_M1] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M2] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M3] = minLoadedVoltage;
    minSingleLoadedVoltage[MOTOR_M4] = minLoadedVoltage;
  }
  if (testState == measureNoiseFloor)
  {
    accX[i] = sensors->acc.x;
    accY[i] = sensors->acc.y;
    accZ[i] = sensors->acc.z;

    if (++i >= PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      i = 0;
      accVarXnf = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarYnf = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZnf = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINT("Acc noise floor variance X+Y:%f, (Z:%f)\n",
                  (double)accVarXnf + (double)accVarYnf, (double)accVarZnf);
      testState = measureProp;
    }

  }
  else if (testState == measureProp)
  {
    if (i < PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accX[i] = sensors->acc.x;
      accY[i] = sensors->acc.y;
      accZ[i] = sensors->acc.z;
      if (pmGetBatteryVoltage() < minSingleLoadedVoltage[motorToTest])
      {
        minSingleLoadedVoltage[motorToTest] = pmGetBatteryVoltage();
      }
    }
    i++;

    if (i == 1)
    {
      motorsSetRatio(motorToTest, 0xFFFF);
    }
    else if (i == 50)
    {
      motorsSetRatio(motorToTest, 0);
    }
    else if (i == PROPTEST_NBR_OF_VARIANCE_VALUES)
    {
      accVarX[motorToTest] = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarY[motorToTest] = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
      accVarZ[motorToTest] = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
      DEBUG_PRINT("Motor M%d variance X+Y:%f (Z:%f)\n",
                   motorToTest+1, (double)accVarX[motorToTest] + (double)accVarY[motorToTest],
                   (double)accVarZ[motorToTest]);
    }
    else if (i >= 1000)
    {
      i = 0;
      motorToTest++;
      if (motorToTest >= NBR_OF_MOTORS)
      {
        i = 0;
        motorToTest = 0;
        testState = evaluateResult;
        sensorsSetAccMode(ACC_MODE_FLIGHT);
      }
    }
  }
  else if (testState == testBattery)
  {
    if (i == 0)
    {
      minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
    }
    if (i == 1)
    {
      motorsSetRatio(MOTOR_M1, 0xFFFF);
      motorsSetRatio(MOTOR_M2, 0xFFFF);
      motorsSetRatio(MOTOR_M3, 0xFFFF);
      motorsSetRatio(MOTOR_M4, 0xFFFF);
    }
    else if (i < 50)
    {
      if (pmGetBatteryVoltage() < minLoadedVoltage)
        minLoadedVoltage = pmGetBatteryVoltage();
    }
    else if (i == 50)
    {
      motorsSetRatio(MOTOR_M1, 0);
      motorsSetRatio(MOTOR_M2, 0);
      motorsSetRatio(MOTOR_M3, 0);
      motorsSetRatio(MOTOR_M4, 0);
//      DEBUG_PRINT("IdleV: %f, minV: %f, M1V: %f, M2V: %f, M3V: %f, M4V: %f\n", (double)idleVoltage,
//                  (double)minLoadedVoltage,
//                  (double)minSingleLoadedVoltage[MOTOR_M1],
//                  (double)minSingleLoadedVoltage[MOTOR_M2],
//                  (double)minSingleLoadedVoltage[MOTOR_M3],
//                  (double)minSingleLoadedVoltage[MOTOR_M4]);
      DEBUG_PRINT("%f %f %f %f %f %f\n", (double)idleVoltage,
                  (double)(idleVoltage - minLoadedVoltage),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M1]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M2]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M3]),
                  (double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M4]));
      testState = restartBatTest;
      i = 0;
    }
    i++;
  }
  else if (testState == restartBatTest)
  {
    if (i++ > 2000)
    {
      testState = configureAcc;
      i = 0;
    }
  }
  else if (testState == evaluateResult)
  {
    for (int m = 0; m < NBR_OF_MOTORS; m++)
    {
      if (!evaluateTest(0, PROPELLER_BALANCE_TEST_THRESHOLD,  accVarX[m] + accVarY[m], m))
      {
        nrFailedTests++;
        for (int j = 0; j < 3; j++)
        {
          motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
          vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
          motorsBeep(m, false, 0, 0);
          vTaskDelay(M2T(100));
        }
      }
    }
#ifdef PLAY_STARTUP_MELODY_ON_MOTORS
    if (nrFailedTests == 0)
    {
      for (int m = 0; m < NBR_OF_MOTORS; m++)
      {
        motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
        vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
        motorsBeep(m, false, 0, 0);
        vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
      }
    }
#endif
    motorTestCount++;
    testState = testDone;
  }
}
LOG_GROUP_START(custom)
// in rad
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)

// angular velocity - milliradians / sec
LOG_ADD(LOG_INT16, rateRoll, &rateRoll)   
LOG_ADD(LOG_INT16, ratePitch, &ratePitch)
LOG_ADD(LOG_INT16, rateYaw, &rateYaw)

LOG_ADD(LOG_UIN16,m1, &motorThrust.m1)
LOG_ADD(LOG_UIN16,m2, &motorThrust.m2)
LOG_ADD(LOG_UIN16,m3, &motorThrust.m3)
LOG_ADD(LOG_UIN16,m4, &motorThrust.m4)

LOG_GROUP_STOP(custom)



