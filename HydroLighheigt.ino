/**
*  @file    HydroLighting.ino
*  @author  peter c
*  @date    11/05/2017
*  @version 0.1
*
*
*  @section DESCRIPTION
*  Ligting Height control via MOSFET H-Bridge and hall effect
*  for position
*  based on BLDG 1 ROW 1 CHMB 1A LIGHTS FRONT
** @section HISTORY
** 2017Nov2 - created
*/
#include <HardwareSerial.h>
#include <Streaming.h>
#include <DA_DiscreteInput.h>
#include <DA_NonBlockingDelay.h>
#include "unitModbus.h"
// comment out to  include terminal processing for debugging
//#define PROCESS_TERMINAL
// #define TRACE_1WIRE
// #define TRACE_ANALOGS
// #define TRACE_DISCRETES
// #define TRACE_MODBUS
// comment out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define WATCH_DOG_COUNT_ELAPSED 2 // when the count reaches this && dir is DOWN and MOTOR ON=>no pulses have been recieved 
                                  // for WATCH_DOG_COUNT_ELAPSED * POLL_CYCLE_SECONDS
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate
#define POSITION_INTERRUPT_PIN  3 // pulses from hall effect sensor
//DA_DiscreteInput B1R1_1A_ZT_001_Int =DA_DiscreteInput(POSITION_INTERRUPT_PIN, DA_DiscreteInput::None, true);
#define ENABLE_HALL_SENSOR_RISING_INTERRUPTS attachInterrupt(digitalPinToInterrupt(POSITION_INTERRUPT_PIN), on_B1R1_1A_ZT_001_Rising, RISING)
#define DISABLE_HALL_SENSOR_INTERRUPTS detachInterrupt(digitalPinToInterrupt(POSITION_INTERRUPT_PIN))
// MOSFET H-Bridge
// Q1,Q3 = P-Channel
// Q2,Q4 = N-Channel
// U4,U5 = MOSFET driver
// UX A pins drive Q1,Q4
// UX B pin  drive Q2, Q3
// 
// 
/*
#define U4A_PIN 11 // Ardiuno pin
#define U4B_PIN 12
#define U5A_PIN 9
#define U5B_PIN 10
*/
// D15,14,13,12,11,10,9,8 D13-D15 not used and set to 0
// Want to write bits as a whole and not via indidual calls
// to digitalWrite() as delays could short H-Bridge
// 
#define MOTOR_CONTROL_PINS B00011110 // 12,11,10,9 pins are outputs
#define MOTOR_OFF_MASK B00010100
#define MOTOR_DOWN_MASK B00001100
#define MOTOR_UP_MASK B00010010
enum motorDirectionType
{
  UP, DOWN
};

enum motorStateType
{
  ON, OFF, FORCED_OFF
};

enum lightingPositionType
{
  TOP, BOTTOM
};

motorDirectionType B1R1_1A_ZY_001 = DOWN; // UP/DOWN DIRECTION
motorStateType B1R1_1A_XY_026 = OFF; // ON/OFF HEIGHT ADJUST
lightingPositionType B1R1_1A_ZY_001_POS; // UP or DOWN when limits are reached.
// bool B1R1_1A_QR_026 = faB1R1_1A_ZY_001lse; // HOME RESET
// count the number of pulses via hall sensor
// 0 to MAX
// MAX to 0
// #define  LINEAR_ACTUATOR_MAX_LENGTH_PULSE_COUNT  237100 // #of pulses emitted to reach the maximum length. No datasheet. Emperically determined
#define LINEAR_ACTUATOR_MAX_LENGTH_PULSE_COUNT 500L
volatile long B1R1_1A_ZT_001_PULSE_CNT = 0;
volatile unsigned short minPositionWatchDog = 0;
// poll I/O every 2 seconds
DA_NonBlockingDelay pollTimer = DA_NonBlockingDelay(2000, & doOnPoll);
// if up to down or down to up command, stop and wait 500 ms before changing
// direction
// DA_NonBlockingDelay motorChangeDirTimer = DA_NonBlockingDelay(500, & doOnDirChange);
// HEARTBEAT
unsigned int heartBeat = 0;

#ifdef PROCESS_TERMINAL
HardwareSerial * tracePort = & Serial;
#endif

void on_B1R1_1A_ZT_001_Rising()
{
  if(B1R1_1A_XY_026 == ON )
  {
  if (B1R1_1A_ZY_001 == UP)
    B1R1_1A_ZT_001_PULSE_CNT++;
  else
  {

    if(--B1R1_1A_ZT_001_PULSE_CNT < 0 )
      B1R1_1A_ZT_001_PULSE_CNT = 0;
  }
  }
  minPositionWatchDog = 0;
}

void setup()
{
  DDRB = MOTOR_CONTROL_PINS;
  doMotorOff(false);

#ifdef PROCESS_TERMINAL
  tracePort -> begin(9600);
  * tracePort << "start" << endl;
  * tracePort << "start direction:" << (B1R1_1A_ZY_001 == UP) << endl;
#endif

#ifdef PROCESS_MODBUS
  slave.begin(MB_SPEED);
#endif


  ENABLE_HALL_SENSOR_RISING_INTERRUPTS;
  randomSeed(analogRead(3));
}

void loop()
{

#ifdef PROCESS_MODBUS
  refreshModbusRegisters();
  slave.poll(modbusRegisters, MODBUS_REG_COUNT);
  processModbusCommands();
#endif

  pollTimer.refresh();
}

void doMaxPosition()
{
  doMotorOff(true);
  #ifdef PROCESS_TERMINAL
  *tracePort << "maxPosition Reached" << endl;
  #endif
  B1R1_1A_ZY_001_POS = TOP;
  B1R1_1A_ZT_001_PULSE_CNT = LINEAR_ACTUATOR_MAX_LENGTH_PULSE_COUNT;
}

bool isMaxPostion()
{
  return(B1R1_1A_ZT_001_PULSE_CNT >= LINEAR_ACTUATOR_MAX_LENGTH_PULSE_COUNT);
}

void doMinPosition()
{
  doMotorOff(true);
  #ifdef PROCESS_TERMINAL
  *tracePort << "minimum Reached" << endl;
  #endif
  B1R1_1A_ZY_001_POS = BOTTOM;
  B1R1_1A_ZT_001_PULSE_CNT = 0;
}

bool isMinPosition()
{

  return(minPositionWatchDog >= WATCH_DOG_COUNT_ELAPSED);
}

void doMaxPositionCheck()
{
  if (B1R1_1A_XY_026 == ON && B1R1_1A_ZY_001 == UP)
  {
    #ifdef PROCESS_TERMINAL
    *tracePort << "checking if at the top" << endl;
    #endif
    if (isMaxPostion())
      doMaxPosition();
  }
}

void doMinPositionCheck()
{
  if (B1R1_1A_XY_026 == ON && B1R1_1A_ZY_001 == DOWN)
  {
        #ifdef PROCESS_TERMINAL
    *tracePort << "checking if at the bottom" << endl;
    #endif
    if (isMinPosition())
      doMinPosition();
  }
}

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
  heartBeat++;

  doMaxPositionCheck();
  doMinPositionCheck();
    if(B1R1_1A_XY_026 == ON && B1R1_1A_ZY_001 == DOWN )
    minPositionWatchDog++;
}

void doMotorUp()
{
  PORTB = MOTOR_UP_MASK;

#ifdef PROCESS_TERMINAL
  * tracePort << "Motor going up" << endl;
#endif

}

void doMotorDown()
{
  PORTB = MOTOR_DOWN_MASK;

#ifdef PROCESS_TERMINAL
  * tracePort << "Motor going down" << endl;
#endif

}

void doMotorOff(bool aForcedOff)
{
  PORTB = MOTOR_OFF_MASK;
  if (!aForcedOff)
    B1R1_1A_XY_026 = OFF;
  else
    B1R1_1A_XY_026 = FORCED_OFF;

#ifdef PROCESS_TERMINAL
  * tracePort << "Motor  off" << endl;
#endif

}

unsigned long getPosition()
{
  return(B1R1_1A_ZT_001_PULSE_CNT);
};

// 
/*
** Modbus related functions
*/
void refreshModbusRegisters()
{
  // modbusRegisters[HR_TEMPERATURE1] =  -127; // B1R1_1A_TT_001.getTempCByIndex(0) * 100;
  modbusRegisters[HR_HEARTBEAT] = heartBeat;
  blconvert.val = getPosition();
  modbusRegisters[B1R1_1A_ZT_001_MB] = blconvert.regsl[1];
  modbusRegisters[B1R1_1A_ZT_001_MB + 1] = blconvert.regsl[0];
}

bool getModbusCoilValue(unsigned short startAddress, unsigned short bitPos)
{
  // *tracePort << "reading at " << startAddress << " bit offset " << bitPos << "value=" << bitRead(modbusRegisters[startAddress + (int)(bitPos / 16)], bitPos % 16 ) << endl;
  return(bitRead(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16));
}

void writeModbusCoil(unsigned short startAddress, unsigned short bitPos, bool value)
{
  bitWrite(modbusRegisters[startAddress + (int) (bitPos / 16)], bitPos % 16, value);
}

void checkforStartStopCommand()
{
  if (B1R1_1A_XY_026 == FORCED_OFF)
  {
    writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, B1R1_1A_XY_026_MB, false);
    B1R1_1A_XY_026 = OFF;
  }
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, B1R1_1A_XY_026_MB))
  {
    if (B1R1_1A_XY_026 == OFF)
    {
      B1R1_1A_XY_026 = ON;
      if (B1R1_1A_ZY_001 == UP)
        doMotorUp();
      else
      {
        doMotorDown();
      }

#ifdef PROCESS_TERMINAL
      * tracePort << "Motor is " << (B1R1_1A_XY_026 == ON) << endl;
#endif

    }
    // writeModbusCoil(COIL_STATUS_READ_WRITE_OFFSET, B1R1_1A_XY_026_MB, false);
  }
  else
  {
    if (B1R1_1A_XY_026 == ON)
      doMotorOff(false);
  }
}

void checkForHomeResetCommand()
{
}

void checkForChangeDirectionCommand()
{
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, B1R1_1A_ZY_001_MB))
  {
    B1R1_1A_ZY_001 = UP;
  }
  else
  {
    B1R1_1A_ZY_001 = DOWN;
  }
}

void processModbusCommands()
{
  checkForChangeDirectionCommand();
  checkforStartStopCommand();
}
