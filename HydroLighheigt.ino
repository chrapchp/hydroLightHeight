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
#include <DA_NonBlockingDelay.h>
#include "unitModbus.h"
// comment out to  include terminal processing for debugging
// #define PROCESS_TERMINAL
// #define TRACE_1WIRE
// #define TRACE_ANALOGS
// #define TRACE_DISCRETES
// #define TRACE_MODBUS
// comment out to disable modbus
#define PROCESS_MODBUS
// refresh intervals
#define POLL_CYCLE_SECONDS 2 // sonar and 1-wire refresh rate
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
#define MOTOR_UP_MASK B00011000
#define MOTOR_DOWN_MASK B00010010

bool B1R1_1A_ZY_001 = false; // UP/DOWN DIRECTION
bool B1R1_1A_XY_026 = false; // ON/OFF HEIGHT ADJUST
bool B1R1_1A_QR_026 = false; // HOME RESET
// count the number of pulses via hall sensor
// 0 to MAX
// MAX to 0
volatile long B1R1_1A_ZT_001 = 0;
// poll I/O every 2 seconds
DA_NonBlockingDelay pollTimer = DA_NonBlockingDelay(2000, & doOnPoll);
// if up to down or down to up command, stop and wait 500 ms before changing
// direction
//DA_NonBlockingDelay motorChangeDirTimer = DA_NonBlockingDelay(500, & doOnDirChange);
// HEARTBEAT
unsigned int heartBeat = 0;
enum motorState
{
   STOPPED = 1, UP_PENDING=2, UP=3, DOWN_PENDING=4, DOWN=5
};

motorState currentMotorState = STOPPED|UP_PENDING;

#ifdef PROCESS_TERMINAL
HardwareSerial * tracePort = & Serial;
#endif

void setup()
{

#ifdef PROCESS_TERMINAL
  tracePort -> begin(9600);
#endif

#ifdef PROCESS_MODBUS
  slave.begin(MB_SPEED);
#endif

pinMode(13, OUTPUT);
  DDRB = MOTOR_CONTROL_PINS;
 doMotorOff();
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

// update sonar and 1-wire DHT-22 readings
void doOnPoll()
{
  heartBeat++;
}

/*
void doOnDirChange()
{
switch (currentMotorState)
{
case UP_PENDING:
PORTB = MOTOR_UP_MASK;
currentMotorState = UP;
break;
case DOWN_PENDING:
PORTB = MOTOR_DOWN_MASK;
currentMotorState = DOWN;
break;
default:
break;
}
motorChangeDirTimer.pause();
}
*/
void doMotorUp()
{
  PORTB = MOTOR_UP_MASK;
  bitSet(currentMotorState, UP);
  bitClear(currentMotorState,UP_PENDING);
  bitClear(currentMotorState,STOP);

  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);


}

void doMotorDown()
{
  PORTB = MOTOR_DOWN_MASK;
  bitSet(currentMotorState, DOWN);
  bitClear(currentMotorState,DOWN_PENDING);

  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
  delay(250);
  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
}

void doMotorOff()
{
  PORTB = MOTOR_OFF_MASK;

    bitSet(currentMotorState, STOPPED);
      digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
  delay(500);
  digitalWrite(13, HIGH);
  delay(250);
  digitalWrite(13, LOW);
}

unsigned getPosition()
{
  return(50);
}

// 
/*
** Modbus related functions
*/

#ifdef PROCESS_MODBUS
void refreshModbusRegisters()
{
  // modbusRegisters[HR_TEMPERATURE1] =  -127; // B1R1_1A_TT_001.getTempCByIndex(0) * 100;
  modbusRegisters[HR_HEARTBEAT] = heartBeat;
  modbusRegisters[B1R1_1A_ZT_001_MB] = getPosition();
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
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, B1R1_1A_XY_026_MB))
  {
    if (currentMotorState & UP_PENDING)
      doMotorUp();
    else
      doMotorDown();
  }
  else
    doMotorOff();
}

void checkForHomeResetCommand()
{
}

void checkForChangeDirectionCommand()
{
  if (getModbusCoilValue(COIL_STATUS_READ_WRITE_OFFSET, B1R1_1A_ZY_001_MB))
  {
    bitSet(currentMotorState, UP_PENDING);
    bitClear(currentMotorState, DOWN_PENDING);
  }
  else
  {
    bitSet(currentMotorState, DOWN_PENDING);
    bitClear(currentMotorState, UP_PENDING);
  }
}

void processModbusCommands()
{
  checkForChangeDirectionCommand();
  checkforStartStopCommand();
}

#endif
