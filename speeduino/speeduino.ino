/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,la
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

//**************************************************************************************************
// Config section
#define engineSquirtsPerCycle 2 //Would be 1 for a 2 stroke
//**************************************************************************************************

//https://developer.mbed.org/handbook/C-Data-Types
#include <stdint.h>
//************************************************
#include "globals.h"
#include "utils.h"
#include "table.h"
#include "scheduler.h"
#include "comms.h"
#include "cancomms.h"
#include "maths.h"
#include "fuel_calc.h"
#include "ignition_calc.h"
#include "timers.h"
//#include "display.h"
#include "decoders.h"
#include "idle.h"
#include "auxiliaries.h"
#include "sensors.h"
#include "src/PID_v1/PID_v1.h"
//#include "src/DigitalWriteFast/digitalWriteFast.h"
#include "errors.h"
#include "storage.h"
#include "scheduledIO.h"
#if defined (CORE_TEENSY)
#include <FlexCAN.h>
#endif

#if defined(CORE_PSOC5)
  //OLED display
  #include "displayspecial.h"
#endif

// Used to flip between the display lines
bool displayLine = 0;

struct config1 configPage1;
struct config2 configPage2;
struct config3 configPage3;
struct config10 configPage10;
struct config11 configPage11;

bool ignitionOn = false; //The current state of the ignition system
bool fuelOn = false; //The current state of the ignition system
bool fuelPumpOn = false; //The current status of the fuel pump

void (*trigger)(); //Pointer for the trigger function (Gets pointed to the relevant decoder)
void (*triggerSecondary)(); //Pointer for the secondary trigger function (Gets pointed to the relevant decoder)
uint16_t (*getRPM)(); //Pointer to the getRPM function (Gets pointed to the relevant decoder)
int (*getCrankAngle)(int); //Pointer to the getCrank Angle function (Gets pointed to the relevant decoder)
void (*triggerSetEndTeeth)(); //Pointer to the triggerSetEndTeeth function of each decoder

byte cltCalibrationTable[CALIBRATION_TABLE_SIZE];
byte iatCalibrationTable[CALIBRATION_TABLE_SIZE];
byte o2CalibrationTable[CALIBRATION_TABLE_SIZE];

//These variables are used for tracking the number of running sensors values that appear to be errors. Once a threshold is reached, the sensor reading will go to default value and assume the sensor is faulty
byte mapErrorCount = 0;
byte iatErrorCount = 0;
byte cltErrorCount = 0;

unsigned long counter;
unsigned long currentLoopTime; //The time the current loop started (uS)
unsigned long previousLoopTime; //The time the previous loop started (uS)

int CRANK_ANGLE_MAX = 720;

static byte coilHIGH = HIGH;
static byte coilLOW = LOW;
static byte fanHIGH = HIGH;             // Used to invert the cooling fan output
static byte fanLOW = LOW;               // Used to invert the cooling fan output

volatile uint16_t mainLoopCount;
byte deltaToothCount = 0; //The last tooth that was used with the deltaV calc
int rpmDelta;
byte ignitionCount;
byte maxIgnOutputs = 1; //Used for rolling rev limiter
byte curRollingCut = 0; //Rolling rev limiter, current ignition channel being cut
uint16_t fixedCrankingOverride = 0;
bool clutchTrigger;
bool previousClutchTrigger;

unsigned long secCounter; //The next time to incremen 'runSecs' counter.

//These are the functions the get called to begin and end the ignition coil charging. They are required for the various spark output modes
void (*ign1StartFunction)();
void (*ign1EndFunction)();
void (*ign2StartFunction)();
void (*ign2EndFunction)();
void (*ign3StartFunction)();
void (*ign3EndFunction)();
void (*ign4StartFunction)();
void (*ign4EndFunction)();
void (*ign5StartFunction)();
void (*ign5EndFunction)();

volatile int timePerDegree;
byte degreesPerLoop; //The number of crank degrees that pass for each mainloop of the program
volatile bool fpPrimed = false; //Tracks whether or not the fuel pump priming has been completed yet
bool initialisationComplete = false; //Tracks whether the setup() functino has run completely

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

#if defined(CORE_PSOC5)
  // Enable interrupts
  interrupts();
#endif

  // initialiseStorage components
  initialiseStorage();

  table3D_setSize(&fuelTable, 16);
  table3D_setSize(&ignitionTable, 16);
  table3D_setSize(&afrTable, 16);
  table3D_setSize(&boostTable, 8);
  table3D_setSize(&vvtTable, 8);
  table3D_setSize(&trim1Table, 6);
  table3D_setSize(&trim2Table, 6);
  table3D_setSize(&trim3Table, 6);
  table3D_setSize(&trim4Table, 6);

  loadConfig();

  doUpdates(); //Check if any data items need updating (Occurs ith firmware updates)

  Serial.begin(115200);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) //ATmega2561 does not have Serial3
  if (configPage10.enable_canbus == 1) { CANSerial.begin(115200); }
#elif defined(CORE_STM32)
  if (configPage10.enable_canbus == 1) { CANSerial.begin(115200); }
  else if (configPage10.enable_canbus == 2)
  {
    //enable local can interface
  }
#elif defined(CORE_TEENSY)
  if (configPage10.enable_canbus == 1) { CANSerial.begin(115200); }
  else if (configPage10.enable_canbus == 2)
  {
    //Teensy onboard CAN not used currently
    //enable local can interface
    //setup can interface to 250k
    //FlexCAN CANbus0(2500000, 0);
    //static CAN_message_t txmsg,rxmsg;
    //CANbus0.begin();
  }
#endif

  //Repoint the 2D table structs to the config pages that were just loaded
  taeTable.valueSize = SIZE_BYTE; //Set this table to use byte values
  taeTable.xSize = 4;
  taeTable.values = configPage2.taeValues;
  taeTable.axisX = configPage2.taeBins;
  WUETable.valueSize = SIZE_BYTE; //Set this table to use byte values
  WUETable.xSize = 10;
  WUETable.values = configPage1.wueValues;
  WUETable.axisX = configPage2.wueBins;
  crankingEnrichTable.valueSize = SIZE_BYTE;
  crankingEnrichTable.xSize = 4;
  crankingEnrichTable.values = configPage11.crankingEnrichValues;
  crankingEnrichTable.axisX = configPage11.crankingEnrichBins;

  dwellVCorrectionTable.valueSize = SIZE_BYTE;
  dwellVCorrectionTable.xSize = 6;
  dwellVCorrectionTable.values = configPage2.dwellCorrectionValues;
  dwellVCorrectionTable.axisX = configPage3.voltageCorrectionBins;
  injectorVCorrectionTable.valueSize = SIZE_BYTE;
  injectorVCorrectionTable.xSize = 6;
  injectorVCorrectionTable.values = configPage3.injVoltageCorrectionValues;
  injectorVCorrectionTable.axisX = configPage3.voltageCorrectionBins;
  IATDensityCorrectionTable.valueSize = SIZE_BYTE;
  IATDensityCorrectionTable.xSize = 9;
  IATDensityCorrectionTable.values = configPage3.airDenRates;
  IATDensityCorrectionTable.axisX = configPage3.airDenBins;
  IATRetardTable.valueSize = SIZE_BYTE;
  IATRetardTable.xSize = 6;
  IATRetardTable.values = configPage2.iatRetValues;
  IATRetardTable.axisX = configPage2.iatRetBins;
  rotarySplitTable.valueSize = SIZE_BYTE;
  rotarySplitTable.xSize = 8;
  rotarySplitTable.values = configPage11.rotarySplitValues;
  rotarySplitTable.axisX = configPage11.rotarySplitBins;

  //Setup the calibration tables - storage "class"
  loadCalibration();

  //Set the pin mappings
  if(configPage1.pinMapping > BOARD_NR_GPIO_PINS)
  {
    //First time running on this board
    setPinMapping(3); //Force board to v0.4
    configPage1.flexEnabled = false; //Have to disable flex. If this isn't done and the wrong flex pin is interrupt attached below, system can hang.
  }
  else { setPinMapping(configPage1.pinMapping); }

  //Need to check early on whether the coil charging is inverted. If this is not set straight away it can cause an unwanted spark at bootup
  if(configPage2.IgInv == 1) { coilHIGH = LOW, coilLOW = HIGH; }
  else { coilHIGH = HIGH, coilLOW = LOW; }
  endCoil1Charge();
  endCoil2Charge();
  endCoil3Charge();
  endCoil4Charge();
  endCoil5Charge();

  //Similar for injectors, make sure they're turned off
  closeInjector1();
  closeInjector2();
  closeInjector3();
  closeInjector4();
  closeInjector5();

  //Set the tacho output default state
  digitalWrite(pinTachOut, HIGH);
  //Perform most initialisations
  initialiseTimers();
  initialiseSchedulers();
  //initialiseDisplay();
  initialiseIdle();
  initialiseFan();
#ifndef CORE_PSOC5
  initialiseAuxPWM();
#endif
  initialiseADC();
  fuel_calc_init();
  ignition_calc_init();

#if defined(CORE_PSOC5)
  // Init display
  displayspecial.begin();
#endif

  //Lookup the current MAP reading for barometric pressure
  instanteneousMAPReading();

  //barometric reading can be taken from either an external sensor if enabled, or simply by using the initial MAP value
  if ( configPage3.useExtBaro != 0 )
  {
    readBaro();
    UpdateStorage(EEPROM_LAST_BARO, currentStatus.baro);
  }
  else
  {
    /*
     * The highest sea-level pressure on Earth occurs in Siberia, where the Siberian High often attains a sea-level pressure above 105 kPa;
     * with record highs close to 108.5 kPa.
     * The lowest measurable sea-level pressure is found at the centers of tropical cyclones and tornadoes, with a record low of 87 kPa;
     */
    if ((currentStatus.MAP >= BARO_MIN) && (currentStatus.MAP <= BARO_MAX)) //Check if engine isn't running
    {
      currentStatus.baro = currentStatus.MAP;
      UpdateStorage(EEPROM_LAST_BARO, currentStatus.baro);
    }
    else
    {
      //Attempt to use the last known good baro reading from EEPROM
      if ((ReadStorage(EEPROM_LAST_BARO) >= BARO_MIN) && (ReadStorage(EEPROM_LAST_BARO) <= BARO_MAX)) //Make sure it's not invalid (Possible on first run etc)
      { currentStatus.baro = ReadStorage(EEPROM_LAST_BARO); } //last baro correction
      else { currentStatus.baro = 100; } //Final fall back position.
    }
  }

  //Check whether the flex sensor is enabled and if so, attach an interupt for it
  if(configPage1.flexEnabled)
  {
    attachInterrupt(digitalPinToInterrupt(pinFlex), flexPulse, RISING);
    currentStatus.ethanolPct = 0;
  }

  currentStatus.RPM = 0;
  currentStatus.hasSync = false;
  currentStatus.runSecs = 0;
  currentStatus.secl = 0;
  currentStatus.startRevolutions = 0;
  currentStatus.flatShiftingHard = false;
  currentStatus.launchingHard = false;
  currentStatus.crankRPM = ((unsigned int)configPage2.crankRPM * 100); //Crank RPM limit (Saves us calculating this over and over again. It's updated once per second in timers.ino)
  triggerFilterTime = 0; //Trigger filter time is the shortest possible time (in uS) that there can be between crank teeth (ie at max RPM). Any pulses that occur faster than this time will be disgarded as noise. This is simply a default value, the actual values are set in the setup() functinos of each decoder
  dwellLimit_uS = (1000 * configPage2.dwellLimit);

  noInterrupts();

  initialiseTriggers(); //Crank/cam triger interrupt attachment

  //Initial values for loop times
  previousLoopTime = 0;
  currentLoopTime = micros();

  mainLoopCount = 0;
  ignitionCount = 0;

  //Begin priming the fuel pump. This is turned off in the low resolution, 1s interrupt in timers.ino
  digitalWrite(pinFuelPump, HIGH);
  fuelPumpOn = true;

  interrupts();
  //Perform the priming pulses. Set these to run at an arbitrary time in the future (100us). The prime pulse value is in ms*10, so need to multiple by 100 to get to uS
  setFuelSchedule1(100, (unsigned long)(configPage1.primePulse * 100));
  setFuelSchedule2(100, (unsigned long)(configPage1.primePulse * 100));
  setFuelSchedule3(100, (unsigned long)(configPage1.primePulse * 100));
  setFuelSchedule4(100, (unsigned long)(configPage1.primePulse * 100));
  initialisationComplete = true;

  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
      mainLoopCount++;
      LOOP_TIMER = TIMER_mask;

      //Check for any requets from serial. Serial operations are checked under 2 scenarios:
      // 1) 10 times pr. second (This is more than fast enough for TunerStudio).
      // 2) If the amount of data in the serial buffer is greater than a set threhold (See globals.h). This is to avoid serial buffer overflow when large amounts of data is being sent
      if ( (BIT_CHECK(TIMER_mask, BIT_TIMER_10HZ)) || (Serial.available() > SERIAL_BUFFER_THRESHOLD) )
      {
        if (Serial.available() > 0) { command(); }
      }

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) //ATmega2561 does not have Serial3
      //if serial3 interface is enabled then check for serial3 requests.
      if (configPage10.enable_canbus == 1)
      {
        if ( (BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)) || (CANSerial.available() > SERIAL_BUFFER_THRESHOLD) )
        {
          if (CANSerial.available() > 0) { canCommand(); }
        }
      }

#elif  defined(CORE_TEENSY) || defined(CORE_STM32)
      //if can or secondary serial interface is enabled then check for requests.
      if (configPage10.enable_canbus == 1)  //secondary serial interface enabled
      {
        if ( (BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)) || (CANSerial.available() > SERIAL_BUFFER_THRESHOLD) )
        {
          if (CANSerial.available() > 0) { canCommand(); }
        }
      }
      else if (configPage10.enable_canbus == 2) // can module enabled
          {
            //check local can module
            // if ( (BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ)) or (CANbus0.available())
            //    {
            //      CANbus0.read(rx_msg);
            //    }
          }
#endif

    //Displays currently disabled
    // if (configPage1.displayType && (mainLoopCount & 255) == 1) { updateDisplay();}

    previousLoopTime = currentLoopTime;
    currentLoopTime = micros();
    unsigned long timeToLastTooth = (currentLoopTime - toothLastToothTime);

    if ( (timeToLastTooth < MAX_STALL_TIME) || (toothLastToothTime > currentLoopTime) ) //Check how long ago the last tooth was seen compared to now. If it was more than half a second ago then the engine is probably stopped. toothLastToothTime can be greater than currentLoopTime if a pulse occurs between getting the lastest time and doing the comparison
    {
      currentStatus.RPM = currentStatus.longRPM = getRPM(); //Long RPM is included here
      FUEL_PUMP_ON();
      fuelPumpOn = true; //Not sure if this is needed.
    }
    else
    {
      //We reach here if the time between teeth is too great. This VERY likely means the engine has stopped
      currentStatus.RPM = 0;
      currentStatus.PW1 = 0;
      currentStatus.VE = 0;
      toothLastToothTime = 0;
      toothLastSecToothTime = 0;
      //toothLastMinusOneToothTime = 0;
      currentStatus.hasSync = false;
      currentStatus.runSecs = 0; //Reset the counter for number of seconds running.
      secCounter = 0; //Reset our seconds counter.
      currentStatus.startRevolutions = 0;
      toothSystemCount = 0;
      secondaryToothCount = 0;
      MAPcurRev = 0;
      MAPcount = 0;
      currentStatus.rpmDOT = 0;
      ignitionOn = false;
      fuelOn = false;
      if (fpPrimed == true) { digitalWrite(pinFuelPump, LOW); fuelPumpOn = false; } //Turn off the fuel pump, but only if the priming is complete
      disableIdle(); //Turn off the idle PWM
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK); //Clear cranking bit (Can otherwise get stuck 'on' even with 0 rpm)
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP); //Same as above except for WUE
      //This is a safety check. If for some reason the interrupts have got screwed up (Leading to 0rpm), this resets them.
      //It can possibly be run much less frequently.
      initialiseTriggers();

#ifndef CORE_PSOC5 // Current not ported to PSOC5
      VVT_PIN_LOW();
      DISABLE_VVT_TIMER();
      boostDisable();
#endif
    }

    //Uncomment the following for testing
    /*
    currentStatus.hasSync = true;
    currentStatus.RPM = 500;
    */

    //***Perform MAP sensor read***
    readMAP();


    // *******************************************************************************
    // Place functions that needs to run 30 times per second here
    // *******************************************************************************
    if(BIT_CHECK(LOOP_TIMER, BIT_TIMER_30HZ)) //30 hertz
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_30HZ);

      //Most boost tends to run at about 30Hz, so placing it here ensures a new target time is fetched frequently enough
#ifndef CORE_PSOC5 // Current not ported to PSCO5
      boostControl();
#endif
    } // End 30 times per second

    // *******************************************************************************
    // Place functions that needs to run 15 times per second here
    // *******************************************************************************
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_15HZ))
    {
      BIT_CLEAR(TIMER_mask, BIT_TIMER_15HZ);
      readTPS(); //TPS reading to be performed every 32 loops (any faster and it can upset the TPSdot sampling time)

      //Check for launching/flat shift (clutch) can be done around here too
      previousClutchTrigger = clutchTrigger;
      if(configPage3.launchHiLo) { clutchTrigger = digitalRead(pinLaunch); }
      else { clutchTrigger = !digitalRead(pinLaunch); }

      if(previousClutchTrigger != clutchTrigger) { currentStatus.clutchEngagedRPM = currentStatus.RPM; }

      if (configPage3.launchEnabled && clutchTrigger && (currentStatus.clutchEngagedRPM < ((unsigned int)(configPage3.flatSArm) * 100)) && (currentStatus.RPM > ((unsigned int)(configPage3.lnchHardLim) * 100)) ) { currentStatus.launchingHard = true; BIT_SET(currentStatus.spark, BIT_SPARK_HLAUNCH); } //HardCut rev limit for 2-step launch control.
      else { currentStatus.launchingHard = false; BIT_CLEAR(currentStatus.spark, BIT_SPARK_HLAUNCH); }

      if(configPage3.flatSEnable && clutchTrigger && (currentStatus.RPM > ((unsigned int)(configPage3.flatSArm) * 100)) && (currentStatus.RPM > currentStatus.clutchEngagedRPM) ) { currentStatus.flatShiftingHard = true; }
      else { currentStatus.flatShiftingHard = false; }

      //Boost cutoff is very similar to launchControl, but with a check against MAP rather than a switch
      if(configPage3.boostCutType && currentStatus.MAP > (configPage3.boostLimit * 2) ) //The boost limit is divided by 2 to allow a limit up to 511kPa
      {
        switch(configPage3.boostCutType)
        {
          case 1:
            BIT_SET(currentStatus.spark, BIT_SPARK_BOOSTCUT);
            BIT_CLEAR(currentStatus.squirt, BIT_SQUIRT_BOOSTCUT);
            break;
          case 2:
            BIT_SET(currentStatus.squirt, BIT_SQUIRT_BOOSTCUT);
            BIT_CLEAR(currentStatus.spark, BIT_SPARK_BOOSTCUT);
            break;
          case 3:
            BIT_SET(currentStatus.spark, BIT_SPARK_BOOSTCUT);
            BIT_SET(currentStatus.squirt, BIT_SQUIRT_BOOSTCUT);
            break;
        }
      }
      else
      {
        BIT_CLEAR(currentStatus.spark, BIT_SPARK_BOOSTCUT);
        BIT_CLEAR(currentStatus.squirt, BIT_SQUIRT_BOOSTCUT);
      }

      //And check whether the tooth log buffer is ready
      if(toothHistoryIndex > TOOTH_LOG_SIZE) { BIT_SET(currentStatus.squirt, BIT_SQUIRT_TOOTHLOG1READY); }
    } // End 15 times per second


    // *******************************************************************************
    // Place functions that needs to run 4 times per second here
    // *******************************************************************************
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_4HZ))
    {
       BIT_CLEAR(TIMER_mask, BIT_TIMER_4HZ);
       readCLT();
       readIAT();
       readO2();
       readBat();

#if defined(CORE_PSOC5)
       // Handle display update - one line is updated at a time to reduce the time waisted on the slow I2C bus
       if (displayLine == 0)
       {
         displayspecial.PrintNumberSpecial(currentStatus.O2,0);
       }
       else
       {
         displayspecial.PrintNumberSpecial(currentStatus.coolant,1);
       }
       displayLine = !displayLine;
#endif

       if(eepromWritesPending == true) { writeAllConfig(); } //Check for any outstanding EEPROM writes.

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) //ATmega2561 does not have Serial3
      //if Can interface is enabled then check for serial3 requests.
      if (configPage10.enable_canbus == 1)  // megas only support can via secondary serial
          {
            if (configPage10.enable_candata_in)
              {
                if (BIT_CHECK(configPage10.caninput_sel,currentStatus.current_caninchannel))  //if current input channel bit is enabled
                  {
                    sendCancommand(2,0,currentStatus.current_caninchannel,0,((configPage10.caninput_param_group[currentStatus.current_caninchannel]&2047)+256));    //send an R command for data from paramgroup[currentStatus.current_caninchannel]
                  }
                else
                  {
                    if (currentStatus.current_caninchannel < 15)
                        {
                          currentStatus.current_caninchannel++;   //step to next input channel if under 15
                        }
                    else
                        {
                          currentStatus.current_caninchannel = 0;   //reset input channel back to 1
                        }
                  }
              }
          }
#elif defined(CORE_STM32) || defined(CORE_TEENSY)
      //if serial3io is enabled then check for serial3 requests.
      if (configPage10.enable_candata_in)
        {
          if (BIT_CHECK(configPage10.caninput_sel,currentStatus.current_caninchannel))  //if current input channel is enabled
            {
              if (configPage10.enable_canbus == 1)  //can via secondary serial
              {
                sendCancommand(2,0,currentStatus.current_caninchannel,0,((configPage10.caninput_param_group[currentStatus.current_caninchannel]&2047)+256));    //send an R command for data from paramgroup[currentStatus.current_caninchannel]
              }
              else if (configPage10.enable_canbus == 2) // can via internal can module
              {
                sendCancommand(3,configPage10.speeduino_tsCanId,currentStatus.current_caninchannel,0,configPage10.caninput_param_group[currentStatus.current_caninchannel]);    //send via localcanbus the command for data from paramgroup[currentStatus.current_caninchannel]
              }
            }
          else
            {
              if (currentStatus.current_caninchannel < 15)
                  {
                    currentStatus.current_caninchannel++;   //step to next input channel if under 15
                  }
              else
                  {
                    currentStatus.current_caninchannel = 0;   //reset input channel back to 0
                  }
            }
        }
#endif

#ifndef CORE_PSOC5 // Current not ported to PSOC5
	     vvtControl();
#endif

       //Perform non stepper idle related actions. Even at higher frequencies, running 4x per second is sufficient.
       if(configPage3.iacAlgorithm != IAC_ALGORITHM_STEP_OL || configPage3.iacAlgorithm != IAC_ALGORITHM_STEP_CL) { idleControl(); }
    } // End 4 times per second

    // *******************************************************************************
    // Place functions that needs to run every second here
    // *******************************************************************************
    if (BIT_CHECK(LOOP_TIMER, BIT_TIMER_1HZ)) //Once per second
    {
        BIT_CLEAR(TIMER_mask, BIT_TIMER_1HZ);
        readBaro(); //Infrequent baro readings are not an issue.

        //Check the fan output status
        if (configPage3.fanEnable == 1)
        {
           fanControl();            // Fucntion to turn the cooling fan on/off
        }

#if defined(CORE_PSOC5)
    		// flip the LED (PIN 1) to show main loop is alive
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif

    } // End Once per second

    if(configPage3.iacAlgorithm == IAC_ALGORITHM_STEP_OL || configPage3.iacAlgorithm == IAC_ALGORITHM_STEP_CL) { idleControl(); } //Run idlecontrol every loop for stepper idle.

    //Always check for sync
    //Main loop runs within this clause
    if (currentStatus.hasSync && (currentStatus.RPM > 0))
    {
        if(currentStatus.startRevolutions >= configPage2.StgCycles)  { ignitionOn = true; fuelOn = true; } //Enable the fuel and ignition, assuming staging revolutions are complete
        //If it is, check is we're running or cranking
        if(currentStatus.RPM > currentStatus.crankRPM) //Crank RPM stored in byte as RPM / 100
        {
          BIT_SET(currentStatus.engine, BIT_ENGINE_RUN); //Sets the engine running bit
          //Only need to do anything if we're transitioning from cranking to running
          if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
          {
            BIT_CLEAR(currentStatus.engine, BIT_ENGINE_CRANK); //clears the engine cranking bit
            if(configPage2.ignBypassEnabled) { digitalWrite(pinIgnBypass, HIGH); }
          }
        }
        else
        {  //Sets the engine cranking bit, clears the engine running bit
          BIT_SET(currentStatus.engine, BIT_ENGINE_CRANK);
          BIT_CLEAR(currentStatus.engine, BIT_ENGINE_RUN);
          currentStatus.runSecs = 0; //We're cranking (hopefully), so reset the engine run time to prompt ASE.
          if(configPage2.ignBypassEnabled) { digitalWrite(pinIgnBypass, LOW); }
        }
      //END SETTING STATUSES
      //-----------------------------------------------------------------------------------------------------

      //********************************************************
      //How fast are we going? Need to know how long (uS) it will take to get from one tooth to the next. We then use that to estimate how far we are between the last tooth and the next one
      //We use a 1st Deriv accleration prediction, but only when there is an even spacing between primary sensor teeth
      //Any decoder that has uneven spacing has its triggerToothAngle set to 0
      if(secondDerivEnabled && toothHistoryIndex >= 3 && currentStatus.RPM < 2000) //toothHistoryIndex must be greater than or equal to 3 as we need the last 3 entries. Currently this mode only runs below 3000 rpm
      //if(true)
      {
        //Only recalculate deltaV if the tooth has changed since last time (DeltaV stays the same until the next tooth)
        //if (deltaToothCount != toothCurrentCount)
        {
          deltaToothCount = toothCurrentCount;
          int angle1, angle2; //These represent the crank angles that are travelled for the last 2 pulses
          if(configPage2.TrigPattern == 4)
          {
            //Special case for 70/110 pattern on 4g63
            angle2 = triggerToothAngle; //Angle 2 is the most recent
            if (angle2 == 70) { angle1 = 110; }
            else { angle1 = 70; }
          }
          else if(configPage2.TrigPattern == 0)
          {
            //Special case for missing tooth decoder where the missing tooth was one of the last 2 seen
            if(toothCurrentCount == 1) { angle2 = 2*triggerToothAngle; angle1 = triggerToothAngle; }
            else if(toothCurrentCount == 2) { angle1 = 2*triggerToothAngle; angle2 = triggerToothAngle; }
            else { angle1 = angle2 = triggerToothAngle; }
          }
          else { angle1 = angle2 = triggerToothAngle; }

          long toothDeltaV = (1000000L * angle2 / toothHistory[toothHistoryIndex]) - (1000000L * angle1 / toothHistory[toothHistoryIndex-1]);
          long toothDeltaT = toothHistory[toothHistoryIndex];
          //long timeToLastTooth = micros() - toothLastToothTime;

          rpmDelta = (toothDeltaV << 10) / (6 * toothDeltaT);
        }

        timePerDegree = ldiv( 166666L, (currentStatus.RPM + rpmDelta)).quot; //There is a small amount of rounding in this calculation, however it is less than 0.001 of a uS (Faster as ldiv than / )
      }
      else
      {
        long rpm_adjust = ((long)(micros() - toothOneTime) * (long)currentStatus.rpmDOT) / 1000000; //Take into account any likely accleration that has occurred since the last full revolution completed

        //timePerDegree = DIV_ROUND_CLOSEST(166666L, (currentStatus.RPM + rpm_adjust));
        timePerDegree = ldiv( 166666L, currentStatus.RPM + rpm_adjust).quot; //There is a small amount of rounding in this calculation, however it is less than 0.001 of a uS (Faster as ldiv than / )
      }

      //Debug
      timePerDegree = 166666L / currentStatus.RPM;

      //Determine the current crank angle
      int crankAngle = getCrankAngle( timePerDegree );

      // Calculate fuel parameters and schedule injection
      fuel_calc( crankAngle, timePerDegree, triggerToothAngle, fuelOn );

      // Calculate ignition parameters and schedule ignition
      ignition_calc( crankAngle, timePerDegree, ignitionOn );

    } //Has sync and RPM
} //loop()
