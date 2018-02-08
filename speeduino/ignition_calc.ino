/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

//*********************************************************************************************************
// All ignition calculations should be done in this file
//*********************************************************************************************************

#include "ignition_calc.h"
#include "globals.h"

#include "scheduler.h"

// Private/internal functions
static inline int8_t correctionFixedTiming(int8_t);
static inline int8_t correctionCrankingFixedTiming(int8_t);
static inline int8_t correctionFlexTiming(int8_t);
static inline int8_t correctionIATretard(int8_t);
static inline int8_t correctionSoftRevLimit(int8_t);
static inline int8_t correctionSoftLaunch(int8_t);
static inline int8_t correctionSoftFlatShift(int8_t);
uint16_t correctionsDwell(uint16_t dwell);
void ignitionToothScheduleCalc( int itscDwell, int itscIgnDegrees, int itscTimePerDegree, int itscTriggerToothAngle, int itscCRANK_ANGLE_MAX_IGN, uint8_t itscIgnitionPortMinusOne );

int CRANK_ANGLE_MAX_IGN = 360; // The number of crank degrees that the system track over. 360 for wasted / timed batch and 720 for sequential
int16_t lastAdvance; //Stores the previous advance figure to track changes.

int ignition1StartAngle = 0;
int ignition2StartAngle = 0;
int ignition3StartAngle = 0;
int ignition4StartAngle = 0;
int ignition5StartAngle = 0;

int channel1IgnDegrees; //The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones)
int channel2IgnDegrees; //The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC
int channel3IgnDegrees; //The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC
int channel4IgnDegrees; //The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC
int channel5IgnDegrees; //The number of crank degrees until cylinder 5 is at TDC

//*********************************************************************************************************
// Initilisation of stuff used for ignition calculations
//*********************************************************************************************************
void ignition_calc_init()
{
  currentStatus.flexIgnCorrection = 0;

  //Calculate the number of degrees between cylinders
  switch (configPage1.nCylinders) {
    case 1:
      channel1IgnDegrees = 0;
      break;

    case 2:
      channel1IgnDegrees = 0;
      maxIgnOutputs = 2;
      if (configPage1.engineType == EVEN_FIRE )
      {
        channel2IgnDegrees = 180;
      }
      else { channel2IgnDegrees = configPage1.oddfire2; }
      break;

    case 3:
      channel1IgnDegrees = 0;
      maxIgnOutputs = 3;
      if (configPage1.engineType == EVEN_FIRE )
      {
        if(configPage2.sparkMode == IGN_MODE_SEQUENTIAL)
        {
          channel2IgnDegrees = 240;
          channel3IgnDegrees = 480;

          CRANK_ANGLE_MAX_IGN = 720;
        }
        else
        {
          channel2IgnDegrees = 120;
          channel3IgnDegrees = 240;
        }
      }
      else
      {
        channel2IgnDegrees = configPage1.oddfire2;
        channel3IgnDegrees = configPage1.oddfire3;
      }
      break;

    case 4:
      channel1IgnDegrees = 0;
      maxIgnOutputs = 2; //Default value for 4 cylinder, may be changed below
      if (configPage1.engineType == EVEN_FIRE )
      {
        channel2IgnDegrees = 180;

        // Define the ignition order
        ignitionStruct[0].next_schedule = 1;
        ignitionStruct[1].next_schedule = 0;

        ignitionStructEnd = 1;

        if(configPage2.sparkMode == IGN_MODE_SEQUENTIAL)
        {
          channel3IgnDegrees = 360;
          channel4IgnDegrees = 540;

          CRANK_ANGLE_MAX_IGN = 720;
          maxIgnOutputs = 4;

          // Define the ignition order
          ignitionStruct[0].next_schedule = 1;
          ignitionStruct[1].next_schedule = 2;
          ignitionStruct[2].next_schedule = 3;
          ignitionStruct[3].next_schedule = 0;

          // 0 to 3 = 4 would be more intuitive to used 4 but this works
          injectorStructEnd = maxIgnOutputs - 1;
        }
        else if(configPage2.sparkMode == IGN_MODE_ROTARY)
        {
          //Rotary uses the ign 3 and 4 schedules for the trailing spark. They are offset from the ign 1 and 2 channels respectively and so use the same degrees as them
          channel3IgnDegrees = 0;
          channel4IgnDegrees = 180;

          // Define the ignition order???? - need to be added, the same count for 1, 2, 3 and 5 cylinder
        }
      }
      else
      {
        channel2IgnDegrees = configPage1.oddfire2;
        channel3IgnDegrees = configPage1.oddfire3;
        channel4IgnDegrees = configPage1.oddfire4;
        maxIgnOutputs = 4;
      }
      break;

    case 5:
      channel1IgnDegrees = 0;
      channel2IgnDegrees = 72;
      channel3IgnDegrees = 144;
      channel4IgnDegrees = 216;
      channel5IgnDegrees = 288;
      maxIgnOutputs = 4; //Only 4 actual outputs, so that's all that can be cut

      if(configPage2.sparkMode == IGN_MODE_SEQUENTIAL)
      {
        channel2IgnDegrees = 144;
        channel3IgnDegrees = 288;
        channel4IgnDegrees = 432;
        channel5IgnDegrees = 576;
        CRANK_ANGLE_MAX_IGN = 720;
      }
      break;

    case 6:
      channel1IgnDegrees = 0;
      channel2IgnDegrees = 120;
      channel3IgnDegrees = 240;
      maxIgnOutputs = 3;
      break;

    case 8:
      channel1IgnDegrees = 0;
      channel2IgnDegrees = 90;
      channel3IgnDegrees = 180;
      channel4IgnDegrees = 270;
      maxIgnOutputs = 4;
      break;

    default: //Handle this better!!!
      channel1IgnDegrees = 0;
      channel2IgnDegrees = 180;
      break;
  }

  switch(configPage2.sparkMode)
  {
    case IGN_MODE_WASTED:
      //Wasted Spark (Normal mode)
      ign1StartFunction = beginCoil1Charge;
      ign1EndFunction = endCoil1Charge;
      ign2StartFunction = beginCoil2Charge;
      ign2EndFunction = endCoil2Charge;
      ign3StartFunction = beginCoil3Charge;
      ign3EndFunction = endCoil3Charge;
      ign4StartFunction = beginCoil4Charge;
      ign4EndFunction = endCoil4Charge;
      ign5StartFunction = beginCoil5Charge;
      ign5EndFunction = endCoil5Charge;
      break;

    case IGN_MODE_SINGLE:
      //Single channel mode. All ignition pulses are on channel 1
      ign1StartFunction = beginCoil1Charge;
      ign1EndFunction = endCoil1Charge;
      ign2StartFunction = beginCoil1Charge;
      ign2EndFunction = endCoil1Charge;
      ign3StartFunction = beginCoil1Charge;
      ign3EndFunction = endCoil1Charge;
      ign4StartFunction = beginCoil1Charge;
      ign4EndFunction = endCoil1Charge;
      ign5StartFunction = beginCoil1Charge;
      ign5EndFunction = endCoil1Charge;
      break;

    case IGN_MODE_WASTEDCOP:
      //Wasted COP mode. Ignition channels 1&3 and 2&4 are paired together
      //This is not a valid mode for >4 cylinders
      if( configPage1.nCylinders <= 4 )
      {
        ign1StartFunction = beginCoil1and3Charge;
        ign1EndFunction = endCoil1and3Charge;
        ign2StartFunction = beginCoil2and4Charge;
        ign2EndFunction = endCoil2and4Charge;

        ign3StartFunction = nullCallback;
        ign3EndFunction = nullCallback;
        ign4StartFunction = nullCallback;
        ign4EndFunction = nullCallback;
      }
      else
      {
        //If the person has inadvertantly selected this when running more than 4 cylinders, just use standard Wasted spark mode
        ign1StartFunction = beginCoil1Charge;
        ign1EndFunction = endCoil1Charge;
        ign2StartFunction = beginCoil2Charge;
        ign2EndFunction = endCoil2Charge;
        ign3StartFunction = beginCoil3Charge;
        ign3EndFunction = endCoil3Charge;
        ign4StartFunction = beginCoil4Charge;
        ign4EndFunction = endCoil4Charge;
        ign5StartFunction = beginCoil5Charge;
        ign5EndFunction = endCoil5Charge;
      }
      break;

    case IGN_MODE_SEQUENTIAL:
      ign1StartFunction = beginCoil1Charge;
      ign1EndFunction = endCoil1Charge;
      ign2StartFunction = beginCoil2Charge;
      ign2EndFunction = endCoil2Charge;
      ign3StartFunction = beginCoil3Charge;
      ign3EndFunction = endCoil3Charge;
      ign4StartFunction = beginCoil4Charge;
      ign4EndFunction = endCoil4Charge;
      ign5StartFunction = beginCoil5Charge;
      ign5EndFunction = endCoil5Charge;
      break;

    case IGN_MODE_ROTARY:
      if(configPage11.rotaryType == ROTARY_IGN_FC)
      {
        ign1StartFunction = beginCoil1Charge;
        ign1EndFunction = endCoil1Charge;
        ign2StartFunction = beginCoil1Charge;
        ign2EndFunction = endCoil1Charge;

        ign3StartFunction = beginTrailingCoilCharge;
        ign3EndFunction = endTrailingCoilCharge1;
        ign4StartFunction = beginTrailingCoilCharge;
        ign4EndFunction = endTrailingCoilCharge2;
      }
      break;

    default:
      //Wasted spark (Shouldn't ever happen anyway)
      ign1StartFunction = beginCoil1Charge;
      ign1EndFunction = endCoil1Charge;
      ign2StartFunction = beginCoil2Charge;
      ign2EndFunction = endCoil2Charge;
      ign3StartFunction = beginCoil3Charge;
      ign3EndFunction = endCoil3Charge;
      ign4StartFunction = beginCoil4Charge;
      ign4EndFunction = endCoil4Charge;
      ign5StartFunction = beginCoil5Charge;
      ign5EndFunction = endCoil5Charge;
      break;
  }
}

//*********************************************************************************************************
// Do the ignition calculations
//*********************************************************************************************************
void ignition_calc( int crankAngle, int timePerDegree, bool ignitionOn)
{
  int8_t advance;

  ignition1StartAngle = 0;
  ignition2StartAngle = 0;
  ignition3StartAngle = 0;
  ignition4StartAngle = 0;
  ignition5StartAngle = 0;

  //These are used for comparisons on channels above 1 where the starting angle (for injectors or ignition) can be less than a single loop time
  int tempCrankAngle;
  int tempStartAngle;

  lastAdvance = currentStatus.advance; //Store the previous advance value

  //Check which fuelling algorithm is being used - the same is used for ignition advance
  if (configPage1.algorithm == LOAD_SOURCE_MAP)
  {
    //Speed Density
    currentStatus.advance = get3DTableValue(&ignitionTable, currentStatus.MAP, currentStatus.RPM) - OFFSET_IGNITION; //As above, but for ignition advance
  }
  else
  {
    //Alpha-N
    currentStatus.advance = get3DTableValue(&ignitionTable, currentStatus.TPS, currentStatus.RPM) - OFFSET_IGNITION; //As above, but for ignition advance
  }

  // Calculate advance corrections
  advance = correctionFlexTiming(currentStatus.advance);
  advance = correctionIATretard(advance);
  advance = correctionSoftRevLimit(advance);
  advance = correctionSoftLaunch(advance);
  advance = correctionSoftFlatShift(advance);

  //Fixed timing check must go last
  advance = correctionFixedTiming(advance);
  advance = correctionCrankingFixedTiming(advance); //This overrrides the regular fixed timing, must come last

  currentStatus.advance = advance;

  //***********************************************************************************************
  //| BEGIN IGNITION CALCULATIONS
  if (currentStatus.RPM > ((unsigned int)(configPage2.HardRevLim) * 100) ) { BIT_SET(currentStatus.spark, BIT_SPARK_HRDLIM); } //Hardcut RPM limit
  else { BIT_CLEAR(currentStatus.spark, BIT_SPARK_HRDLIM); }


  //Set dwell
   //Dwell is stored as ms * 10. ie Dwell of 4.3ms would be 43 in configPage2. This number therefore needs to be multiplied by 100 to get dwell in uS
  if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) ) { currentStatus.dwell =  (configPage2.dwellCrank * 100); }
  else { currentStatus.dwell =  (configPage2.dwellRun * 100); }
  currentStatus.dwell = correctionsDwell(currentStatus.dwell);

  int dwellAngle = uSToDegrees(currentStatus.dwell); //Convert the dwell time to dwell angle based on the current engine speed

  //Calculate start angle for each channel
  //1 cylinder (Everyone gets this)
  ignition1EndAngle = CRANK_ANGLE_MAX_IGN - currentStatus.advance;
  ignition1StartAngle = ignition1EndAngle - dwellAngle; // 360 - desired advance angle - number of degrees the dwell will take
  if(ignition1StartAngle < 0) {ignition1StartAngle += CRANK_ANGLE_MAX_IGN;}

  //This test for more cylinders and do the same thing
  switch (configPage1.nCylinders)
  {
    //2 cylinders
    case 2:
      ignition2EndAngle = channel2IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition2StartAngle = ignition2EndAngle - dwellAngle;
      if(ignition2StartAngle > CRANK_ANGLE_MAX_IGN) {ignition2StartAngle -= CRANK_ANGLE_MAX_IGN;}
      break;
    //3 cylinders
    case 3:
      ignition2EndAngle = channel2IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition2StartAngle = ignition2EndAngle - dwellAngle;
      if(ignition2StartAngle > CRANK_ANGLE_MAX_IGN) {ignition2StartAngle -= CRANK_ANGLE_MAX_IGN;}
      ignition3EndAngle = channel3IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition3StartAngle = channel3IgnDegrees + 360 - currentStatus.advance - dwellAngle;
      if(ignition3StartAngle > CRANK_ANGLE_MAX_IGN) {ignition3StartAngle -= CRANK_ANGLE_MAX_IGN;}
      break;
    //4 cylinders
    case 4:
      ignition2EndAngle = channel2IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition2StartAngle = ignition2EndAngle - dwellAngle;
      if(ignition2StartAngle > CRANK_ANGLE_MAX_IGN) {ignition2StartAngle -= CRANK_ANGLE_MAX_IGN;}
      if(ignition2StartAngle < 0) {ignition2StartAngle += CRANK_ANGLE_MAX_IGN;}

      if(configPage2.sparkMode == IGN_MODE_SEQUENTIAL)
      {
        ignition3EndAngle = channel3IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
        ignition3StartAngle = ignition3EndAngle - dwellAngle;
        if(ignition3StartAngle > CRANK_ANGLE_MAX_IGN) {ignition3StartAngle -= CRANK_ANGLE_MAX_IGN;}

        ignition4EndAngle = channel4IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
        ignition4StartAngle = ignition4EndAngle - dwellAngle;
        if(ignition4StartAngle > CRANK_ANGLE_MAX_IGN) {ignition4StartAngle -= CRANK_ANGLE_MAX_IGN;}
      }
      else if(configPage2.sparkMode == IGN_MODE_ROTARY)
      {
        if(configPage11.rotaryType == ROTARY_IGN_FC)
        {
          byte splitDegrees = 0;
          if (configPage1.algorithm == LOAD_SOURCE_MAP) { splitDegrees = table2D_getValue(&rotarySplitTable, currentStatus.MAP/2); }
          else { splitDegrees = table2D_getValue(&rotarySplitTable, currentStatus.TPS/2); }

          //The trailing angles are set relative to the leading ones
          ignition3EndAngle = ignition1EndAngle + splitDegrees;
          ignition3StartAngle = ignition3EndAngle - dwellAngle;
          if(ignition3StartAngle > CRANK_ANGLE_MAX_IGN) {ignition3StartAngle -= CRANK_ANGLE_MAX_IGN;}
          if(ignition3StartAngle < 0) {ignition3StartAngle += CRANK_ANGLE_MAX_IGN;}

          ignition4EndAngle = ignition2EndAngle + splitDegrees;
          ignition4StartAngle = ignition4EndAngle - dwellAngle;
          if(ignition4StartAngle > CRANK_ANGLE_MAX_IGN) {ignition4StartAngle -= CRANK_ANGLE_MAX_IGN;}
          if(ignition4StartAngle < 0) {ignition4StartAngle += CRANK_ANGLE_MAX_IGN;}
        }
      }
      break;
    //5 cylinders
    case 5:
      ignition2EndAngle = channel2IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition2StartAngle = ignition2EndAngle - dwellAngle;
      if(ignition2StartAngle > CRANK_ANGLE_MAX_IGN) {ignition2StartAngle -= CRANK_ANGLE_MAX_IGN;}
      if(ignition2StartAngle < 0) {ignition2StartAngle += CRANK_ANGLE_MAX_IGN;}

      ignition3EndAngle = channel3IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition3StartAngle = ignition3EndAngle - dwellAngle;
      if(ignition3StartAngle > CRANK_ANGLE_MAX_IGN) {ignition3StartAngle -= CRANK_ANGLE_MAX_IGN;}

      ignition4EndAngle = channel4IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition4StartAngle = ignition4EndAngle - dwellAngle;
      if(ignition4StartAngle > CRANK_ANGLE_MAX_IGN) {ignition4StartAngle -= CRANK_ANGLE_MAX_IGN;}

      ignition5StartAngle = channel5IgnDegrees + CRANK_ANGLE_MAX - currentStatus.advance - dwellAngle;
      if(ignition5StartAngle > CRANK_ANGLE_MAX_IGN) {ignition5StartAngle -= CRANK_ANGLE_MAX_IGN;}

      break;
    //6 cylinders
    case 6:
      ignition2EndAngle = channel2IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition2StartAngle = ignition2EndAngle - dwellAngle;
      if(ignition2StartAngle > CRANK_ANGLE_MAX_IGN) {ignition2StartAngle -= CRANK_ANGLE_MAX_IGN;}

      ignition3EndAngle = channel3IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition3StartAngle = ignition3EndAngle - dwellAngle;
      if(ignition3StartAngle > CRANK_ANGLE_MAX_IGN) {ignition3StartAngle -= CRANK_ANGLE_MAX_IGN;}
      break;
    //8 cylinders
    case 8:
      ignition2EndAngle = channel2IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition2StartAngle = ignition2EndAngle - dwellAngle;
      if(ignition2StartAngle > CRANK_ANGLE_MAX_IGN) {ignition2StartAngle -= CRANK_ANGLE_MAX_IGN;}

      ignition3EndAngle = channel3IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition3StartAngle = ignition3EndAngle - dwellAngle;
      if(ignition3StartAngle > CRANK_ANGLE_MAX_IGN) {ignition3StartAngle -= CRANK_ANGLE_MAX_IGN;}

      ignition4EndAngle = channel4IgnDegrees + CRANK_ANGLE_MAX_IGN - currentStatus.advance;
      ignition4StartAngle = ignition4EndAngle - dwellAngle;
      if(ignition4StartAngle > CRANK_ANGLE_MAX_IGN) {ignition4StartAngle -= CRANK_ANGLE_MAX_IGN;}
      break;

    //Will hit the default case on 1 cylinder or >8 cylinders. Do nothing in these cases
    default:
      break;
  }

  //fixedCrankingOverride is used to extend the dwell during cranking so that the decoder can trigger the spark upon seeing a certain tooth. Currently only available on the basic distributor and 4g63 decoders.
  if ( configPage2.ignCranklock && BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) { fixedCrankingOverride = currentStatus.dwell * 3; }
  else { fixedCrankingOverride = 0; }

  //Perform an initial check to see if the ignition is turned on (Ignition only turns on after a preset number of cranking revolutions and:
  //Check for any of the hard cut rev limits being on
  if(currentStatus.launchingHard || BIT_CHECK(currentStatus.spark, BIT_SPARK_BOOSTCUT) || BIT_CHECK(currentStatus.spark, BIT_SPARK_HRDLIM) || currentStatus.flatShiftingHard)
  {
    if(configPage1.hardCutType == HARD_CUT_FULL) { ignitionOn = false; }
    else { curRollingCut = ( (currentStatus.startRevolutions / 2) % maxIgnOutputs) + 1; } //Rolls through each of the active ignition channels based on how many revolutions have taken place
  }
  else { curRollingCut = 0; } //Disables the rolling hard cut

  // New code for tooth based injection schedules - added Jan. 2018
  // The PSOC5 code schedule is done in decoders.ino - here we only add the needed data to the arrays
#if defined(CORE_PSOC5)
  if (( configPage2.TrigPattern == 0 || configPage2.TrigPattern == 16 ) && configPage1.perToothIgn == true ) // Use the new code / schedule if on the PSOC5 platform
#else
  if ( 1 != 1 ) // Use the old schedule for all other platforms
#endif
  {
    // Here we calculate the start tooth and remanding time before we start the dwell periode and fire the ignition

    if(ignitionOn)
    {
      // We always have one ignition coil
      if (curRollingCut != 1 )
      {
        ignitionToothScheduleCalc( currentStatus.dwell, channel1IgnDegrees, timePerDegree, triggerToothAngle, CRANK_ANGLE_MAX_IGN, 0 );
      }
      else
      {
        ignitionStruct[0].pulse_width_uS = 0;
      }

      if ( maxIgnOutputs > 1 && (curRollingCut != 2) )
      {
        ignitionToothScheduleCalc( currentStatus.dwell, channel2IgnDegrees, timePerDegree, triggerToothAngle, CRANK_ANGLE_MAX_IGN, 1 );
      }
      else
      {
        ignitionStruct[1].pulse_width_uS = 0;
      }

      if ( maxIgnOutputs > 2 && (curRollingCut != 3) )
      {
        ignitionToothScheduleCalc( currentStatus.dwell, channel3IgnDegrees, timePerDegree, triggerToothAngle, CRANK_ANGLE_MAX_IGN, 2 );
      }
      else
      {
        ignitionStruct[2].pulse_width_uS = 0;
      }

      if ( maxIgnOutputs > 3 && (curRollingCut != 4) )
      {
        ignitionToothScheduleCalc( currentStatus.dwell, channel4IgnDegrees, timePerDegree, triggerToothAngle, CRANK_ANGLE_MAX_IGN, 3 );
      }
      else
      {
        ignitionStruct[3].pulse_width_uS = 0;
      }

      // Set the array number that we have to start at after tooth 1 when running scheduling
      if ( ignitionStruct[0].start_tooth < ignitionStruct[ignitionStructEnd].start_tooth || ignitionStructEnd == 0 )
      {
        ignitionStructStart = 0;
      }
      else
      {
        ignitionStructStart = 1;
      }

      // PSOC5 implementation current don't support 5 ignition coils
    }
    else
    {
      ignitionStruct[0].pulse_width_uS = 0;
      ignitionStruct[1].pulse_width_uS = 0;
      ignitionStruct[2].pulse_width_uS = 0;
      ignitionStruct[3].pulse_width_uS = 0;
    }
  }
  else // Beging old schedule
  {
    //If ignition timing is being tracked per tooth, perform the calcs to get the end teeth
    //This only needs to be run if the advance figure has changed, otherwise the end teeth will still be the same
    if( (configPage1.perToothIgn == true) && (lastAdvance != currentStatus.advance) ) { triggerSetEndTeeth(); }

    //***********************************************************************************************
    // BEGIN IGNITION SCHEDULES
    //***********************************************************************************************

    if (crankAngle > CRANK_ANGLE_MAX_IGN ) { crankAngle -= 360; }

    //if(ignitionOn && !currentStatus.launchingHard && !BIT_CHECK(currentStatus.spark, BIT_SPARK_BOOSTCUT) && !BIT_CHECK(currentStatus.spark, BIT_SPARK_HRDLIM) && !currentStatus.flatShiftingHard)
    if(ignitionOn)
    {

      if ( (ignition1StartAngle > crankAngle) && (curRollingCut != 1) )
      {
          /*
          long some_time = ((unsigned long)(ignition1StartAngle - crankAngle) * (unsigned long)timePerDegree);
          long newRPM = (long)(some_time * currentStatus.rpmDOT) / 1000000L;
          newRPM = currentStatus.RPM + (newRPM/2);
          unsigned long timePerDegree_1 = ldiv( 166666L, newRPM).quot;
          unsigned long timeout = (unsigned long)(ignition1StartAngle - crankAngle) * 282UL;
          */
          setIgnitionSchedule1(ign1StartFunction,
                    //((unsigned long)(ignition1StartAngle - crankAngle) * (unsigned long)timePerDegree),
                    degreesToUS((ignition1StartAngle - crankAngle)),
                    currentStatus.dwell + fixedCrankingOverride, //((unsigned long)((unsigned long)currentStatus.dwell* currentStatus.RPM) / newRPM) + fixedCrankingOverride,
                    ign1EndFunction
                    );
      }

      tempCrankAngle = crankAngle - channel2IgnDegrees;
      if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
      tempStartAngle = ignition2StartAngle - channel2IgnDegrees;
      if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
      {
          unsigned long ignition2StartTime = 0;
          if(tempStartAngle > tempCrankAngle) { ignition2StartTime = degreesToUS((tempStartAngle - tempCrankAngle)); }
          //else if (tempStartAngle < tempCrankAngle) { ignition2StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
          else { ignition2StartTime = 0; }

          if( (ignition2StartTime > 0) && (curRollingCut != 2) )
          {
            setIgnitionSchedule2(ign2StartFunction,
                      ignition2StartTime,
                      currentStatus.dwell + fixedCrankingOverride,
                      ign2EndFunction
                      );
          }
      }

      tempCrankAngle = crankAngle - channel3IgnDegrees;
      if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
      tempStartAngle = ignition3StartAngle - channel3IgnDegrees;
      if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
      //if (tempStartAngle > tempCrankAngle)
      {
          long ignition3StartTime = 0;
          if(tempStartAngle > tempCrankAngle) { ignition3StartTime = degreesToUS((tempStartAngle - tempCrankAngle)); }
          //else if (tempStartAngle < tempCrankAngle) { ignition4StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
          else { ignition3StartTime = 0; }

          if( (ignition3StartTime > 0) && (curRollingCut != 3) )
          {
            setIgnitionSchedule3(ign3StartFunction,
                      ignition3StartTime,
                      currentStatus.dwell + fixedCrankingOverride,
                      ign3EndFunction
                      );
          }
      }

      tempCrankAngle = crankAngle - channel4IgnDegrees;
      if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
      tempStartAngle = ignition4StartAngle - channel4IgnDegrees;
      if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
      //if (tempStartAngle > tempCrankAngle)
      {

          long ignition4StartTime = 0;
          if(tempStartAngle > tempCrankAngle) { ignition4StartTime = degreesToUS((tempStartAngle - tempCrankAngle)); }
          //else if (tempStartAngle < tempCrankAngle) { ignition4StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
          else { ignition4StartTime = 0; }

          if( (ignition4StartTime > 0) && (curRollingCut != 4) )
          {
            setIgnitionSchedule4(ign4StartFunction,
                      ignition4StartTime,
                      currentStatus.dwell + fixedCrankingOverride,
                      ign4EndFunction
                      );
          }
      }

      tempCrankAngle = crankAngle - channel5IgnDegrees;
      if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_IGN; }
      tempStartAngle = ignition5StartAngle - channel5IgnDegrees;
      if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_IGN; }
      //if (tempStartAngle > tempCrankAngle)
      {
          long ignition5StartTime = 0;
          if(tempStartAngle > tempCrankAngle) { ignition5StartTime = degreesToUS((tempStartAngle - tempCrankAngle)); }
          //else if (tempStartAngle < tempCrankAngle) { ignition4StartTime = ((long)(360 - tempCrankAngle + tempStartAngle) * (long)timePerDegree); }
          else { ignition5StartTime = 0; }

          if(ignition5StartTime > 0) {
          setIgnitionSchedule5(ign5StartFunction,
                    ignition5StartTime,
                    currentStatus.dwell + fixedCrankingOverride,
                    ign5EndFunction
                    );
          }
      }
    } //Ignition schedules on
  } // End old schedule code
} // End ignition_calc

static inline int8_t correctionFixedTiming(int8_t advance)
{
  byte ignFixValue = advance;
  if (configPage2.FixAng != 0) { ignFixValue = configPage2.FixAng; } //Check whether the user has set a fixed timing angle
  return ignFixValue;
}

static inline int8_t correctionCrankingFixedTiming(int8_t advance)
{
  byte ignCrankFixValue = advance;
  if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) ) { ignCrankFixValue = configPage2.CrankAng; } //Use the fixed cranking ignition angle
  return ignCrankFixValue;
}

static inline int8_t correctionFlexTiming(int8_t advance)
{
  byte ignFlexValue = advance;
  if( configPage1.flexEnabled == 1 ) //Check for flex being enabled
  {
    byte flexRange = configPage1.flexAdvHigh - configPage1.flexAdvLow;

    if (currentStatus.ethanolPct != 0) { currentStatus.flexIgnCorrection = percentage(currentStatus.ethanolPct, flexRange); }
    else { currentStatus.flexIgnCorrection = 0; }

    ignFlexValue = advance + currentStatus.flexIgnCorrection;
  }
  return ignFlexValue;
}

static inline int8_t correctionIATretard(int8_t advance)
{
  byte ignIATValue = advance;
  //Adjust the advance based on IAT. If the adjustment amount is greater than the current advance, just set advance to 0
  int8_t advanceIATadjust = table2D_getValue(&IATRetardTable, currentStatus.IAT);
  int tempAdvance = (advance - advanceIATadjust);
  if (tempAdvance >= -OFFSET_IGNITION) { ignIATValue = tempAdvance; }
  else { ignIATValue = -OFFSET_IGNITION; }

  return ignIATValue;
}

static inline int8_t correctionSoftRevLimit(int8_t advance)
{
  byte ignSoftRevValue = advance;
  BIT_CLEAR(currentStatus.spark, BIT_SPARK_SFTLIM);
  if (currentStatus.RPM > ((unsigned int)(configPage2.SoftRevLim) * 100) ) { BIT_SET(currentStatus.spark, BIT_SPARK_SFTLIM); ignSoftRevValue = configPage2.SoftLimRetard;  } //Softcut RPM limit (If we're above softcut limit, delay timing by configured number of degrees)

  return ignSoftRevValue;
}

static inline int8_t correctionSoftLaunch(int8_t advance)
{
  byte ignSoftLaunchValue = advance;
  //SoftCut rev limit for 2-step launch control.
  if (configPage3.launchEnabled && clutchTrigger && (currentStatus.clutchEngagedRPM < ((unsigned int)(configPage3.flatSArm) * 100)) && (currentStatus.RPM > ((unsigned int)(configPage3.lnchSoftLim) * 100)) )
  {
    currentStatus.launchingSoft = true;
    BIT_SET(currentStatus.spark, BIT_SPARK_SLAUNCH);
    ignSoftLaunchValue = configPage3.lnchRetard;
  }
  else
  {
    currentStatus.launchingSoft = false;
    BIT_CLEAR(currentStatus.spark, BIT_SPARK_SLAUNCH);
  }

  return ignSoftLaunchValue;
}

static inline int8_t correctionSoftFlatShift(int8_t  advance)
{
  byte ignSoftFlatValue = advance;

  if(configPage3.flatSEnable && clutchTrigger && (currentStatus.clutchEngagedRPM > ((unsigned int)(configPage3.flatSArm) * 100)) && (currentStatus.RPM > (currentStatus.clutchEngagedRPM-configPage3.flatSSoftWin) ) )
  {
    BIT_SET(currentStatus.spark2, BIT_SPARK2_FLATSS);
    ignSoftFlatValue = configPage3.flatSRetard;
  }
  else { BIT_CLEAR(currentStatus.spark2, BIT_SPARK2_FLATSS); }

  return ignSoftFlatValue;
}

//******************************** DWELL CORRECTIONS ********************************
uint16_t correctionsDwell(uint16_t dwell)
{
  uint16_t tempDwell = dwell;
  //Pull battery voltage based dwell correction and apply if needed
  currentStatus.dwellCorrection = table2D_getValue(&dwellVCorrectionTable, currentStatus.battery10);
  if (currentStatus.dwellCorrection != 100) { tempDwell = divs100(dwell) * currentStatus.dwellCorrection; }

  //Dwell limiter
  uint16_t dwellPerRevolution = tempDwell + (uint16_t)(configPage2.sparkDur * 100); //Spark duration is in mS*10. Multiple it by 100 to get spark duration in uS
  int8_t pulsesPerRevolution = 1;
  //Single channel spark mode is the only time there will be more than 1 pulse per revolution on any given output
  if( ( (configPage2.sparkMode == IGN_MODE_SINGLE) || (configPage2.sparkMode == IGN_MODE_ROTARY) ) && (configPage1.nCylinders > 1) ) //No point in running this for 1 cylinder engines
  {
    pulsesPerRevolution = (configPage1.nCylinders >> 1);
    dwellPerRevolution = dwellPerRevolution * pulsesPerRevolution;
  }

  if(dwellPerRevolution > revolutionTime)
  {
    //Possibly need some method of reducing spark duration here as well, but this is a start
    tempDwell = (revolutionTime / pulsesPerRevolution) - (configPage2.sparkDur * 100);
  }
  return tempDwell;
}

// Calculate the tooth and timer data required to schedule the ignition
void ignitionToothScheduleCalc( int itscDwell, int itscIgnDegrees, int itscTimePerDegree, int itscTriggerToothAngle, int itscCRANK_ANGLE_MAX_IGN, uint8_t itscIgnitionPortMinusOne )
{
  // Temp's used
  int itscTempStartTooth;
  unsigned int itscTempStartDelayuS;
  unsigned int itscTempTotalTimeuS;

  // We have to extract the dwell time from the defined start time to get the real start time
  itscIgnDegrees -= (( itscDwell ) / itscTimePerDegree );

  // We can't start at a negative angel, use the next comming time that fit
  if ( itscIgnDegrees  < 0 ) { itscIgnDegrees += itscCRANK_ANGLE_MAX_IGN; }

  // UPS, we are to far ahead
  if ( itscIgnDegrees > itscCRANK_ANGLE_MAX_IGN ) { itscIgnDegrees -= itscCRANK_ANGLE_MAX_IGN; }

  // Calculate the tooth number where we should start the fuel timer
  itscTempStartTooth = itscIgnDegrees / itscTriggerToothAngle;

  // Calculate the time we should wait before the injection pulse is started after we have reached the wanted tooth
  itscTempStartDelayuS = (unsigned int) (( itscIgnDegrees % itscTriggerToothAngle ) * itscTimePerDegree );

  // We need some time before the pulse should be fired otherwise we may see issues in the timing
  if ( itscTempStartDelayuS < 10 )
  {
    itscTempStartDelayuS += ( itscTriggerToothAngle * itscTimePerDegree );
    itscTempStartTooth--;
  }

  // Scheduler is set to start at next tooth - so to save time in the interupt by extracting 1 here.
  itscTempStartTooth--;

  // If below 1 then we add the number of teeths on the wheel to get a positive number
  if ( itscTempStartTooth < 1 ) { itscTempStartTooth += scheduling_max_tooth_count; }

  itscTempTotalTimeuS = itscTempStartDelayuS + itscDwell;

  // Copy to ignition structure - precalculated temp's used to speed the update time up
  ignitionStruct[itscIgnitionPortMinusOne].start_tooth = (unsigned int) itscTempStartTooth;
  ignitionStruct[itscIgnitionPortMinusOne].start_delay_uS = itscTempStartDelayuS;
  ignitionStruct[itscIgnitionPortMinusOne].total_timer_time_uS = itscTempTotalTimeuS;
  ignitionStruct[itscIgnitionPortMinusOne].pulse_width_uS = itscDwell;
}
