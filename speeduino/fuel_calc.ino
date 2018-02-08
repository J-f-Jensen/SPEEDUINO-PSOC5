//*********************************************************************************************************
// All fuel calculations should be done in this file
//*********************************************************************************************************
#include "fuel_calc.h"
#include "globals.h"

#include "scheduler.h"

// Private/internal functions
static inline byte correctionWUE(); //Warmup enrichment
static inline byte correctionCranking(); //Cranking enrichment
static inline byte correctionASE(); //After Start Enrichment
static inline byte correctionAccel(); //Acceleration Enrichment
static inline byte correctionFloodClear(); //Check for flood clear on cranking
static inline byte correctionAFRClosedLoop(); //Closed loop AFR adjustment
static inline byte correctionFlex(); //Flex fuel adjustment
static inline byte correctionBatVoltage(); //Battery voltage correction
static inline byte correctionIATDensity(); //Inlet temp density correction
static inline byte correctionLaunch(); //Launch control correction
static inline bool correctionDFCO(); //Decelleration fuel cutoff
void injectorToothScheduleCalc( int injPulseWidth, int injOpenTime, int injDegrees, int injTimePerDegree, int injTriggerToothAngle, int injCRANK_ANGLE_MAX, uint8_t injInjectorPortMinusOne );

int CRANK_ANGLE_MAX_INJ = 360; // The number of crank degrees that the system track over. 360 for wasted / timed batch and 720 for sequential

int injector1StartAngle = 0;
int injector2StartAngle = 0;
int injector3StartAngle = 0;
int injector4StartAngle = 0;
int injector5StartAngle = 0;

int channel1InjDegrees; //The number of crank degrees until cylinder 1 is at TDC (This is obviously 0 for virtually ALL engines, but there's some weird ones)
int channel2InjDegrees; //The number of crank degrees until cylinder 2 (and 5/6/7/8) is at TDC
int channel3InjDegrees; //The number of crank degrees until cylinder 3 (and 5/6/7/8) is at TDC
int channel4InjDegrees; //The number of crank degrees until cylinder 4 (and 5/6/7/8) is at TDC
int channel5InjDegrees; //The number of crank degrees until cylinder 5 is at TDC

int req_fuel_uS;
int inj_opentime_uS;

long PID_O2, PID_output, PID_AFRTarget;
PID egoPID(&PID_O2, &PID_output, &PID_AFRTarget, configPage3.egoKP, configPage3.egoKI, configPage3.egoKD, REVERSE); //This is the PID object if that algorithm is used. Needs to be global as it maintains state outside of each function call

//*********************************************************************************************************
// Initilisation of stuff used for fuel calculations
//*********************************************************************************************************
void fuel_calc_init( void )
{
  egoPID.SetMode(AUTOMATIC); //Turn O2 PID on

  //Once the configs have been loaded, a number of one time calculations can be completed
  req_fuel_uS = configPage1.reqFuel * 100; //Convert to uS and an int. This is the only variable to be used in calculations

  //Why is the engineSquirtsPerCycle hardcoded in speeduino.ino, is it not a value that tunerStudio deliver?
  req_fuel_uS = req_fuel_uS / engineSquirtsPerCycle; //The req_fuel calculation above gives the total required fuel (At VE 100%) in the full cycle. If we're doing more than 1 squirt per cycle then we need to split the amount accordingly. (Note that in a non-sequential 4-stroke setup you cannot have less than 2 squirts as you cannot determine the stroke to make the single squirt on)

  //Forcing all injector close angles to be the same if individual angel not is defined
  if(!configPage1.indInjAng) {configPage1.inj4Ang = configPage1.inj3Ang = configPage1.inj2Ang = configPage1.inj1Ang;}

  //Initial injector open time in uS. Comes through as ms*10 (Eg 15.5ms = 155)
  inj_opentime_uS = configPage1.injOpen * 100;

  //Calculate the number of degrees between cylinders
  switch (configPage1.nCylinders) {
    case 1:
      channel1InjDegrees = 0;

      channel1InjEnabled = true;

      // Define the injection arrays - one for each injectore
      injectorStruct[0].next_schedule = 0;
      injectorStructEnd = 1;
      break;

    case 2:
      //For alternating injection, the squirt occurs at different times for each channel
      if(configPage1.injLayout == INJ_SEMISEQUENTIAL)
      {
        channel1InjDegrees = 0;

        if (configPage1.engineType == EVEN_FIRE )
        {
          channel2InjDegrees = 180;
        }
        else { channel2InjDegrees = configPage1.oddfire2; }
      }
      if (!configPage1.injTiming) { channel1InjDegrees = channel2InjDegrees = 0; } //For simultaneous, all squirts happen at the same time

      channel1InjEnabled = true;
      channel2InjEnabled = true;

      // Define the injection arrays - one for each injectore
      injectorStruct[0].next_schedule = 1;
      injectorStruct[1].next_schedule = 0;
      injectorStructEnd = 1;

      break;

    case 3:
      //For alternatiing injection, the squirt occurs at different times for each channel
      if(configPage1.injLayout == INJ_SEMISEQUENTIAL  || configPage1.injLayout == INJ_PAIRED)
      {
        channel1InjDegrees = 0;
        channel2InjDegrees = 120;
        channel3InjDegrees = 240;
      }
      else if (configPage1.injLayout == INJ_SEQUENTIAL)
      {
        channel1InjDegrees = 0;
        channel2InjDegrees = 240;
        channel3InjDegrees = 480;
        CRANK_ANGLE_MAX_INJ = 720;
        req_fuel_uS = req_fuel_uS * 2;
      }
      if (!configPage1.injTiming) { channel1InjDegrees = channel2InjDegrees = channel3InjDegrees = 0; } //For simultaneous, all squirts happen at the same time

      channel1InjEnabled = true;
      channel2InjEnabled = true;
      channel3InjEnabled = true;

      // Define the injection arrays - one for each injectore
      injectorStruct[0].next_schedule = 1;
      injectorStruct[1].next_schedule = 2;
      injectorStruct[2].next_schedule = 0;
      injectorStructEnd = 2;
      break;

    case 4:
      //For alternatiing injection, the squirt occurs at different times for each channel
      if(configPage1.injLayout == INJ_SEMISEQUENTIAL || configPage1.injLayout == INJ_PAIRED )
      {
        channel1InjDegrees = 0;
        channel2InjDegrees = 180;

        // Define the injection arrays - one for each injectore
        injectorStruct[0].next_schedule = 1;
        injectorStruct[1].next_schedule = 0;
        injectorStructEnd = 1; // Max array id
      }
      else if (configPage1.injLayout == INJ_SEQUENTIAL)
      {
        channel1InjDegrees = 0;
        channel2InjDegrees = 180;
        channel3InjDegrees = 360;
        channel4InjDegrees = 540;

        // Define the injection arrays - one for each injectore
        injectorStruct[0].next_schedule = 1; //Not 3 as we start at 0
        injectorStruct[1].next_schedule = 2;
        injectorStruct[2].next_schedule = 3;
        injectorStruct[3].next_schedule = 1;
        injectorStructEnd = 3;

        channel3InjEnabled = true;
        channel4InjEnabled = true;

        CRANK_ANGLE_MAX_INJ = 720;
        req_fuel_uS = req_fuel_uS * 2;
      }
      if (!configPage1.injTiming) { channel1InjDegrees = channel2InjDegrees = 0; } //For simultaneous, all squirts happen at the same time

      channel1InjEnabled = true;

      //Current Siamese only support semisequential, not shure this ever will change ;-)
      // Due to that we inject in both ports and they are shared between two cylinders we only can have one injection
      // in a 360 degree cycle
      if (configPage1.injLayout == INJ_SIAMESE)
      {
        // Cyrrent only one injectore channel is needed
        channel1InjDegrees = 0;

        // Define the next injector to schedule
        injectorStruct[0].next_schedule = 0;

        injectorStructEnd = 0;

        // Full fuel pulse needed
        req_fuel_uS = req_fuel_uS * 2;
      }
      else
      {
        // All other than Siamese need two or more injectores enabled
        channel2InjEnabled = true;
      }
      break;

    case 5:
      //For alternatiing injection, the squirt occurs at different times for each channel
      if(configPage1.injLayout == INJ_SEMISEQUENTIAL || configPage1.injLayout == INJ_PAIRED)
      {
        channel1InjDegrees = 0;
        channel2InjDegrees = 72;
        channel3InjDegrees = 144;
        channel4InjDegrees = 216;
        channel5InjDegrees = 288;
      }
      else if (configPage1.injLayout == INJ_SEQUENTIAL)
      {
        channel1InjDegrees = 0;
        channel2InjDegrees = 144;
        channel3InjDegrees = 288;
        channel4InjDegrees = 432;
        channel5InjDegrees = 576;

        CRANK_ANGLE_MAX_INJ = 720;
      }
      if (!configPage1.injTiming) { channel1InjDegrees = channel2InjDegrees = channel3InjDegrees = channel4InjDegrees = channel5InjDegrees = 0; } //For simultaneous, all squirts happen at the same time
      channel1InjEnabled = true;
      channel2InjEnabled = true;
      channel3InjEnabled = false; //this is disabled as injector 5 function calls 3 & 5 together
      channel4InjEnabled = true;
      channel5InjEnabled = true;
      break;

    case 6:
      if (!configPage1.injTiming) { channel1InjDegrees = channel2InjDegrees = channel3InjDegrees = 0; } //For simultaneous, all squirts happen at the same time

      configPage1.injLayout = 0; //This is a failsafe. We can never run semi-sequential with more than 4 cylinders
      channel1InjEnabled = true;
      channel2InjEnabled = true;
      channel3InjEnabled = true;
      break;

    case 8:
      if (!configPage1.injTiming)  { channel1InjDegrees = channel2InjDegrees = channel3InjDegrees = channel4InjDegrees = 0; } //For simultaneous, all squirts happen at the same time

      configPage1.injLayout = 0; //This is a failsafe. We can never run semi-sequential with more than 4 cylinders
      channel1InjEnabled = true;
      channel2InjEnabled = true;
      channel3InjEnabled = true;
      channel4InjEnabled = true;
      break;

    default: //Handle this better!!!
      channel1InjDegrees = 0;
      channel2InjDegrees = 180;
      break;
  }

  //If individual angel is defined pr. injectore then we add it here
  channel1InjDegrees += configPage1.inj1Ang;
  channel2InjDegrees += configPage1.inj2Ang;
  channel3InjDegrees += configPage1.inj3Ang;
  channel4InjDegrees += configPage1.inj4Ang;

  //For missing Tooth wheels - teeth triggering is used and we need to extract the defined ATDC value
  if ( configPage2.TrigPattern == 0 || configPage2.TrigPattern == 16 )  {
    channel1InjDegrees -= configPage2.triggerAngle;
    channel2InjDegrees -= configPage2.triggerAngle;
    channel3InjDegrees -= configPage2.triggerAngle;
    channel4InjDegrees -= configPage2.triggerAngle;
  }
}

//*********************************************************************************************************
// Do all the fuel calculations including timing
// Scheduling is handled too for non tooth timing
//*********************************************************************************************************
void fuel_calc( int crankAngle, int timePerDegree, int triggerToothAngle, bool fuelOn  )
{
  //Begin the fuel calculation

  unsigned long sumCorrections = 100;
  byte activeCorrections = 0;
  byte result; //temporary variable to store the result of each corrections function
  injector1StartAngle = 0;
  injector2StartAngle = 0;
  injector3StartAngle = 0;
  injector4StartAngle = 0;
  injector5StartAngle = 0;
  uint16_t iVE, iCorrections;
  uint16_t iMAP = 100;
  uint16_t iAFR = 147;

  // Temp for injector pulse width to ensure that we don't get hit by overflow
  unsigned long int tempPW4, tempPW3, tempPW2, tempPW1;

  //These are used for comparisons on channels above 1 where the starting angle (for injectors or ignition) can be less than a single loop time
  //(Don't ask why this is needed, it will break your head)
  int tempCrankAngle;
  int tempStartAngle;

  //Calculate an injector pulsewidth

  //Injector open time in uS. Comes through as ms*10 (Eg 15.5ms = 155)
  //Injector open time is affected by battery voltage and must be corrected
  currentStatus.batCorrection = correctionBatVoltage();
  if (currentStatus.batCorrection != 100) { inj_opentime_uS = (configPage1.injOpen * currentStatus.batCorrection); }

  //The values returned by each of the correction functions are multipled together and then divided back to give a single 0-255 value.
  currentStatus.wueCorrection = correctionWUE();
  if (currentStatus.wueCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.wueCorrection); activeCorrections++; }

  result = correctionASE();
  if (result != 100) { sumCorrections = (sumCorrections * result); activeCorrections++; }

  result = correctionCranking();
  if (result != 100) { sumCorrections = (sumCorrections * result); activeCorrections++; }
  if (activeCorrections == 3) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; } // Need to check this to ensure that sumCorrections doesn't overflow. Can occur when the number of corrections is greater than 3 (Which is 100^4) as 100^5 can overflow

  currentStatus.TAEamount = correctionAccel();
  if (currentStatus.TAEamount != 100) { sumCorrections = (sumCorrections * currentStatus.TAEamount); activeCorrections++; }
  if (activeCorrections == 3) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  result = correctionFloodClear();
  if (result != 100) { sumCorrections = (sumCorrections * result); activeCorrections++; }
  if (activeCorrections == 3) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.egoCorrection = correctionAFRClosedLoop();
  if (currentStatus.egoCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.egoCorrection); activeCorrections++; }
  if (activeCorrections == 3) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.iatCorrection = correctionIATDensity();
  if (currentStatus.iatCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.iatCorrection); activeCorrections++; }
  if (activeCorrections == 3) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.flexCorrection = correctionFlex();
  if (currentStatus.flexCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.flexCorrection); activeCorrections++; }
  if (activeCorrections == 3) { sumCorrections = sumCorrections / powint(100,activeCorrections); activeCorrections = 0; }

  currentStatus.launchCorrection = correctionLaunch();
  if (currentStatus.launchCorrection != 100) { sumCorrections = (sumCorrections * currentStatus.launchCorrection); activeCorrections++; }

  sumCorrections = sumCorrections / powint(100,activeCorrections);

  if(sumCorrections > 255) { sumCorrections = 255; } //This is the maximum allowable increase

  currentStatus.corrections = (byte)sumCorrections;

  if (configPage1.algorithm == LOAD_SOURCE_MAP) //Check which fuelling algorithm is being used
  {
    //Speed Density
    currentStatus.VE = get3DTableValue(&fuelTable, currentStatus.MAP, currentStatus.RPM); //Perform lookup into fuel map for RPM vs MAP value
  }
  else
  {
    //Alpha-N
    currentStatus.VE = get3DTableValue(&fuelTable, currentStatus.TPS, currentStatus.RPM); //Perform lookup into fuel map for RPM vs TPS value
  }

  //100% float free version, does sacrifice a little bit of accuracy, but not much.
  iVE = ((unsigned int) currentStatus.VE << 7) / 100;

  iCorrections = (currentStatus.corrections << 7) / 100;

  //Need to use an intermediate value to avoid overflowing the long
  tempPW1 = ((unsigned long int) req_fuel_uS * (unsigned long int) iVE) >> 7;

  //Include multiply MAP (vs baro) if enabled
  if ( configPage1.multiplyMAP == true ) {
    iMAP = ((unsigned int) currentStatus.MAP << 7) / currentStatus.baro;
    tempPW1 = (tempPW1 * (unsigned long int) iMAP) >> 7;
  }

  //Include AFR (vs target) if enabled - EGO type must be set to wideband for this to be used
  if ( (configPage1.includeAFR == true) && (configPage3.egoType == 2) ) {
    iAFR = ((unsigned int) currentStatus.O2 << 7) / currentStatus.afrTarget;
    tempPW1 = (tempPW1 * (unsigned long int) iAFR) >> 7;
  }

  tempPW1 = (tempPW1 * (unsigned long int) iCorrections) >> 7;

  tempPW4 = tempPW3 = tempPW2 = tempPW1; // Initial state is for all pulsewidths to be the same (This gets changed below)

  // If individual fuel trim is defined, we apply the defined correction pr. cylinder
  if(configPage3.fuelTrimEnabled)
  {
    unsigned int pw1percent = 100 + (byte)get3DTableValue(&trim1Table, currentStatus.MAP, currentStatus.RPM) - OFFSET_FUELTRIM;
    unsigned int pw2percent = 100 + (byte)get3DTableValue(&trim2Table, currentStatus.MAP, currentStatus.RPM) - OFFSET_FUELTRIM;
    unsigned int pw3percent = 100 + (byte)get3DTableValue(&trim3Table, currentStatus.MAP, currentStatus.RPM) - OFFSET_FUELTRIM;
    unsigned int pw4percent = 100 + (byte)get3DTableValue(&trim4Table, currentStatus.MAP, currentStatus.RPM) - OFFSET_FUELTRIM;

    if (pw1percent != 100) { tempPW1 = (pw1percent * tempPW1) / 100; }
    if (pw2percent != 100) { tempPW2 = (pw2percent * tempPW2) / 100; }
    if (pw3percent != 100) { tempPW3 = (pw3percent * tempPW3) / 100; }
    if (pw4percent != 100) { tempPW4 = (pw4percent * tempPW4) / 100; }
  }

  // As last step add injectore opening time to get final pulse width
  tempPW1 += inj_opentime_uS;
  tempPW2 += inj_opentime_uS;
  tempPW3 += inj_opentime_uS;
  tempPW4 += inj_opentime_uS;

  //Make sure it won't overflow. This means the maximum pulsewidth possible is 65.535mS
  if ( tempPW1 > 65535) { currentStatus.PW1 = 65535; } else { currentStatus.PW1 = (unsigned int) tempPW1; }
  if ( tempPW2 > 65535) { currentStatus.PW2 = 65535; } else { currentStatus.PW2 = (unsigned int) tempPW2; }
  if ( tempPW3 > 65535) { currentStatus.PW3 = 65535; } else { currentStatus.PW3 = (unsigned int) tempPW3; }
  if ( tempPW4 > 65535) { currentStatus.PW4 = 65535; } else { currentStatus.PW4 = (unsigned int) tempPW4; }

  //Check that the duty cycle of the calculated pulsewidth isn't too high against the limit set. This is disabled at cranking
  if( !BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
  {
    unsigned long int pwLimit = percentage(configPage1.dutyLim, revolutionTime); //The pulsewidth limit is determined to be the duty cycle limit (Eg 85%) by the total time it takes to perform 1 revolution
    if (CRANK_ANGLE_MAX_INJ == 720 || configPage1.injLayout == INJ_SIAMESE) { pwLimit = pwLimit * 2; } //For sequential and siamese, the maximum pulse time is double (2 revolutions). Wouldn't work for 2 stroke...
    if (currentStatus.PW1 > pwLimit) { currentStatus.PW1 = pwLimit; }
    if (currentStatus.PW2 > pwLimit) { currentStatus.PW2 = pwLimit; }
    if (currentStatus.PW3 > pwLimit) { currentStatus.PW3 = pwLimit; }
    if (currentStatus.PW4 > pwLimit) { currentStatus.PW4 = pwLimit; }
  }

  // Check if we are in a fuelcut state
  bitWrite(currentStatus.squirt, BIT_SQUIRT_DFCO, correctionDFCO());

  // If we have a fuelcut state set then set pulse width to 0
  // This could be added in the start off the function, but we don't need the CPU cycle if we are in fuelcut state so who care :-)
  if ( bitRead(currentStatus.squirt, BIT_SQUIRT_DFCO) == 1 )
  {
    currentStatus.PW1 = currentStatus.PW2 = currentStatus.PW3 = currentStatus.PW4 = 0;
  }

  //***********************************************************************************************
  //CALCULATE INJECTION TIMING
  // New code for tooth based injection schedules - added Jan. 2018

  // The PSOC5 code schedule is done in decoders.ino
#if defined(CORE_PSOC5)
  if (( configPage2.TrigPattern == 0 || configPage2.TrigPattern == 16 ) && configPage1.perToothIgn == true ) // Use the new code / schedule if on the PSOC5 platform
#else
  if ( 1 != 1 ) // Use the old schedule for all other platforms
#endif
  {
    //Calculate tooth scheduling data for injector(s)
    injectorToothScheduleCalc( currentStatus.PW1, inj_opentime_uS, channel1InjDegrees, timePerDegree, triggerToothAngle, CRANK_ANGLE_MAX_INJ, 0 );

    if ( injectorStructEnd > 0 )
    {
      injectorToothScheduleCalc( currentStatus.PW2, inj_opentime_uS, channel2InjDegrees, timePerDegree, triggerToothAngle, CRANK_ANGLE_MAX_INJ, 1 );
    }
    if ( injectorStructEnd > 1 )
    {
      injectorToothScheduleCalc( currentStatus.PW3, inj_opentime_uS, channel3InjDegrees, timePerDegree, triggerToothAngle, CRANK_ANGLE_MAX_INJ, 2 );
    }
    if ( injectorStructEnd > 2 )
    {
      injectorToothScheduleCalc( currentStatus.PW4, inj_opentime_uS, channel4InjDegrees, timePerDegree, triggerToothAngle, CRANK_ANGLE_MAX_INJ, 3 );
    }
    // 5 cylinder is current not supported in the PSOC5 implementation

    // Set the array number that we start at when running scheduling, scheduling starts from tooth one
    if ( injectorStruct[0].start_tooth < injectorStruct[injectorStructEnd].start_tooth || injectorStructEnd == 0 )
    {
      injectorStructStart = 0;
    }
    else
    {
      injectorStructStart = injectorStructEnd;
    }
  }
  else // This is for non even spaced trigger wheels using the old fuel schedules
  {
    //***********************************************************************************************
    //CALCULATE INJECTION TIMING
    //Determine next firing angles
    int PWdivTimerPerDegree = div(currentStatus.PW1, timePerDegree).quot; //How many crank degrees the calculated PW will take at the current speed

    injector1StartAngle = configPage1.inj1Ang - PWdivTimerPerDegree ; //This is a little primitive, but is based on the idea that all fuel needs to be delivered before the inlet valve opens. http://www.extraefi.co.uk/support/efi/sequential_fuel.html for more detail
    if(injector1StartAngle < 0) {injector1StartAngle += CRANK_ANGLE_MAX_INJ;}
    if(injector1StartAngle > CRANK_ANGLE_MAX_INJ) {injector1StartAngle -= CRANK_ANGLE_MAX_INJ;}

    //Repeat the above for each cylinder
    switch (configPage1.nCylinders)
    {
      //2 cylinders
      case 2:
        injector2StartAngle = channel2InjDegrees - PWdivTimerPerDegree ;
        if(injector2StartAngle > CRANK_ANGLE_MAX_INJ) {injector2StartAngle -= CRANK_ANGLE_MAX_INJ;}
        if(injector2StartAngle < 0) {injector2StartAngle += CRANK_ANGLE_MAX_INJ;}
        break;
      //3 cylinders
      case 3:
        injector2StartAngle = channel2InjDegrees - PWdivTimerPerDegree ;
        if(injector2StartAngle > CRANK_ANGLE_MAX_INJ) {injector2StartAngle -= CRANK_ANGLE_MAX_INJ;}
        if(injector2StartAngle < 0) {injector2StartAngle += CRANK_ANGLE_MAX_INJ;}

        injector3StartAngle = channel3InjDegrees - PWdivTimerPerDegree ;
        if(injector3StartAngle > CRANK_ANGLE_MAX_INJ) {injector3StartAngle -= CRANK_ANGLE_MAX_INJ;}
        if(injector3StartAngle < 0) {injector3StartAngle += CRANK_ANGLE_MAX_INJ;}
        break;
      //4 cylinders
      case 4:
        injector2StartAngle = channel2InjDegrees - PWdivTimerPerDegree ;
        if(injector2StartAngle > CRANK_ANGLE_MAX_INJ) {injector2StartAngle -= CRANK_ANGLE_MAX_INJ;}
        if(injector2StartAngle < 0) {injector2StartAngle += CRANK_ANGLE_MAX_INJ;}

        if(configPage1.injLayout == INJ_SEQUENTIAL)
        {
          injector3StartAngle = channel3InjDegrees - PWdivTimerPerDegree ;
          if(injector3StartAngle > CRANK_ANGLE_MAX_INJ) {injector3StartAngle -= CRANK_ANGLE_MAX_INJ;}
          if(injector3StartAngle < 0) {injector3StartAngle += CRANK_ANGLE_MAX_INJ;}

          injector4StartAngle = channel4InjDegrees - PWdivTimerPerDegree ;
          if(injector4StartAngle > CRANK_ANGLE_MAX_INJ) {injector4StartAngle -= CRANK_ANGLE_MAX_INJ;}
          if(injector4StartAngle < 0) {injector4StartAngle += CRANK_ANGLE_MAX_INJ;}
        }
        break;
      //5 cylinders
      case 5:
        injector2StartAngle = channel2InjDegrees - PWdivTimerPerDegree ;
        if(injector2StartAngle > CRANK_ANGLE_MAX_INJ) {injector2StartAngle -= CRANK_ANGLE_MAX_INJ;}
        injector3StartAngle = channel3InjDegrees - PWdivTimerPerDegree ;
        if(injector3StartAngle > CRANK_ANGLE_MAX_INJ) {injector3StartAngle -= CRANK_ANGLE_MAX_INJ;}
        injector4StartAngle = channel4InjDegrees - PWdivTimerPerDegree ;
        if(injector4StartAngle > CRANK_ANGLE_MAX_INJ) {injector4StartAngle -= CRANK_ANGLE_MAX_INJ;}
        injector5StartAngle = channel5InjDegrees - PWdivTimerPerDegree ;
        if(injector5StartAngle > CRANK_ANGLE_MAX_INJ) {injector5StartAngle -= CRANK_ANGLE_MAX_INJ;}
        break;
      //6 cylinders
      case 6:
        injector2StartAngle = channel2InjDegrees - PWdivTimerPerDegree ;
        if(injector2StartAngle > CRANK_ANGLE_MAX_INJ) {injector2StartAngle -= CRANK_ANGLE_MAX_INJ;}
        injector3StartAngle = channel3InjDegrees - PWdivTimerPerDegree ;
        if(injector3StartAngle > CRANK_ANGLE_MAX_INJ) {injector3StartAngle -= CRANK_ANGLE_MAX_INJ;}
        break;
      //8 cylinders
      case 8:
        injector2StartAngle = channel2InjDegrees - PWdivTimerPerDegree ;
        if(injector2StartAngle > CRANK_ANGLE_MAX_INJ) {injector2StartAngle -= CRANK_ANGLE_MAX_INJ;}
        injector3StartAngle = channel3InjDegrees - PWdivTimerPerDegree ;
        if(injector3StartAngle > CRANK_ANGLE_MAX_INJ) {injector3StartAngle -= CRANK_ANGLE_MAX_INJ;}
        injector4StartAngle = channel4InjDegrees - PWdivTimerPerDegree ;
        if(injector4StartAngle > CRANK_ANGLE_MAX_INJ) {injector4StartAngle -= CRANK_ANGLE_MAX_INJ;}
        break;
      //Will hit the default case on 1 cylinder or >8 cylinders. Do nothing in these cases
      default:
        break;
    }

    //***********************************************************************************************
    //| BEGIN FUEL SCHEDULES
    //Finally calculate the time (uS) until we reach the firing angles and set the schedules
    //We only need to set the shcedule if we're BEFORE the open angle
    //This may potentially be called a number of times as we get closer and closer to the opening time

    if (crankAngle > CRANK_ANGLE_MAX_INJ ) { crankAngle -= 360; }

    if (fuelOn && currentStatus.PW1 > 0 && !BIT_CHECK(currentStatus.squirt, BIT_SQUIRT_BOOSTCUT))
    {
      if (injector1StartAngle > crankAngle)
      {
        setFuelSchedule1(
                  ((unsigned long)(injector1StartAngle - crankAngle) * (unsigned long)timePerDegree),
                  (unsigned long)currentStatus.PW1
                  );
      }

      /*-----------------------------------------------------------------------------------------
      | A Note on tempCrankAngle and tempStartAngle:
      |   The use of tempCrankAngle/tempStartAngle is described below. It is then used in the same way for channels 2, 3 and 4 on both injectors and ignition
      |   Essentially, these 2 variables are used to realign the current crank angle and the desired start angle around 0 degrees for the given cylinder/output
      |   Eg: If cylinder 2 TDC is 180 degrees after cylinder 1 (Eg a standard 4 cylidner engine), then tempCrankAngle is 180* less than the current crank angle and
      |       tempStartAngle is the desired open time less 180*. Thus the cylinder is being treated relative to its own TDC, regardless of its offset
      |
      |   This is done to avoid problems with very short of very long times until tempStartAngle.
      |   This will very likely need to be rewritten when sequential is enabled
      |------------------------------------------------------------------------------------------
      */
      if(channel2InjEnabled)
      {
        tempCrankAngle = crankAngle - channel2InjDegrees;
        if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
        tempStartAngle = injector2StartAngle - channel2InjDegrees;
        if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
        //      if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule2.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
        if ( tempStartAngle > tempCrankAngle )
        {
          setFuelSchedule2(
                    ((unsigned long)(tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                    (unsigned long)currentStatus.PW2
                    );
        }
      }

      if(channel3InjEnabled)
      {
        tempCrankAngle = crankAngle - channel3InjDegrees;
        if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
        tempStartAngle = injector3StartAngle - channel3InjDegrees;
        if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
        //      if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule3.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
        if ( tempStartAngle > tempCrankAngle )
        {
          setFuelSchedule3(
                    ((unsigned long)(tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                    (unsigned long)currentStatus.PW3
                    );
        }
      }

      if(channel4InjEnabled)
      {
        tempCrankAngle = crankAngle - channel4InjDegrees;
        if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
        tempStartAngle = injector4StartAngle - channel4InjDegrees;
        if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
        //      if ( (tempStartAngle <= tempCrankAngle) && (fuelSchedule4.Status == RUNNING) ) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
        if ( tempStartAngle > tempCrankAngle )
        {
          setFuelSchedule4(
                    ((unsigned long)(tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                    (unsigned long)currentStatus.PW4
                    );
        }
      }

      if(channel5InjEnabled)
      {
        tempCrankAngle = crankAngle - channel5InjDegrees;
        if( tempCrankAngle < 0) { tempCrankAngle += CRANK_ANGLE_MAX_INJ; }
        tempStartAngle = injector5StartAngle - channel5InjDegrees;
        if ( tempStartAngle < 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
        //      if (tempStartAngle <= tempCrankAngle && fuelSchedule5.schedulesSet == 0) { tempStartAngle += CRANK_ANGLE_MAX_INJ; }
        if ( tempStartAngle > tempCrankAngle )
        {
          //Note the hacky use of fuel schedule 3 below
          /*
          setFuelSchedule3(openInjector3and5,
                    ((unsigned long)(tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                    (unsigned long)currentStatus.PW1,
                    closeInjector3and5
                  );*/
          setFuelSchedule3(
                    ((unsigned long)(tempStartAngle - tempCrankAngle) * (unsigned long)timePerDegree),
                    (unsigned long)currentStatus.PW1
                    );
        }
      }
    }
  }
} //End fuel_calc

/*
Warm Up Enrichment (WUE)
Uses a 2D enrichment table (WUETable) where the X axis is engine temp and the Y axis is the amount of extra fuel to add
*/
static inline byte correctionWUE()
{
  byte WUEValue;
  //Possibly reduce the frequency this runs at (Costs about 50 loops per second)
  if (currentStatus.coolant > (WUETable.axisX[9] - CALIBRATION_TEMPERATURE_OFFSET))
  {
    //This prevents us doing the 2D lookup if we're already up to temp
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_WARMUP);
    WUEValue = 100;
  }
  else
  {
    BIT_SET(currentStatus.engine, BIT_ENGINE_WARMUP);
    WUEValue = table2D_getValue(&WUETable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
  }

  return WUEValue;
}

/*
Cranking Enrichment
Additional fuel % to be added when the engine is cranking
*/
static inline byte correctionCranking()
{
  byte crankingValue = 100;
  //if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) ) { crankingValue = 100 + configPage1.crankingPct; }
  if ( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
  {
    crankingValue = table2D_getValue(&crankingEnrichTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);
  }
  return crankingValue;
}

/*
After Start Enrichment
This is a short period (Usually <20 seconds) immediately after the engine first fires (But not when cranking)
where an additional amount of fuel is added (Over and above the WUE amount)
*/
static inline byte correctionASE()
{
  byte ASEValue;
  //Two checks are requiredL:
  //1) Is the negine run time less than the configured ase time
  //2) Make sure we're not still cranking
  if ( (currentStatus.runSecs < configPage1.aseCount) && !(BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK)) )
  {
    BIT_SET(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as active.
    ASEValue = 100 + configPage1.asePct;
  }
  else
  {
    BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ASE); //Mark ASE as inactive.
    ASEValue = 100;
  }
  return ASEValue;
}

/*
TPS based acceleration enrichment
Calculates the % change of the throttle over time (%/second) and performs a lookup based on this
When the enrichment is turned on, it runs at that amount for a fixed period of time (taeTime)
*/
static inline byte correctionAccel()
{
  byte accelValue = 100;
  //First, check whether the accel. enrichment is already running
  if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_ACC) )
  {
    //If it is currently running, check whether it should still be running or whether it's reached it's end time
    if( micros() >= currentStatus.TAEEndTime )
    {
      //Time to turn enrichment off
      BIT_CLEAR(currentStatus.engine, BIT_ENGINE_ACC);
      currentStatus.TAEamount = 0;
      accelValue = 100;
      currentStatus.tpsDOT = 0;
    }
    else
    {
      //Enrichment still needs to keep running. Simply return the total TAE amount
      accelValue = currentStatus.TAEamount;
    }
  }
  else
  {
    int8_t TPS_change = (currentStatus.TPS - currentStatus.TPSlast);
    //Check for deceleration (Deceleration adjustment not yet supported)
    //Also check for only very small movement (Movement less than or equal to 2% is ignored). This not only means we can skip the lookup, but helps reduce false triggering around 0-2% throttle openings
    if (TPS_change <= 2)
    {
      accelValue = 100;
      currentStatus.tpsDOT = 0;
    }
    else
    {
      //If TAE isn't currently turned on, need to check whether it needs to be turned on
      int rateOfChange = ldiv(1000000, (currentStatus.TPS_time - currentStatus.TPSlast_time)).quot * TPS_change; //This is the % per second that the TPS has moved
      currentStatus.tpsDOT = rateOfChange / 10; //The TAE bins are divided by 10 in order to allow them to be stored in a byte. Faster as this than divu10

      if (rateOfChange > configPage1.tpsThresh)
      {
        BIT_SET(currentStatus.engine, BIT_ENGINE_ACC); //Mark accleration enrichment as active.
        currentStatus.TAEEndTime = micros() + ((unsigned long)configPage1.taeTime * 10000); //Set the time in the future where the enrichment will be turned off. taeTime is stored as mS / 10, so multiply it by 100 to get it in uS
        accelValue = 100 + table2D_getValue(&taeTable, currentStatus.tpsDOT);
      }
    }
  }

  return accelValue;
}

/*
Simple check to see whether we are cranking with the TPS above the flood clear threshold
This function always returns either 100 or 0
*/

static inline byte correctionFloodClear()
{
  byte floodValue = 100;
  if( BIT_CHECK(currentStatus.engine, BIT_ENGINE_CRANK) )
  {
    //Engine is currently cranking, check what the TPS is
    if(currentStatus.TPS >= configPage2.floodClear)
    {
      //Engine is cranking and TPS is above threshold. Cut all fuel
      floodValue = 0;
    }
  }
  return floodValue;
}

/*
Battery Voltage correction
Uses a 2D enrichment table (WUETable) where the X axis is engine temp and the Y axis is the amount of extra fuel to add
*/
static inline byte correctionBatVoltage()
{
  byte batValue = 100;
  if (currentStatus.battery10 > (injectorVCorrectionTable.axisX[5])) { batValue = injectorVCorrectionTable.values[injectorVCorrectionTable.xSize-1]; } //This prevents us doing the 2D lookup if the voltage is above maximum
  else { batValue = table2D_getValue(&injectorVCorrectionTable, currentStatus.battery10); }

  return batValue;
}

/*
Simple temperature based corrections lookup based on the inlet air temperature.
This corrects for changes in air density from movement of the temperature
*/
static inline byte correctionIATDensity()
{
  byte IATValue = 100;
  if ( (currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET) > (IATDensityCorrectionTable.axisX[8])) { IATValue = IATDensityCorrectionTable.values[IATDensityCorrectionTable.xSize-1]; } //This prevents us doing the 2D lookup if the intake temp is above maximum
  else { IATValue = table2D_getValue(&IATDensityCorrectionTable, currentStatus.IAT + CALIBRATION_TEMPERATURE_OFFSET); }//currentStatus.IAT is the actual temperature, values in IATDensityCorrectionTable.axisX are temp+offset

  return IATValue;
}

/*
Launch control has a setting to increase the fuel load to assist in bringing up boost
This simple check applies the extra fuel if we're currently launching
*/
static inline byte correctionLaunch()
{
  byte launchValue = 100;
  if(currentStatus.launchingHard || currentStatus.launchingSoft) { launchValue = (100 + configPage3.lnchFuelAdd); }

  return launchValue;
}

/*
 * Returns true if decelleration fuel cutoff should be on, false if its off
 */
static inline bool correctionDFCO()
{
  bool DFCOValue = false;
  if ( configPage1.dfcoEnabled == 1 )
  {
    if ( bitRead(currentStatus.squirt, BIT_SQUIRT_DFCO) == 1 ) { DFCOValue = ( currentStatus.RPM > ( configPage2.dfcoRPM * 10) ) && ( currentStatus.TPS < configPage2.dfcoTPSThresh ); }
    else { DFCOValue = ( currentStatus.RPM > (unsigned int)( (configPage2.dfcoRPM * 10) + configPage2.dfcoHyster) ) && ( currentStatus.TPS < configPage2.dfcoTPSThresh ); }
  }
  return DFCOValue;
}

/*
 * Flex fuel adjustment to vary fuel based on ethanol content
 * The amount of extra fuel required is a linear relationship based on the % of ethanol.
*/
static inline byte correctionFlex()
{
  byte flexValue = 100;
  if(configPage1.flexEnabled == 1)
  {
    byte flexRange = configPage1.flexFuelHigh - configPage1.flexFuelLow;
    flexValue = percentage(currentStatus.ethanolPct, flexRange) + 100;
  }
  return flexValue;
}

/*
Lookup the AFR target table and perform either a simple or PID adjustment based on this

Simple (Best suited to narrowband sensors):
If the O2 sensor reports that the mixture is lean/rich compared to the desired AFR target, it will make a 1% adjustment
It then waits <egoDelta> number of ignition events and compares O2 against the target table again. If it is still lean/rich then the adjustment is increased to 2%
This continues until either:
  a) the O2 reading flips from lean to rich, at which point the adjustment cycle starts again at 1% or
  b) the adjustment amount increases to <egoLimit> at which point it stays at this level until the O2 state (rich/lean) changes

PID (Best suited to wideband sensors):

*/

static inline byte correctionAFRClosedLoop()
{
  byte AFRValue = 100;
  if( configPage3.egoType > 0 ) //egoType of 0 means no O2 sensor
  {
    currentStatus.afrTarget = currentStatus.O2; //Catch all incase the below doesn't run. This prevents the Include AFR option from doing crazy things if the AFR target conditions aren't met. This value is changed again below if all conditions are met.

    //Check the ignition count to see whether the next step is required
    //This if statement is the equivalent of ( (ignitionCount % configPage3.egoCount) == 0 ) but without the expensive modulus operation. ie It results in True every <egoCount> ignition loops. Note that it only works for power of two vlaues for egoCount
    //if( (ignitionCount & (configPage3.egoCount - 1)) == 1 )
    {
      //Determine whether the Y axis of the AFR target table tshould be MAP (Speed-Density) or TPS (Alpha-N)
      byte yValue;
      if (configPage1.algorithm == 0) { yValue = currentStatus.MAP; }
      else  { yValue = currentStatus.TPS; }
      currentStatus.afrTarget = get3DTableValue(&afrTable, yValue, currentStatus.RPM); //Perform the target lookup

      //Check all other requirements for closed loop adjustments
      if( (currentStatus.coolant > (int)(configPage3.egoTemp - CALIBRATION_TEMPERATURE_OFFSET)) && (currentStatus.RPM > (unsigned int)(configPage3.egoRPM * 100)) && (currentStatus.TPS < configPage3.egoTPSMax) && (currentStatus.O2 < configPage3.ego_max) && (currentStatus.O2 > configPage3.ego_min) && (currentStatus.runSecs > configPage3.ego_sdelay) )
      {
        //Check which algorithm is used, simple or PID
        if (configPage3.egoAlgorithm == 0)
        {
          //*************************************************************************************************************************************
          //Simple algorithm
          if(currentStatus.O2 > currentStatus.afrTarget)
          {
            //Running lean
            if(currentStatus.egoCorrection < (100 + configPage3.egoLimit) ) //Fueling adjustment must be at most the egoLimit amount (up or down)
            {
              if(currentStatus.egoCorrection >= 100) { AFRValue = (currentStatus.egoCorrection + 1); } //Increase the fueling by 1%
              else { AFRValue = 100; } //This means that the last reading had been rich, so simply return back to no adjustment (100%)
            }
            else { AFRValue = currentStatus.egoCorrection; } //Means we're at the maximum adjustment amount, so simply return then again
          }
          else
            //Running Rich
            if(currentStatus.egoCorrection > (100 - configPage3.egoLimit) ) //Fueling adjustment must be at most the egoLimit amount (up or down)
            {
              if(currentStatus.egoCorrection <= 100) { AFRValue = (currentStatus.egoCorrection - 1); } //Increase the fueling by 1%
              else { AFRValue = 100; } //This means that the last reading had been lean, so simply return back to no adjustment (100%)
            }
            else { AFRValue = currentStatus.egoCorrection; } //Means we're at the maximum adjustment amount, so simply return then again
        }
        else if(configPage3.egoAlgorithm == 2)
        {
          //*************************************************************************************************************************************
          //PID algorithm
          egoPID.SetOutputLimits((long)(-configPage3.egoLimit), (long)(configPage3.egoLimit)); //Set the limits again, just incase the user has changed them since the last loop. Note that these are sent to the PID library as (Eg:) -15 and +15
          egoPID.SetTunings(configPage3.egoKP, configPage3.egoKI, configPage3.egoKD); //Set the PID values again, just incase the user has changed them since the last loop
          PID_O2 = (long)(currentStatus.O2);
          PID_AFRTarget = (long)(currentStatus.afrTarget);

          egoPID.Compute();
          //currentStatus.egoCorrection = 100 + PID_output;
          AFRValue = 100 + PID_output;
        }
        else { AFRValue = 100; } // Occurs if the egoAlgorithm is set to 0 (No Correction)

      } //Multi variable check
    } //ignitionCount
  } //egoType

  return AFRValue; //Catch all (Includes when AFR target = current AFR
}

// Calculate the tooth and timer data required to schedule the injector
void injectorToothScheduleCalc( int injPulseWidth, int injOpenTime, int injDegrees, int injTimePerDegree, int injTriggerToothAngle, int injCRANK_ANGLE_MAX, uint8_t injInjectorPortMinusOne )
{
  // Temp's used
  int injTempStartTooth;
  unsigned int injTempStartDelayuS;
  unsigned int injTempTotalTimeuS;

  if ( configPage1.injLayout == INJ_SIAMESE )
  {
    // Special injection opening calculations for siamese-port engines like the classic Mini A/A+ engine and the B-series engine of the MGB
    // Missing a parameter for the average delay time from injector tip to valve - approximat 2,5 mS for a A engine, it depends on
    // engine speed and load. It could require an lookup table, but with the huge injector I have selected it should not be an issue
    // You can read more about the siamese-port port issue here http://www.starchak.ca/efi/siamese.htm

    //int injTimeFromInjectorTipToValveuS = 2000; // hardcoded value in uS
    int injTimeFromInjectorTipToValveuS = 0; // hardcoded value in uS // Debug

    injDegrees -= (( injOpenTime + injTimeFromInjectorTipToValveuS ) / injTimePerDegree );
  }
  else
  {
    // We have to extract the injector opening time from the defined start time to get the real start time
    // and to ensure that we have injected all before port opening we extract the pulse width too
    injDegrees -= (( injOpenTime + injPulseWidth ) / injTimePerDegree );
  }

  // We can't start at a negative angel, use the next comming time that fit
  if ( injDegrees  < 0 ) { injDegrees += injCRANK_ANGLE_MAX; }

  // UPS, we are to far ahead
  if ( injDegrees > injCRANK_ANGLE_MAX ) { injDegrees -= injCRANK_ANGLE_MAX; }

  // Calculate the tooth number where we should start the fuel timer
  injTempStartTooth = injDegrees / injTriggerToothAngle ;

  // If we have 0 then we need to use the tooth before 1
  if ( injTempStartTooth == 0 ) { injTempStartTooth = scheduling_max_tooth_count; }

  // Calculate the time we should wait before the injection pulse is started after we have reached the wanted tooth
  injTempStartDelayuS = (( injDegrees % injTriggerToothAngle ) * injTimePerDegree ) ;

  // We need some time before the pulse should be fired otherwise we may see issues in the timing
  if ( injTempStartDelayuS < 10 )
  {
    injTempStartDelayuS += ( injTriggerToothAngle * injTimePerDegree );
    injTempStartTooth--;
  }

  // Scheduler is set to start at next comming tooth - so to save time in the interupt we extracting 1 here.
  injTempStartTooth--;

  // If below 1 then we add the number of teeths on the wheel to get a positive number
  if ( injTempStartTooth < 1 ) { injTempStartTooth += scheduling_max_tooth_count; }

  injTempTotalTimeuS = injTempStartDelayuS + injPulseWidth;

  // Copy to injector structure - precalculated temp's used to speed the update time up
  injectorStruct[injInjectorPortMinusOne].start_tooth = (unsigned int) injTempStartTooth;
  injectorStruct[injInjectorPortMinusOne].start_delay_uS = injTempStartDelayuS;
  injectorStruct[injInjectorPortMinusOne].total_timer_time_uS = injTempTotalTimeuS;
  injectorStruct[injInjectorPortMinusOne].pulse_width_uS = injPulseWidth;
}
