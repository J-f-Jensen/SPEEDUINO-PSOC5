//*********************************************************************************************************
// All ignition calculations should be done in the ignition_calc functions
// All sub functions only used in the injecton calculations are private and defined in the ino file
//*********************************************************************************************************

#ifndef IGNITION_CALC_H
#define IGNITION_CALC_H

void ignition_calc_init( void );

void ignition_calc(int crankAngle, int timePerDegree, bool ignitionOn);

#endif // IGNITION_CALC_H
