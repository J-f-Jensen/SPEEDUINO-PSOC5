//*********************************************************************************************************
// All fuel calculations should be done in the fuel_calc functions
// All sub functions only used in the fuel calculations are private and defined in the ino file
//*********************************************************************************************************
#ifndef FUEL_CALC_H
#define FUEL_CALC_H

void fuel_calc_init( void );

void fuel_calc( int crankAngle, int timePerDegree, int triggerToothAngle, bool fuelOn );

#endif // FUEL_CALC_H
