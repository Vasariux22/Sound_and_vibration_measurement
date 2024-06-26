/** Generated by itemis CREATE code generator. */

#ifndef STATECHART_REQUIRED_H_
#define STATECHART_REQUIRED_H_

#include "../src/sc_types.h"
#include "Statechart.h"

#ifdef __cplusplus
extern "C"
{
#endif 

/*! \file
This header defines prototypes for all functions that are required by the state machine implementation.

This state machine makes use of operations declared in the state machines interface or internal scopes. Thus the function prototypes:
- statechart_startConvADC
- statechart_readADCSample
- statechart_processDataADC
- statechart_saveADCSample
- statechart_displayInfo_ADC
- statechart_tarpinis_RMS
- statechart_save_tarpinis_RMS
- statechart_usartTransmit_ADC
- statechart_save_UsartTransmit_ADC
- statechart_readI2CSensor
- statechart_processDataI2C
- statechart_saveI2CSample
- statechart_usartTransmit_I2C
- statechart_save_UsartTransmit_I2C
- statechart_displayInfo_I2C
are defined.

These functions will be called during a 'run to completion step' (runCycle) of the statechart. 
There are some constraints that have to be considered for the implementation of these functions:
- never call the statechart API functions from within these functions.
- make sure that the execution time is as short as possible.

*/

extern void statechart_startConvADC( Statechart* handle);
extern void statechart_readADCSample( Statechart* handle);
extern void statechart_processDataADC( Statechart* handle);
extern sc_integer statechart_saveADCSample( Statechart* handle, const sc_integer sample_no);
extern void statechart_displayInfo_ADC( Statechart* handle);
extern void statechart_tarpinis_RMS( Statechart* handle);
extern sc_integer statechart_save_tarpinis_RMS( Statechart* handle, const sc_integer sample1_no);
extern void statechart_usartTransmit_ADC( Statechart* handle);
extern sc_integer statechart_save_UsartTransmit_ADC( Statechart* handle, const sc_integer N_ADC);
extern void statechart_readI2CSensor( Statechart* handle);
extern void statechart_processDataI2C( Statechart* handle);
extern sc_integer statechart_saveI2CSample( Statechart* handle, const sc_integer sample_i2c);
extern void statechart_usartTransmit_I2C( Statechart* handle);
extern sc_integer statechart_save_UsartTransmit_I2C( Statechart* handle, const sc_integer N_I2C);
extern void statechart_displayInfo_I2C( Statechart* handle);




#ifdef __cplusplus
}
#endif 

#endif /* STATECHART_REQUIRED_H_ */
