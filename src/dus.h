#ifndef __DUS_H__
#define __DUS_H__

#include "MDR1986VE1T.h"
#include "dusFilter.h"


#define DUS_CONVERT_TO_DEGREES_PER_SEC(code)    ((float)code/32.0)
#define VEL_CALIBRATION_AVERAGE     128 




uint16_t dus_getNextSample();
float    dus_convertCodeToFloatVelocity(uint16_t code);


#endif //__DUS_H__