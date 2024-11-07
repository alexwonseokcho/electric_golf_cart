#ifndef _UTILS_HPP
#define _UTILS_HPP

#include "config.hpp"

#define INCH_TO_KM 0.0000254
#define MICROSEC_TO_HOUR 0.00000000027778

#define TURN_PER_SEC_TO_KPH ((WHEEL_DIAMETER * 3.141592) * INCH_TO_KM * 3600.0)
#define KPH_TO_TURN_PER_SEC (1.0 / (TURN_PER_SEC_TO_KPH))

double sgn(double num){
  if(num > 0) return 1.0;
  if(num < 0) return -1.0;
  return 0;
}

#endif