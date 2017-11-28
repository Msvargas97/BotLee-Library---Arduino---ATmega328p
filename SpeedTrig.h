/*
==============================================================
 SpeedTrig Library
==============================================================
 Original Code (C) 2012-2013 Oscar Liang
 Ported September 2014 with permission from original author
 Licensed under the MIT licence.
 
 This is a library to quickly do common trigonometry functions
 on Arduinos/other microprocessors.
 
 This is the main header file.
==============================================================
*/
//Library official:https://github.com/Anon-Penguin/SpeedTrig/blob/master/SpeedTrig/SpeedTrig.cpp

//Editada por: Michael Vargas

//Include Guard:
#ifndef SpeedTrig_h
#define SpeedTrig_h

//Arduino libraries:
#include "Arduino.h"

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)
#define PI_2	M_PI/2

class Speed_Trig {
  public:
    Speed_Trig();
    int radToMicro(float rad);
    int floatToInt(float input);
    float sin(int deg);
    float cos(int deg);
    float atan2(float y, float x);
};

//Create class to use:
//extern Speed_Trig SpeedTrig;

#endif //#ifndef SpeedTrig_h

