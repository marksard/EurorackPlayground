
#include <Arduino.h>
#include "synthcore.hpp"

///////////////////////////////////////////////////////////////////////////////
// mozzi flow

void setup()
{
    initSynth();
}

void updateControl()
{
    updateSynth();
}

void loop()
{
    synthHook();
}

void setup1()
{
    initOLED();
}

void loop1()
{
    delay(33);
    updateOLED();
}
