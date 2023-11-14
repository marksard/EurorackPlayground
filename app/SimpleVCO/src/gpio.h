#pragma once
#include <Arduino.h>

#define GATE_A A3 // 1.1.0->gate / rev1.0.0->v/oct
#define GATE_B A2 // 1.1.0->v/oct / rev1.0.0->gate
#define OUT_A D0 // ac out (rev1.0.0->out)
#define OUT_B D1 // dc out
#define PWM_RESO 4096

#define ENC0A D2
#define ENC0B D3
#define ENC1A D6
#define ENC1B D7
#define SW0 D10
#define SW1 D11
#define POT0 A0
#define POT1 A1
