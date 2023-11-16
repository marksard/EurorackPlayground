#pragma once
#include <Arduino.h>

// #define rev100
#ifdef rev100
#define GATE_A A2 // gate
#define GATE_B A3 // v/oct
#define OUT_A D0 // ac out (rev1.0.0->out)
#define OUT_B D1 // dc out
#else
#define GATE_A A3 // gate (impedance 4.7k)
#define GATE_B A2 // v/oct (impedance 47k)
#define OUT_A D1 // ac out (rev1.0.0->out)
#define OUT_B D0 // dc out
#endif

#define PWM_RESO 4096

#define ENC0A D2
#define ENC0B D3
#define ENC1A D6
#define ENC1B D7
#define SW0 D10
#define SW1 D11
#define POT0 A0
#define POT1 A1
