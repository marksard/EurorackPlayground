/*!
 * Eurorack Playground Mk I GPIOs
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

#define PWM_INTR_PIN D25 // PMW4 B

#ifdef rev100
#define GATE_A A2 // gate
#define GATE_B A3 // v/oct
#define OUT_A D0 // ac out (rev1.0.0->out)
#define OUT_B D1 // dc out
#else
#define GATE_A A3 // gate (impedance 4.7k)
#define GATE_B A2 // v/oct (impedance 47k)
#define OUT_A D1 // dc out
#define OUT_B D0 // ac out (rev1.0.0->out)
#endif

// rev110改専用
#ifndef USE_MCP4922
#define OUT_A_BIAS D13
#define OUT_B_BIAS D12 // SPIのピンを使用
#endif
#define EXTRA_GATE D9

#define ENC0A D2
#define ENC0B D3
#define ENC1A D6
#define ENC1B D7
#define SW0 D10
#define SW1 D11
#define POT0 A0
#define POT1 A1
