/*!
 * gpio setting
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

// #define USE_MCP4922
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

#define PWM_INTR_PIN D25 // D0/D1ピンとPWMチャンネルがかぶらないように

// rev100改専用
#ifndef USE_MCP4922
#define AC_BIAS D12 // SPIのピンを使用
#define DC_BIAS D13
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
