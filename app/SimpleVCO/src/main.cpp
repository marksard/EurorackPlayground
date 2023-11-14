/*!
 * SimpleArduinoVCO
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>
#include <hardware/pwm.h>
#include <U8g2lib.h>
#include "../../commonlib/common/Button.hpp"
#include "../../commonlib/common/SmoothAnalogRead.hpp"
#include "../../commonlib/common/RotaryEncoder.hpp"
#include "gpio.h"
#include "note.h"
#include "sine_12bit_4096.h"

#define PWM_RESO 4096
#define PWM_RESO_M1 (PWM_RESO - 1)
#define TIMER_INTR_TM 10      // us == 100kHz
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define ADC_RESO_M1 (ADC_RESO - 1)
#define MAX_COARSE_FREQ 550
#define MAX_FREQ 5000
#define UINT32_MAX_P1 4294967296

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static Button buttons[2];
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];
static SmoothAnalogRead vOct;

static repeating_timer timer;
const static float rateRatio = (float)ADC_RESO / (float)MAX_COARSE_FREQ;

const static float intrruptClock = 1000000.0 / (float)TIMER_INTR_TM; // == 1sec / 10us == 1000000us / 10us == 100kHz
static uint16_t pulseWidth = ADC_RESO / 2;
const static uint16_t halfReso = ADC_RESO / 2;
static uint32_t tuningWordM = 0;
static uint8_t selectWave = 0;
static uint8_t noteNameIndex = 0;
static uint8_t requiresUpdate = 0;

static const char *waveName[] = {"SQUARE", "UP SAW", "TRIANGLE", "SINE", "NONE"};

template <typename vs = int8_t>
vs constrainCyclic(vs value, vs min, vs max)
{
    if (value > max)
        return min;
    if (value < min)
        return max;
    return value;
}

void initOLED()
{
    u8g2.begin();
    u8g2.setContrast(40);
    u8g2.setFontPosTop();
    u8g2.setDrawColor(2);
    //   u8g2.setFlipMode(1);
}

void dispOLED()
{
    static char disp_buf[33] = {0};

    if (!requiresUpdate)
        return;
    requiresUpdate = 0;
    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_7x14B_tf);
    sprintf(disp_buf, "%s : %s", waveName[selectWave], noteName[noteNameIndex]);
    u8g2.drawStr(0, 0, disp_buf);

    u8g2.sendBuffer();
}

void initPWM()
{
    gpio_set_function(OUT_A, GPIO_FUNC_PWM);
    uint potSlice = pwm_gpio_to_slice_num(OUT_A);
    // 最速設定（可能な限り高い周波数にしてRCの値をあげることなく平滑な電圧を得たい）
    // clockdiv = 125MHz / (PWM_RESO * 欲しいfreq)
    // 欲しいfreq = 125MHz / (PWM_RESO * clockdiv)
    pwm_set_clkdiv(potSlice, 1);
    pwm_set_wrap(potSlice, PWM_RESO - 1);
    pwm_set_enabled(potSlice, true);
}

bool intrTimer(struct repeating_timer *t)
{
    static uint32_t phaseAccum = 0;
    phaseAccum = phaseAccum + tuningWordM;
    int index = phaseAccum >> 20;
    uint16_t value = 0;
    switch (selectWave)
    {
    case 0: // SQR
        value = index < pulseWidth ? ADC_RESO_M1 : 0;
        break;
    case 1: // UP SAW
        value = index;
        break;
    case 2: // TRI
        value = index < halfReso ? index * 2 : (ADC_RESO_M1 - index) * 2;
        break;
    case 3: // SINE
        value = sine_12bit_4096[index];
        break;
    default:
        value = 0;
        break;
    }
    pwm_set_gpio_level(OUT_A, value);

    return true;
}

void setup()
{
    analogReadResolution(12);

    enc[0].init(ENC0A, ENC0B);
    enc[1].init(ENC1A, ENC1B);
    pot[0].init(POT0);
    pot[1].init(POT1);
    buttons[0].init(SW0);
    buttons[1].init(SW1);
    vOct.init(GATE_A);

    // 第一引数は負数でコールバック開始-開始間
    add_repeating_timer_us(-1 * TIMER_INTR_TM, intrTimer, NULL, &timer);
    initPWM();
}

void loop()
{
    uint16_t voct = vOct.analogReadDirect();
    uint16_t coarse = (float)pot[0].analogRead(false) / rateRatio;

    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    uint16_t freqency = coarse *
                        (float)pow(2, map(voct, 0, ADC_RESO, 0, DAC_MAX_MILLVOLT) * 0.001);

    tuningWordM = UINT32_MAX_P1 * freqency / intrruptClock;

    pulseWidth = pot[1].analogRead(false);
    selectWave = constrainCyclic(selectWave + (int)enc[0].getDirection(true), 0, 3);

    for (int i = 127; i >= 0; --i)
    {
        if (noteFreq[i] <= (float)coarse)
        {
            noteNameIndex = i;
            break;
        }
    }

    // OLED描画更新でノイズが乗るので必要時以外更新しない
    static uint8_t selectWaveOld = 0;
    static uint8_t noteNameIndexOld = 0;
    if (selectWave != selectWaveOld)
    {
        requiresUpdate = 1;
        selectWaveOld = selectWave;
    }
    if (noteNameIndex != noteNameIndexOld)
    {
        requiresUpdate = 1;
        noteNameIndexOld = noteNameIndex;
    }

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.print(voct);
    //     Serial.print(", ");
    //     Serial.print(coarse);
    //     Serial.print(", ");
    //     Serial.print(freqency);
    //     Serial.print(", ");
    //     Serial.print(selectWave);
    //     Serial.println();
    // }

    sleep_us(50); // 20kHz
}

void setup1()
{
    initOLED();
}

void loop1()
{
    dispOLED();
    sleep_ms(33);
}
