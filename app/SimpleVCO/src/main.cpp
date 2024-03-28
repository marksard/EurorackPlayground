/*!
 * SimpleVCO
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
#include "Oscillator.hpp"

#define USE_MCP4922
#ifdef USE_MCP4922
#include "MCP_DAC.h"
MCP4922 MCP(&SPI1);
#endif

#define SAMPLE_FREQ 32000
#define PWM_RESO 4096
#define SAMPLE_US (1000000.0 / SAMPLE_FREQ)
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define MAX_COARSE_FREQ 550

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static Button buttons[2];
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];
static SmoothAnalogRead vOct;

const static float rateRatio = (float)ADC_RESO / (float)MAX_COARSE_FREQ;
static uint interruptSliceNum;

Oscillator osc[2];

static uint8_t requiresUpdate = 0;

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
    u8g2.setFont(u8g2_font_7x14B_tf);
    //   u8g2.setFlipMode(1);
}

void dispOLED()
{
    static char disp_buf[33] = {0};

    if (!requiresUpdate)
        return;
    requiresUpdate = 0;
    u8g2.clearBuffer();
    sprintf(disp_buf, "%s: %s", osc[0].getWaveName(), osc[0].getNoteName());
    u8g2.drawStr(0, 16, disp_buf);
    sprintf(disp_buf, "%s: %s", osc[1].getWaveName(), osc[1].getNoteName());
    u8g2.drawStr(0, 32, disp_buf);

    u8g2.sendBuffer();
}

void  interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    uint16_t valueA = osc[0].getWaveValue();
    uint16_t valueB = osc[1].getWaveValue();

#ifdef USE_MCP4922
    MCP.fastWriteA(valueA);
    MCP.fastWriteB(valueB);
#else
    pwm_set_gpio_level(OUT_A, valueA);
    pwm_set_gpio_level(OUT_B, valueB);
#endif

    // test
    // digitalWrite(GATE_A, HIGH);
    // sleep_us(1);
    // digitalWrite(GATE_A, LOW);
}


void initPWM(uint gpio, bool interrupt)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    if (interrupt)
    {
        interruptSliceNum = slice;
        pwm_clear_irq(slice);
        pwm_set_irq_enabled(slice, true);
        irq_set_exclusive_handler(PWM_IRQ_WRAP, interruptPWM);
        irq_set_enabled(PWM_IRQ_WRAP, true);
    }

    // 最速設定（可能な限り高い周波数にしてRCの値をあげることなく平滑な電圧を得たい）
    // clockdiv = 125MHz / (PWM_RESO * 欲しいfreq)
    // 欲しいfreq = 125MHz / (PWM_RESO * clockdiv)
    // pwm_set_clkdiv(slice, 125000000.0 / (PWM_RESO * 10000.0));
    // pwm_set_clkdiv(slice, 1);

    pwm_set_wrap(slice, PWM_RESO - 1);
    pwm_set_enabled(slice, true);
    analogWriteFreq(SAMPLE_FREQ);
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
    osc[0].init(SAMPLE_US);
    osc[1].init(SAMPLE_US);
    vOct.init(GATE_B);

    // test
    // pinMode(GATE_A, OUTPUT);

#ifdef USE_MCP4922
    pinMode(PIN_SPI1_SS, OUTPUT);
    MCP.setSPIspeed(20000000);
    MCP.begin(PIN_SPI1_SS);
    initPWM(OUT_A, true);
#else
    initPWM(OUT_A, true);
    initPWM(OUT_B, false);
#endif
}

void loop()
{
    static int16_t voctTune = 118;
    uint16_t voct = vOct.analogRead(false);
    // pulseWidth = pot[1].analogRead(false);
    if (buttons[0].getState() == 3)
    {
        voctTune = (pot[1].analogRead() / ((float)ADC_RESO / 400.0)) - (400 >> 1);
    }
    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    float powVOct = (float)pow(2, map(voct, 0, ADC_RESO - voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);

    uint16_t coarseA = (float)pot[0].analogRead(false) / rateRatio;
    uint16_t freqencyA = coarseA * powVOct;
    osc[0].setFrequency(freqencyA);
    // OLED描画更新でノイズが乗るので必要時以外更新しない
    requiresUpdate |= osc[0].setNoteNameFromFrequency(coarseA);
    requiresUpdate |= osc[0].setWave((Oscillator::Wave)
        constrainCyclic((int)osc[0].getWave() + (int)enc[0].getDirection(true), 0, (int)Oscillator::Wave::MAX));

    uint16_t coarseB = (float)pot[1].analogRead(false) / rateRatio;
    uint16_t freqencyB = coarseB * powVOct;
    osc[1].setFrequency(freqencyB);
    // OLED描画更新でノイズが乗るので必要時以外更新しない
    requiresUpdate |= osc[1].setNoteNameFromFrequency(coarseB);
    requiresUpdate |= osc[1].setWave((Oscillator::Wave)
        constrainCyclic((int)osc[1].getWave() + (int)enc[1].getDirection(true), 0, (int)Oscillator::Wave::MAX));

    static uint8_t dispCount = 0;
    dispCount++;
    if (dispCount == 0)
    {
        Serial.print(voct);
        Serial.print(", ");
        Serial.print(voctTune);
        Serial.print(", ");
        Serial.print(coarseA);
        Serial.print(", ");
        Serial.print(freqencyA);
        Serial.print(", ");
        Serial.print(osc[0].getWaveName());
        Serial.print(", ");
        Serial.println();
    }

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
