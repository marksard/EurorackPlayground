/*!
 * Euclid Rhythm Sequencer
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#include <Arduino.h>
#include <hardware/pwm.h>
#include <U8g2lib.h>
#include "../../commonlib/common/Button.hpp"
#include "../../commonlib/common/SmoothAnalogRead.hpp"
#include "../../commonlib/common/RotaryEncoder.hpp"
#include "../../commonlib/common/EdgeChecker.hpp"
#include "../../commonlib/common/TriggerOut.hpp"
#include "../../commonlib/common/PollingTimeEvent.hpp"
#include "../../commonlib/common/Euclidean.hpp"
#include "../../commonlib/ui_common/EuclideanDisp.hpp"
#include "OscilloscopeLite.hpp"
#include "Oscillator.hpp"
#include "gpio.h"

// #define CPU_CLOCK 125000000.0
#define CPU_CLOCK 133000000.0 // 標準めいっぱい
#define SPI_CLOCK 20000000 * 2 // 20MHz上限なんだが24MHzあたりに張り付かせるためにこの数値をセット

#define INTR_PWM_RESO 1024
#define PWM_RESO 4096         // 12bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define LFO_MAX_COARSE_FREQ 99
// static float rateRatio = (float)ADC_RESO / (float)LFO_MAX_COARSE_FREQ;
#define EXP_CURVE(value, ratio) (exp((value * (ratio / (ADC_RESO-1)))) - 1) / (exp(ratio) - 1)

#ifdef USE_MCP4922
#include "MCP_DAC.h"
MCP4922 MCP(&SPI1);
// pwm_set_clkdivの演算で結果的に3
// SPI処理が重めのため3
#define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO / 3)
#else
// pwm_set_clkdivの演算で結果的に1
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO)
#define SAMPLE_FREQ 10000
#endif

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static Button buttons[2];
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];
static int8_t menuIndex = 0;
static EdgeChecker clockEdge;
static TriggerOut triggerOut[2];
static Euclidean euclid[2];
static EuclideanDisp euclidDisp[2];
static SmoothAnalogRead cv;
static OscilloscopeLite oscillo(SAMPLE_FREQ);
static PollingTimeEvent intClock;
static Oscillator intLFO;
static int8_t holdSrc = 0;
static int8_t clockSrc = 0;
static int16_t cvInput = 0;
static int8_t holdTrigger = 0;
static int8_t intOctMax = 1;
static int8_t scaleIndex = 2;
static int8_t requestSeqReset = 0;
static int8_t duration = 20;
static int8_t intPoler = 0;
static const char *scaleNames[] = {"maj", "dor", "phr", "lyd", "mix", "min", "loc", "blu", "spa", "luo"};

static uint interruptSliceNum;

#define MAX_SCALE_KEY 7
#define MAX_SCALES 10
#define MAX_SCALES_M1 (MAX_SCALES - 1)
// スケール
static const uint8_t scales[MAX_SCALES][MAX_SCALE_KEY] =
{
    {0, 2, 4, 5, 7, 9, 11}, // ionian / major
    {0, 2, 3, 5, 7, 9, 10}, // dorian
    {0, 1, 3, 5, 7, 8, 10}, // phrygian
    {0, 2, 4, 6, 7, 9, 11}, // lydian
    {0, 2, 4, 5, 7, 9, 10}, // mixolydian
    {0, 2, 3, 5, 7, 8, 10}, // aeolian / natural minor
    {0, 1, 3, 5, 6, 8, 10}, // locrian
    {0, 2, 3, 4, 7, 9, 0},  // m.blues
    {0, 1, 4, 5, 7, 8, 10}, // spanish
    {0, 2, 4, 7, 9, 0, 2},  // luoyin
};
static float voltPerTone = 4095.0 / 12.0 / 5.0;

bool updateEncWhenGenerate(Euclidean *pEuclid, int8_t enc0, int8_t enc1)
{
    uint8_t onsets = constrain(pEuclid->getOnsets() + enc0, 0, pEuclid->getStepSize());
    uint8_t stepSize = constrain(pEuclid->getStepSize() + enc1, pEuclid->getOnsets(), Euclidean::EUCLID_MAX_STEPS);
    if (pEuclid->getOnsets() == onsets && pEuclid->getStepSize() == stepSize)
    {
        return false;
    }
    pEuclid->generate(onsets, stepSize);
    return true;
}

void initOLED()
{
    u8g2.begin();
    u8g2.setContrast(40);
    u8g2.setFontPosTop();
    // u8g2.setDrawColor(2);
    //   u8g2.setFlipMode(1);
}

void dispOLED()
{
    static char disp_buf[5][32] = {0};

    if (menuIndex < 2)
    {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x14B_tf);
        sprintf(disp_buf[0], "%cA:%02d/%02d %cB:%02d/%02d",
                menuIndex == 0 ? '*' : ' ',
                euclid[0].getOnsets(), euclid[0].getStepSize(),
                menuIndex == 1 ? '*' : ' ',
                euclid[1].getOnsets(), euclid[1].getStepSize());
        u8g2.drawStr(0, 2, disp_buf[0]);
        euclidDisp[0].drawCircle(&u8g2, euclid[0].getStepSize(), euclid[0].getStartPos(), euclid[0].getCurrent(), euclid[0].getSteps());
        euclidDisp[1].drawCircle(&u8g2, euclid[1].getStepSize(), euclid[1].getStartPos(), euclid[1].getCurrent(), euclid[1].getSteps());
        u8g2.sendBuffer();
    }
    else if (menuIndex >= 2 && menuIndex < 7)
    {
        sprintf(disp_buf[0], "%cCLK:%s S&H:%s",
                    menuIndex == 2 ? '*' : ' ',
                    clockSrc == 0 ? "Int" : "Ext",
                    holdSrc == 0 ? "Int" : "Ext");
        sprintf(disp_buf[1], "%ciBPM:%03d iRNG:%d",
                    menuIndex == 3 ? '*' : ' ',
                    intClock.getBPM(),
                    intOctMax);
        sprintf(disp_buf[2], "%cTRIG:%s SCAL:%s",
                menuIndex == 4 ? '*' : ' ',
                holdTrigger == 0 ? "CLK" : holdTrigger == 1 ? " A " : " B ",
                scaleNames[scaleIndex]);
        sprintf(disp_buf[3], "%cTRIG LEN:%03d",
                menuIndex == 5 ? '*' : ' ',
                duration);
        // sprintf(disp_buf[4], "%ciWAV:%s iPOL:%s",
        //         menuIndex == 6 ? '*' : ' ',
        //         intLFO.getWaveName(),
        //         intPoler ? "BI " : "UNI");
        sprintf(disp_buf[4], "%ciWAV:%s FRQ:%3.1f",
                menuIndex == 6 ? '*' : ' ',
                intLFO.getWaveName(),
                intLFO.getFrequency());

        static uint8_t menuSlider = 0;
        menuSlider = map(menuIndex, 0, 8, 0, 3);

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(0, 2, "Euclid/S&H/Quantiz");
        u8g2.drawStr(0, 16, disp_buf[menuSlider]);
        u8g2.drawStr(0, 32, disp_buf[menuSlider + 1]);
        u8g2.drawStr(0, 48, disp_buf[menuSlider + 2]);
        u8g2.sendBuffer();
    }
    else if (menuIndex == 7)
    {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(0, 2, "INT LFO MON");
        oscillo.draw();
        u8g2.sendBuffer();
    }
    else if (menuIndex == 8)
    {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(0, 2, "S&H MON");
        oscillo.draw();
        u8g2.sendBuffer();
    }
    else if (menuIndex == 9)
    {
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(0, 2, "EXT CV MON");
        oscillo.draw();
        u8g2.sendBuffer();
    }
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(GATE_A, HIGH);

    static uint16_t lastCvInput = 0;
    byte trig = 0;
    if (clockSrc == 0)
        trig = intClock.ready();
    else
        trig = clockEdge.isEdgeHigh();

    if (trig)
    {
        int8_t trigA = euclid[0].getNext();
        int8_t trigB = euclid[1].getNext();
        triggerOut[0].update(trigA ? HIGH : LOW);
        triggerOut[1].update(trigB ? HIGH : LOW);

        if ((holdTrigger == 0 && trig) ||
            (holdTrigger == 1 && trigA) ||
            (holdTrigger == 2 && trigB))
        {
            pwm_set_gpio_level(OUT_A, cvInput);
            lastCvInput = cvInput;
        }
    }
    else
    {
        triggerOut[0].update(LOW);
        triggerOut[1].update(LOW);
        if (requestSeqReset)
        {
            euclid[0].resetCurrent();
            euclid[1].resetCurrent();
            requestSeqReset = 0;
        }
    }

    if (menuIndex == 7)
    {
        oscillo.write(intLFO.getValue());
    }
    else if (menuIndex == 8)
    {
        oscillo.write(lastCvInput);
    }
    else if (menuIndex == 9)
    {
        oscillo.write(cv.getValue());
    }

    // pwm_set_gpio_level(OUT_B, intLFO.getWaveValue());
    // gpio_put(GATE_A, LOW);
}

// OUT_A/Bとは違うPWMチャンネルのPWM割り込みにすること
void initPWMIntr(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    interruptSliceNum = slice;
    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, interruptPWM);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // 割り込み頻度
    pwm_set_wrap(slice, INTR_PWM_RESO - 1);
    pwm_set_enabled(slice, true);
    // clockdiv = 125MHz / (INTR_PWM_RESO * 欲しいfreq)
    pwm_set_clkdiv(slice, CPU_CLOCK / (INTR_PWM_RESO * SAMPLE_FREQ));
}

void initPWM(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice, PWM_RESO - 1);
    pwm_set_enabled(slice, true);
    // 最速にして滑らかなPWMを得る
    pwm_set_clkdiv(slice, 1);
}


void setDuration(int8_t duration)
{
    int length = 0;
    if (clockSrc == 0)
        length = intClock.getMills();
    else
        length = clockEdge.getDurationMills();
    length = map(duration, 0, 100, 0, length);
    triggerOut[0].setDuration(length);
    triggerOut[1].setDuration(length);
    // Serial.print(length);
    // Serial.print(",");
    // Serial.print(intClock.getMills());
    // Serial.print(",");
    // Serial.println();
}

void setup()
{
    // Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    // delay(500);
    // Serial.print("started!");

    analogReadResolution(12);

    enc[0].init(ENC0A, ENC0B);
    enc[1].init(ENC1A, ENC1B);
    pot[0].init(POT0);
    pot[1].init(POT1);
    buttons[0].init(SW0);
    buttons[1].init(SW1);
    clockEdge.init(GATE_A);
    triggerOut[0].init(OUT_B);
    // triggerOut[0].init(GATE_A, duration);
    triggerOut[1].init(EXTRA_GATE);
    cv.init(GATE_B); // voct in
    euclidDisp[0].init(32, 40, 21);
    euclidDisp[1].init(32 + 64, 40, 21);
    oscillo.init(&u8g2, 16);
    intClock.setBPM(133,4);
    intClock.start();
    intLFO.init(SAMPLE_FREQ);
    intLFO.setWave(Oscillator::Wave::TRI);
    intLFO.setFrequency(1.0);
    // pinMode(OUT_B_BIAS, OUTPUT);
    // digitalWrite(OUT_B_BIAS, intPoler);
    setDuration(duration);

#ifdef USE_MCP4922
    pinMode(PIN_SPI1_SS, OUTPUT);
    MCP.setSPIspeed(SPI_CLOCK);
    MCP.begin(PIN_SPI1_SS);
#else
    initPWM(OUT_A);
    // initPWM(OUT_B);
#endif

    euclid[0].generate(4, 5);
    euclid[1].generate(4, 16);
    euclidDisp[0].generateCircle(5);
    euclidDisp[1].generateCircle(16);

    initPWMIntr(PWM_INTR_PIN);
}

template <typename vs = int8_t>
vs constrainCyclic(vs value, vs min, vs max)
{
    if (value > max)
        return min;
    if (value < min)
        return max;
    return value;
}

void loop()
{
    int8_t enc0 = enc[0].getDirection(true);
    int8_t enc1 = enc[1].getDirection(true);
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint16_t pot0 = pot[0].analogReadDropLow4bit();
    uint16_t pot1 = pot[1].analogReadDropLow4bit();

    // float coarse = (float)pot1 / rateRatio;
    float coarse = EXP_CURVE((float)pot1, 3.0) * LFO_MAX_COARSE_FREQ;
    float lfoFreq = max(coarse, 0.01);
    intLFO.setFrequency(lfoFreq);
    uint16_t extCV = cv.analogReadDropLow4bit();
    uint16_t intCV = intLFO.getWaveValue();

    menuIndex = map(pot0, 0, 4096, 0, 10);
    int16_t cvIn = 0;
    if (holdSrc)
    {
        cvIn = map(extCV, 0, 4096, 0, 36);
    }
    else {
        cvIn = map(intCV, 0, 4096, 0, (7 * intOctMax) + 1);
    }
    uint8_t oct = cvIn / 7;
    uint8_t semi = scales[scaleIndex][cvIn % 7];
    cvInput = ((oct * 12) + semi) * voltPerTone;

    if (btn1 == 2)
    {
        requestSeqReset = 1;
    }

    if (menuIndex == 0)
    {
        if (updateEncWhenGenerate(&euclid[0], enc0, enc1))
        {
            euclidDisp[0].generateCircle(euclid[0].getStepSize());
        }
    }
    else if (menuIndex == 1)
    {
        if (updateEncWhenGenerate(&euclid[1], enc0, enc1))
        {
            euclidDisp[1].generateCircle(euclid[1].getStepSize());
        }
    }
    else if (menuIndex == 2)
    {
        clockSrc = constrain(clockSrc + enc0, 0, 1);
        holdSrc = constrain(holdSrc + enc1, 0, 1);
    }
    else if (menuIndex == 3)
    {
        uint8_t bpm = constrain(intClock.getBPM() + enc0, 0, 255);
        intClock.setBPM(bpm);
        intOctMax = constrain(intOctMax + enc1, 1, 5);
    }
    else if (menuIndex == 4)
    {
        holdTrigger = constrain(holdTrigger + enc0, 0, 2);
        scaleIndex = constrain(scaleIndex + enc1, 0, MAX_SCALES_M1);
    }
    else if (menuIndex == 5)
    {
        duration = constrain(duration + (enc0 * 2), 2, 100);
    }
    else if (menuIndex == 6)
    {
        intLFO.setWave((Oscillator::Wave)constrain((int)intLFO.getWave() + (int)enc0, 0, (int)Oscillator::Wave::MAX));
        // intPoler = constrain(intPoler + enc1, 0, 1);
        // digitalWrite(OUT_B_BIAS, intPoler);
    }
    else if (menuIndex >= 7 && menuIndex <= 10)
    {
        oscillo.addHorizontalScale(enc0 * -1);
        if (enc1 == 1)
            oscillo.setTrigger(true);
        if (enc1 == -1)
            oscillo.setTrigger(false);
    }

    setDuration(duration);

    sleep_us(100);
}

void setup1()
{
    initOLED();
}

void loop1()
{
    randomSeed(micros());
    dispOLED();
    sleep_ms(33);
}
