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
#include "Euclidean.hpp"
#include "EuclideanDisp.hpp"
#include "EzOscilloscope.hpp"
#include "gpio.h"

#define CPU_CLOCK 125000000.0
// #define CPU_CLOCK 133000000.0 // 標準めいっぱい
#define SPI_CLOCK 20000000 * 2 // 20MHz上限なんだが24MHzあたりに張り付かせるためにこの数値をセット

#define INTR_PWM_RESO 1024
#define PWM_RESO 4096         // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define VCO_MAX_COARSE_FREQ 330
#define LFO_MAX_COARSE_FREQ 10

#ifdef USE_MCP4922
#include "MCP_DAC.h"
MCP4922 MCP(&SPI1);
// pwm_set_clkdivの演算で結果的に3
// SPI処理が重めのため3
#define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO / 3)
#else
// pwm_set_clkdivの演算で結果的に1
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO)
#define SAMPLE_FREQ 8000
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
static EzOscilloscope oscillo;
static int8_t shSource = 1;
static int8_t cvInput = 0;
static int8_t shTiming = 0;
static int8_t octMax = 1;
static int8_t scaleIndex = 0;
static int8_t requestSeqReset = 0;
static const char *scaleNames[] = {"maj", "dor", "phr", "lyd", "mix", "min", "loc"};

static uint interruptSliceNum;

#define MAX_SCALE_KEY 7
#define MAX_SCALES 7
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
};
static float voltPerTone = 4095.0 / 12.0 / 5.0;

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
    static char disp_buf[4][32] = {0};

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
        euclidDisp[0].drawCircle(&u8g2, euclid[0].getStepSize(), euclid[0].getCurrent(), euclid[0].getSteps());
        euclidDisp[1].drawCircle(&u8g2, euclid[1].getStepSize(), euclid[1].getCurrent(), euclid[1].getSteps());
        u8g2.sendBuffer();
    }
    else if (menuIndex >= 2 && menuIndex < 4)
    {
        sprintf(disp_buf[0], "%cA:%02d %c t:%02d s:%02d",
                menuIndex == 0 ? '*' : ' ',
                euclid[0].getCurrent(),
                euclid[0].getCurrentTrigger() ? '#' : '-',
                euclid[0].getOnsets(), euclid[0].getStepSize());
        sprintf(disp_buf[1], "%cB:%02d %c t:%02d s:%02d",
                menuIndex == 1 ? '*' : ' ',
                euclid[1].getCurrent(),
                euclid[1].getCurrentTrigger() ? '#' : '-',
                euclid[1].getOnsets(), euclid[1].getStepSize());
        sprintf(disp_buf[2], "%cRANGE:%d SCALE:%s",
                menuIndex == 2 ? '*' : ' ',
                octMax + 1,
                scaleNames[scaleIndex]);
        sprintf(disp_buf[3], "%cS&H:%s TM:%s",
                menuIndex == 3 ? '*' : ' ',
                shSource == 0 ? "Int" : "Ext",
                shTiming == 0 ? "CLK" : shTiming == 1 ? "A"
                                                      : "B");

        static uint8_t menuSlider = 0;
        if (menuIndex == 2)
        {
            menuSlider = 0;
        }
        else if (menuIndex == 3)
        {
            menuSlider = 1;
        }

        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_7x14B_tf);
        u8g2.drawStr(0, 2, "Euclid/S&H Quantiz");
        u8g2.drawStr(0, 16, disp_buf[menuSlider]);
        u8g2.drawStr(0, 32, disp_buf[menuSlider + 1]);
        u8g2.drawStr(0, 48, disp_buf[menuSlider + 2]);
        u8g2.sendBuffer();
    }
    else
    {
        oscillo.play();
    }
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);

    // gpio_put(EXTRA_GATE, HIGH);
    byte trig = clockEdge.isEdgeHigh();
    if (trig)
    {
        int8_t trigA = euclid[0].getNext();
        int8_t trigB = euclid[1].getNext();
        triggerOut[0].update(trigA ? HIGH : LOW);
        triggerOut[1].update(trigB ? HIGH : LOW);

        if ((shTiming == 0 && trig) ||
            (shTiming == 1 && trigA) ||
            (shTiming == 2 && trigB))
        {
            int16_t voct = 0;
            if (shSource == 0)
            {
                uint8_t oct = random(octMax);
                uint8_t semi = scales[scaleIndex][random(MAX_SCALE_KEY)];
                voct = ((oct * 12) + semi) * voltPerTone;
            }
            else
            {
                uint8_t oct = cvInput / 7;
                uint8_t semi = scales[scaleIndex][cvInput % 7];
                voct = ((oct * 12) + semi) * voltPerTone;
            }
            pwm_set_gpio_level(OUT_A, voct);
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
    // gpio_put(EXTRA_GATE, LOW);
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
    triggerOut[0].init(OUT_B, 10);
    triggerOut[1].init(EXTRA_GATE, 10);
    cv.init(GATE_B); // voct in
    euclidDisp[0].init(32, 40, 21);
    euclidDisp[1].init(32 + 64, 40, 21);
    oscillo.init(&u8g2, &cv, 16);

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

    // for (int i = 0; i < 32; i++) {
    //     Serial.print(euclid[0].getNext());
    // }
    // Serial.println();

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
    uint16_t pot0 = pot[0].analogRead(false);
    uint16_t pot1 = pot[1].analogRead(false);

    uint8_t duration = map(pot1, 0, 4096, 0, 100);
    triggerOut[0].setDuration(duration);
    triggerOut[1].setDuration(duration);

    menuIndex = map(pot0, 0, 4096, 0, 5);
    cvInput = map(cv.analogRead(false), 0, 4096, 0, 36);

    if (btn1 == 2)
    {
        requestSeqReset = 1;
    }

    if (menuIndex == 0)
    {
        if (euclid[0].updateEncWhenGenerate(enc0, enc1))
        {
            euclidDisp[0].generateCircle(euclid[0].getStepSize());
        }
    }
    else if (menuIndex == 1)
    {
        if (euclid[1].updateEncWhenGenerate(enc0, enc1))
        {
            euclidDisp[1].generateCircle(euclid[1].getStepSize());
        }
    }
    else if (menuIndex == 2)
    {
        octMax = constrain(octMax + enc0, 0, 4);
        scaleIndex = constrain(scaleIndex + enc1, 0, MAX_SCALES_M1);
    }
    else if (menuIndex == 3)
    {
        shSource = constrain(shSource + enc0, 0, 1);
        shTiming = constrain(shTiming + enc1, 0, 2);
    }
    else if (menuIndex == 4)
    {
        if (enc0 == 1)oscillo.incDelay();
        else if (enc0 == -1)oscillo.decDelay();
    }

    sleep_us(250);
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
