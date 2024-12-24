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
#include "../../commonlib/common/EdgeChecker.hpp"
#include "gpio.h"
#include "Oscillator.hpp"
#include "EepromData.h"

// #define CPU_CLOCK 125000000.0
#define CPU_CLOCK 133000000.0 // 標準めいっぱい
#define SPI_CLOCK 20000000 * 2 // 20MHz上限なんだが24MHzあたりに張り付かせるためにこの数値をセット

#define INTR_PWM_RESO 512
#define PWM_RESO 1024         // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO) // 結果的に1になる
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 4) // 64941.40625khz
// #define SAMPLE_FREQ 88200

#define VCO_MAX_COARSE_FREQ 330
#define LFO_MAX_COARSE_FREQ 66

#ifdef USE_MCP4922
#include "MCP_DAC.h"
MCP4922 MCP(&SPI1);
#endif

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static Button buttons[2];
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];
static SmoothAnalogRead vOct;
static SmoothAnalogRead gate;
static EdgeChecker extraGate;

// static float rateRatio = (float)ADC_RESO / (float)VCO_MAX_COARSE_FREQ;
static float max_coarse_freq = VCO_MAX_COARSE_FREQ;
#define EXP_CURVE(value, ratio) (exp((value * (ratio / (ADC_RESO-1)))) - 1) / (exp(ratio) - 1)

const static float foldRatio = (float)ADC_RESO / (float)100;
const static float waveRatio = (float)ADC_RESO / (float)(Oscillator::Wave::MAX+1);
static uint interruptSliceNum;

Oscillator osc[4];
static int8_t arpStep = 0;
static int menuIndex = 0;
static uint8_t externalInMode = 0;
static bool saveConfirm = false;
static UserConfig userConfig;

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
    //   u8g2.setFlipMode(1);
}

static char cvDisp[][4] = { "---", "PHS" };
void dispOLED()
{
    static char disp_buf[33] = {0};

    if (!requiresUpdate)
        return;
    requiresUpdate = 0;
    u8g2.clearBuffer();
    switch (menuIndex)
    {
    case 0:
        u8g2.setFont(u8g2_font_VCR_OSD_tf);
        sprintf(disp_buf, "Chord %s", cvDisp[externalInMode]);
        u8g2.drawStr(0, 0, disp_buf);
        if (osc[0].getWave() == Oscillator::Wave::SAW || osc[0].getWave() == Oscillator::Wave::MUL_TRI)
        {
            sprintf(disp_buf, "%s p:%02d", osc[0].getNoteNameOrFreq(false), osc[0].getPhaseShift());
        }
        else if (osc[0].getWave() == Oscillator::Wave::TRI)
        {
            sprintf(disp_buf, "%s f:%02d", osc[0].getNoteNameOrFreq(false), osc[0].getFolding());
        }
        else{
            sprintf(disp_buf, "%s", osc[0].getNoteNameOrFreq(false));
        }
        u8g2.drawStr(0, 48, disp_buf);
        u8g2.setFont(u8g2_font_logisoso26_tf);
        sprintf(disp_buf, "%s", osc[0].getWaveName());
        u8g2.drawStr(0, 16, disp_buf);
        break;
    case 1:
        u8g2.setFont(u8g2_font_profont22_tf);
        sprintf(disp_buf, "SETTINGS 1");
        u8g2.drawStr(0, 0, disp_buf);
        sprintf(disp_buf, "VOtune:%d", userConfig.voctTune);
        u8g2.drawStr(0, 16, disp_buf);
        sprintf(disp_buf, "BIAS: %s", userConfig.biasMode ? "UNI": "BI");
        u8g2.drawStr(0, 32, disp_buf);
        break;
    }

    if (saveConfirm)
    {
        u8g2.setDrawColor(0);
        u8g2.drawBox(0, 0, 128, 40);
        u8g2.setDrawColor(2);
        u8g2.setFont(u8g2_font_VCR_OSD_mf);
        sprintf(disp_buf, "SAVE?");
        u8g2.drawStr(0, 0, disp_buf);
        sprintf(disp_buf, "Y:Dwn N:Up");
        u8g2.drawStr(0, 16, disp_buf);
    }

    u8g2.sendBuffer();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    uint32_t values[4] = {0};
    // gpio_put(GATE_A, HIGH);
    values[0] = osc[0].getWaveValue();
    values[1] = osc[1].getWaveValue();
    values[2] = osc[2].getWaveValue();
    values[3] = osc[3].getWaveValue();
    uint32_t value = (values[0] + values[1] + values[2] + values[3]) >> 2;
    value *= (osc[0].getWave() == Oscillator::Wave::SQU) ? 0.9 : 1.2;
#ifdef USE_MCP4922
    MCP.fastWriteA(valueA);
    MCP.fastWriteB(valueB);
#else
    pwm_set_gpio_level(OUT_A, value);
    pwm_set_gpio_level(OUT_B, values[arpStep]);
#endif
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

void setup()
{
    // Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    // delay(500);

    analogReadResolution(12);

    enc[0].init(ENC0A, ENC0B);
    enc[1].init(ENC1A, ENC1B);
    pot[0].init(POT0);
    pot[1].init(POT1);
    buttons[0].init(SW0);
    buttons[1].init(SW1);
    osc[0].init(SAMPLE_FREQ);
    osc[1].init(SAMPLE_FREQ);
    osc[2].init(SAMPLE_FREQ);
    osc[3].init(SAMPLE_FREQ);
    vOct.init(GATE_B);
    gate.init(GATE_A);
    extraGate.init(EXTRA_GATE);

    initEEPROM();
    loadUserConfig(&userConfig);
    for (int i = 0; i < 4; ++i)
    {
        osc[i].setWave((Oscillator::Wave)userConfig.oscWave);
        osc[i].addPhaseShift(userConfig.oscPhaseShift);
        osc[i].addFolding(userConfig.oscFolding);
    }

    max_coarse_freq = (float)VCO_MAX_COARSE_FREQ;

#ifdef USE_MCP4922
    pinMode(PIN_SPI1_SS, OUTPUT);
    MCP.setSPIspeed(SPI_CLOCK);
    MCP.begin(PIN_SPI1_SS);
#else
    pinMode(OUT_A_BIAS, OUTPUT);
    pinMode(OUT_B_BIAS, OUTPUT);
    digitalWrite(OUT_A_BIAS, userConfig.biasMode ? LOW : HIGH);
    digitalWrite(OUT_B_BIAS, userConfig.biasMode ? LOW : HIGH);

    initPWM(OUT_A);
    initPWM(OUT_B);
#endif

    initPWMIntr(PWM_INTR_PIN);

    osc[0].startFolding(true);
    osc[1].startFolding(true);
    osc[2].startFolding(true);
    osc[3].startFolding(true);
}

static uint8_t addRootScale[7][4] = 
{
    {0, 3, 7, 10}, // Im7
    {0, 3, 6, 10}, // IIdim
    {0, 4, 7, 11}, // IIIM7
    {0, 3, 7, 10}, // IVm7
    {0, 3, 7, 10}, // Vm7
    {0, 4, 7, 11}, // VIM7
    {0, 4, 7, 10}, // VII7
};

static uint8_t routeScaleIndexFromSemitone[] = 
{
    0,0,1,2,0,3,0,4,5,0,0,6
};

// static uint8_t addRootScale[7][4] = 
// {
//     {0, 3, 7, 11}, // ImM7
//     {0, 3, 6, 10}, // IIdim
//     {0, 4, 8, 11}, // IIIM7#5
//     {0, 3, 7, 10}, // IVm7
//     {0, 4, 7, 10}, // V7
//     {0, 4, 7, 11}, // VIM7
//     {0, 3, 6,  9}, // VIIdim
// };

// static uint8_t routeScaleIndexFromSemitone[] = 
// {
//     0,0,1,2,0,3,0,4,5,0,6,0
// };


void loop()
{
    int8_t enc0 = enc[0].getDirection();
    int8_t enc1 = enc[1].getDirection();
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint16_t pot0 = pot[0].analogRead();
    uint16_t pot1 = pot[1].analogRead();

    static float coarse = {0};
    coarse = EXP_CURVE((float)pot0, 2.0) * max_coarse_freq;

    static uint16_t lastPot0 = pot0;
    static uint8_t unlock = 0;
    static uint8_t lastMenuIndex = 0;

    uint16_t voct = vOct.analogRead(false);
    uint16_t externalIn = gate.analogRead(false);
    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    float powVOct = (float)pow(2, map(voct, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);

    static uint8_t freqIndexOld = 0;
    uint8_t courceIndex = osc[0].getNoteNameIndexFromFreq(coarse);
    float freq = coarse * powVOct;
    uint8_t freqIndex = osc[0].getNoteNameIndexFromFreq(freq);
    uint8_t rootDiff = (freqIndex - courceIndex) % 12;
    uint8_t scaleIndex = routeScaleIndexFromSemitone[rootDiff];

    osc[0].setFrequencyFromNoteNameIndex(freqIndex + addRootScale[scaleIndex][0]);
    osc[1].setFrequencyFromNoteNameIndex(freqIndex + addRootScale[scaleIndex][1]);
    osc[2].setFrequencyFromNoteNameIndex(freqIndex + addRootScale[scaleIndex][2]);
    osc[3].setFrequencyFromNoteNameIndex(freqIndex + addRootScale[scaleIndex][3]);

    if (freqIndex != freqIndexOld)
    {
        arpStep = 0;
        freqIndexOld = freqIndex;
    }
    if (extraGate.isEdgeHigh())
    {
        arpStep = (arpStep + 1) % 4;
    }

    if (btn0 == 4)
    {
        externalInMode = (externalInMode + 1) % 2;
        requiresUpdate = 1;
    }
    if (externalInMode == 1)
    {
        uint16_t shift = externalIn / foldRatio;
        requiresUpdate |= osc[0].setFolding(shift);
        requiresUpdate |= osc[0].setPhaseShift(shift);
    }

    // メニュー変更時のポットロック・ロック解除
    if (!unlock)
    {
        if (lastPot0 + 10 < pot0 || lastPot0 - 10 > pot0)
        {
            unlock = 1;
        }
    }
    else
    {
        lastPot0 = pot0;
    }

    menuIndex = map(pot1, 0, 4040, 0, 2);
    if (lastMenuIndex != menuIndex)
    {
        unlock = 0;
        requiresUpdate = 1;
    }
    lastMenuIndex = menuIndex;

    switch (menuIndex)
    {
    case 0:
    {
        for (int i = 0; i < 4; ++i)
        {
            // OLED描画更新でノイズが乗るので必要時以外更新しない
            requiresUpdate |= osc[i].setNoteNameFromFrequency(coarse);
            requiresUpdate |= osc[i].setWave((Oscillator::Wave)
                                                constrainCyclic((int)osc[i].getWave() + (int)enc0, 0, (int)Oscillator::Wave::MAX));
            requiresUpdate |= osc[i].setFreqName(coarse);

            userConfig.oscWave = osc[i].getWave();
            if (userConfig.oscWave == Oscillator::Wave::SAW || userConfig.oscWave == Oscillator::Wave::MUL_TRI)
            {
                osc[i].addPhaseShift((int)enc1);
                // osc[i].setPhaseShift(osc[i].getPhaseShift() + osc[i].getRandom16(5));
                requiresUpdate |= userConfig.oscPhaseShift != osc[i].getPhaseShift();
                userConfig.oscPhaseShift = osc[i].getPhaseShift();
            }
            else if (userConfig.oscWave == Oscillator::Wave::TRI)
            {
                osc[i].addFolding((int)enc1);
                requiresUpdate |= userConfig.oscFolding != osc[i].getFolding();
                userConfig.oscFolding = osc[i].getFolding();
            }

        }
    }
    break;
    case 1:
    {
        int16_t tune = constrain(userConfig.voctTune + (int)enc0, -200, 200);
        requiresUpdate |= tune != userConfig.voctTune;
        userConfig.voctTune = tune;
#ifndef USE_MCP4922
        int8_t bias = constrain(userConfig.biasMode + (int)enc1, 0, 1);
        requiresUpdate |= userConfig.biasMode != bias;
        if (userConfig.biasMode != bias)
        {
            digitalWrite(OUT_A_BIAS, bias ? LOW : HIGH);
            digitalWrite(OUT_B_BIAS, bias ? LOW : HIGH);
        }
        userConfig.biasMode = bias;
#endif
    }
    break;
    }

    if (btn1 == 4)
    {
        saveConfirm = true;
        requiresUpdate |= 1;
    }
    if (saveConfirm)
    {
        if (btn1 == 2)
        {
            saveUserConfig(&userConfig);
            saveConfirm = false;
            requiresUpdate |= 1;
        }
        else if (btn0 == 2)
        {
            saveConfirm = false;
            requiresUpdate |= 1;
        }
    }

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.print(externalIn);
    //     Serial.print(", ");
    //     Serial.print(voct);
    //     Serial.print(", ");
    //     Serial.print(userConfig.voctTune);
    //     Serial.print(", ");
    //     Serial.print(coarseA);
    //     Serial.print(", ");
    //     Serial.print(freqencyA);
    //     Serial.print(", ");
    //     Serial.print(osc[0].getWaveName());
    //     Serial.print(", ");
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
