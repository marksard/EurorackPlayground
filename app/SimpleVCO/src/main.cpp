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
#include "EepromData.h"

// #define CPU_CLOCK 125000000.0
#define CPU_CLOCK 133000000.0 // 標準めいっぱい
#define SPI_CLOCK 20000000 * 2 // 20MHz上限なんだが24MHzあたりに張り付かせるためにこの数値をセット

#define INTR_PWM_RESO 512
#define PWM_RESO 4096         // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO) // 結果的に1になる
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 4) // 73.2khz
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

// static float rateRatio = (float)ADC_RESO / (float)VCO_MAX_COARSE_FREQ;
static float max_coarse_freq = VCO_MAX_COARSE_FREQ;
#define EXP_CURVE(value, ratio) (exp((value * (ratio / (ADC_RESO-1)))) - 1) / (exp(ratio) - 1)

const static float foldRatio = (float)ADC_RESO / (float)100;
const static float waveRatio = (float)ADC_RESO / (float)(Oscillator::Wave::MAX+1);
static uint interruptSliceNum;

Oscillator osc[2];
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

static char modeDisp[2][4] = { "VCO", "LFO" };
static char cvDisp[][4] = { "---", "WAV", "PHS" };
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
        sprintf(disp_buf, "%s A %s", modeDisp[userConfig.rangeMode], cvDisp[externalInMode]);
        u8g2.drawStr(0, 0, disp_buf);
        if (osc[0].getWave() == Oscillator::Wave::PH_RAMP)
        {
            sprintf(disp_buf, "%s p:%02d", osc[0].getNoteNameOrFreq(userConfig.rangeMode), osc[0].getPhaseShift());
        }
        else if (osc[0].getWave() == Oscillator::Wave::TRI ||
        osc[0].getWave() == Oscillator::Wave::SINE)
        {
            sprintf(disp_buf, "%s f:%02d", osc[0].getNoteNameOrFreq(userConfig.rangeMode), osc[0].getFolding());
        }
        else{
            sprintf(disp_buf, "%s", osc[0].getNoteNameOrFreq(userConfig.rangeMode));
        }
        u8g2.drawStr(0, 48, disp_buf);
        u8g2.setFont(u8g2_font_logisoso26_tf);
        sprintf(disp_buf, "%s", osc[0].getWaveName());
        u8g2.drawStr(0, 16, disp_buf);
        break;
    case 1:
        u8g2.setFont(u8g2_font_VCR_OSD_tf);
        sprintf(disp_buf, "%s B %s", modeDisp[userConfig.rangeMode], cvDisp[externalInMode]);
        u8g2.drawStr(0, 0, disp_buf);
        if (osc[1].getWave() == Oscillator::Wave::PH_RAMP)
        {
            sprintf(disp_buf, "%s p:%02d", osc[1].getNoteNameOrFreq(userConfig.rangeMode), osc[1].getPhaseShift());
        }
        else if (osc[1].getWave() == Oscillator::Wave::TRI ||
        osc[1].getWave() == Oscillator::Wave::SINE)
        {
            sprintf(disp_buf, "%s f:%02d", osc[1].getNoteNameOrFreq(userConfig.rangeMode), osc[1].getFolding());
        }
        else{
            sprintf(disp_buf, "%s", osc[1].getNoteNameOrFreq(userConfig.rangeMode));
        }
        u8g2.drawStr(0, 48, disp_buf);
        u8g2.setFont(u8g2_font_logisoso26_tf);
        sprintf(disp_buf, "%s", osc[1].getWaveName());
        u8g2.drawStr(0, 16, disp_buf);
        break;
    case 2:
        u8g2.setFont(u8g2_font_profont22_tf);
        sprintf(disp_buf, "SETTINGS 1");
        u8g2.drawStr(0, 0, disp_buf);
        sprintf(disp_buf, "VCLFO: %s", userConfig.rangeMode ? "ON": "OFF");
        u8g2.drawStr(0, 16, disp_buf);
        sprintf(disp_buf, "CV->: %s", userConfig.cvAssigned ? "B": "A");
        u8g2.drawStr(0, 32, disp_buf);
        break;
    case 3:
        u8g2.setFont(u8g2_font_profont22_tf);
        sprintf(disp_buf, "SETTINGS 2");
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
    // gpio_put(GATE_A, HIGH);
    uint16_t valueA = osc[0].getWaveValue();
    uint16_t valueB = osc[1].getWaveValue();

#ifdef USE_MCP4922
    MCP.fastWriteA(valueA);
    MCP.fastWriteB(valueB);
#else
    pwm_set_gpio_level(OUT_A, valueA);
    pwm_set_gpio_level(OUT_B, valueB);
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
    vOct.init(GATE_B);
    gate.init(GATE_A);
    // pinMode(GATE_A, OUTPUT);

    initEEPROM();
    loadUserConfig(&userConfig);
    osc[0].setWave((Oscillator::Wave)userConfig.oscAWave);
    osc[0].addPhaseShift(userConfig.oscAPhaseShift);
    osc[0].addFolding(userConfig.oscAFolding);
    osc[1].setWave((Oscillator::Wave)userConfig.oscBWave);
    osc[1].addPhaseShift(userConfig.oscBPhaseShift);
    osc[1].addFolding(userConfig.oscBFolding);

    if (userConfig.rangeMode)
    {
        // rateRatio = (float)ADC_RESO / (float)LFO_MAX_COARSE_FREQ;
        max_coarse_freq = (float)LFO_MAX_COARSE_FREQ;
    }
    else
    {
        // rateRatio = (float)ADC_RESO / (float)VCO_MAX_COARSE_FREQ;
        max_coarse_freq = (float)VCO_MAX_COARSE_FREQ;
    }

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
}

void loop()
{
    int8_t enc0 = enc[0].getDirection();
    int8_t enc1 = enc[1].getDirection();
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint16_t pot0 = pot[0].analogRead();
    uint16_t pot1 = pot[1].analogRead();

    static float coarseA = EXP_CURVE((float)pot0, 2.0) * max_coarse_freq;
    static float coarseB = EXP_CURVE((float)pot0, 2.0) * max_coarse_freq;

    // static float coarseA = (float)pot0 / rateRatio;
    // static float coarseB = (float)pot0 / rateRatio;
    static uint16_t lastPot0 = pot0;
    static uint8_t unlock = 0;
    static uint8_t lastMenuIndex = 0;

    uint16_t voct = vOct.analogRead(false);
    uint16_t externalIn = gate.analogRead(false);
    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    float powVOct = (float)pow(2, map(voct, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);
    float freqencyA = max(coarseA * powVOct, 0.01);
    osc[0].setFrequency(freqencyA);
    float freqencyB = max(coarseB * powVOct, 0.01);
    osc[1].setFrequency(freqencyB);


    int8_t updateCVindex = 0;
    if (userConfig.cvAssigned) updateCVindex = 1;

    if (btn0 == 4)
    {
        externalInMode = (externalInMode + 1) % 3;
        requiresUpdate = 1;
    }
    if (externalInMode == 1)
    {
        uint16_t shift = externalIn / waveRatio;
        requiresUpdate |= osc[updateCVindex].setWave((Oscillator::Wave)shift);
    }
    else if (externalInMode == 2)
    {
        uint16_t shift = externalIn / foldRatio;
        requiresUpdate |= osc[updateCVindex].setFolding(shift);
        requiresUpdate |= osc[updateCVindex].setPhaseShift(shift);
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

    menuIndex = map(pot1, 0, 4040, 0, 3);
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
        if (unlock)
        {
            coarseA = EXP_CURVE((float)pot0, 2.0) * max_coarse_freq;
            // coarseA = (float)pot0 / rateRatio;
        }
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        requiresUpdate |= osc[0].setNoteNameFromFrequency(coarseA);
        requiresUpdate |= osc[0].setWave((Oscillator::Wave)
                                             constrainCyclic((int)osc[0].getWave() + (int)enc0, 0, (int)Oscillator::Wave::MAX));
        if (userConfig.rangeMode)
            requiresUpdate |= osc[0].setFreqName(coarseA);

        userConfig.oscAWave = osc[0].getWave();
        if (userConfig.oscAWave == Oscillator::Wave::PH_RAMP)
        {
            osc[0].addPhaseShift((int)enc1);
            requiresUpdate |= userConfig.oscAPhaseShift != osc[0].getPhaseShift();
            userConfig.oscAPhaseShift = osc[0].getPhaseShift();
        }
        else if (userConfig.oscAWave == Oscillator::Wave::TRI ||
        userConfig.oscAWave == Oscillator::Wave::SINE)
        {
            osc[0].addFolding((int)enc1);
            requiresUpdate |= userConfig.oscAFolding != osc[0].getFolding();
            userConfig.oscAFolding = osc[0].getFolding();
        }
    }
    break;
    case 1:
    {
        if (unlock)
        {
            coarseB = EXP_CURVE((float)pot0, 2.0) * max_coarse_freq;
            // coarseB = (float)pot0 / rateRatio;
        }
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        requiresUpdate |= osc[1].setNoteNameFromFrequency(coarseB);
        requiresUpdate |= osc[1].setWave((Oscillator::Wave)
                                             constrainCyclic((int)osc[1].getWave() + (int)enc0, 0, (int)Oscillator::Wave::MAX));
        if (userConfig.rangeMode)
            requiresUpdate |= osc[1].setFreqName(coarseB);

        userConfig.oscBWave = osc[1].getWave();
        if (userConfig.oscBWave == Oscillator::Wave::PH_RAMP)
        {
            osc[1].addPhaseShift((int)enc1);
            requiresUpdate |= userConfig.oscBPhaseShift != osc[1].getPhaseShift();
            userConfig.oscBPhaseShift = osc[1].getPhaseShift();
        }
        else if (userConfig.oscBWave == Oscillator::Wave::TRI ||
        userConfig.oscBWave == Oscillator::Wave::SINE)
        {
            osc[1].addFolding((int)enc1);
            requiresUpdate |= userConfig.oscBFolding != osc[1].getFolding();
            userConfig.oscBFolding = osc[1].getFolding();
        }
    }
    break;
    case 2:
    {
        int8_t mode = constrain((int)userConfig.rangeMode + (int)enc0, 0, 1);
        requiresUpdate |= userConfig.rangeMode != mode;
        userConfig.rangeMode = mode;
        if (userConfig.rangeMode)
        {
            // rateRatio = (float)ADC_RESO / (float)LFO_MAX_COARSE_FREQ;
            max_coarse_freq = (float)LFO_MAX_COARSE_FREQ;
        }
        else
        {
            // rateRatio = (float)ADC_RESO / (float)VCO_MAX_COARSE_FREQ;
            max_coarse_freq = (float)VCO_MAX_COARSE_FREQ;
        }
        int8_t cvb = constrain((int)userConfig.cvAssigned + (int)enc1, 0, 1);
        requiresUpdate |= userConfig.cvAssigned != mode;
        userConfig.cvAssigned = cvb;
    }
    break;
    case 3:
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
