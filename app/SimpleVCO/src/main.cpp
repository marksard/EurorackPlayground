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

// #define USE_MCP4922
#ifdef USE_MCP4922
#include "MCP_DAC.h"
MCP4922 MCP(&SPI1);
#endif

#define SAMPLE_FREQ 122070.3125
// #define SAMPLE_FREQ 192000
// #define SAMPLE_FREQ 244140.625
#define PWM_RESO 2048
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define MAX_COARSE_FREQ 550

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static Button buttons[2];
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];
static SmoothAnalogRead vOct;
static SmoothAnalogRead gate;

const static float rateRatio = (float)ADC_RESO / (float)MAX_COARSE_FREQ;
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
        sprintf(disp_buf, "VCO A cv:%d", externalInMode);
        u8g2.drawStr(0, 0, disp_buf);
        if (osc[0].getWave() == Oscillator::Wave::PH_RAMP)
        {
            sprintf(disp_buf, "%s p:%02d", osc[0].getNoteName(), osc[0].getPhaseShift());
        }
        else if (osc[0].getWave() == Oscillator::Wave::TRI ||
        osc[0].getWave() == Oscillator::Wave::SINE)
        {
            sprintf(disp_buf, "%s f:%02d", osc[0].getNoteName(), osc[0].getFolding());
        }
        else{
            sprintf(disp_buf, "%s", osc[0].getNoteName());
        }
        u8g2.drawStr(0, 48, disp_buf);
        u8g2.setFont(u8g2_font_logisoso26_tf);
        sprintf(disp_buf, "%s", osc[0].getWaveName());
        u8g2.drawStr(0, 16, disp_buf);
        break;
    case 1:
        u8g2.setFont(u8g2_font_VCR_OSD_tf);
        sprintf(disp_buf, "VCO B cv:%d", externalInMode);
        u8g2.drawStr(0, 0, disp_buf);
        if (osc[1].getWave() == Oscillator::Wave::PH_RAMP)
        {
            sprintf(disp_buf, "%s p:%02d", osc[1].getNoteName(), osc[1].getPhaseShift());
        }
        else if (osc[1].getWave() == Oscillator::Wave::TRI ||
        osc[1].getWave() == Oscillator::Wave::SINE)
        {
            sprintf(disp_buf, "%s f:%02d", osc[1].getNoteName(), osc[1].getFolding());
        }
        else{
            sprintf(disp_buf, "%s", osc[1].getNoteName());
        }
        u8g2.drawStr(0, 48, disp_buf);
        u8g2.setFont(u8g2_font_logisoso26_tf);
        sprintf(disp_buf, "%s", osc[1].getWaveName());
        u8g2.drawStr(0, 16, disp_buf);
        break;
    case 2:
        u8g2.setFont(u8g2_font_VCR_OSD_tf);
        sprintf(disp_buf, "V/OCT tune");
        u8g2.drawStr(0, 0, disp_buf);
        sprintf(disp_buf, "%d", userConfig.voctTune);
        u8g2.drawStr(0, 48, disp_buf);
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
    // digitalWrite(GATE_A, HIGH);
    uint16_t valueA = osc[0].getWaveValue();
    uint16_t valueB = osc[1].getWaveValue();

#ifdef USE_MCP4922
    MCP.fastWriteA(valueA);
    MCP.fastWriteB(valueB);
#else
    pwm_set_gpio_level(OUT_A, valueA);
    pwm_set_gpio_level(OUT_B, valueB);
#endif
    // digitalWrite(GATE_A, LOW);
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

    // 割り込み回数をRESOの1/4にして、サンプルレートを上げられるように
    uint32_t reso = (PWM_RESO >> 2);
    pwm_set_wrap(slice, reso - 1);
    pwm_set_enabled(slice, true);
    // clockdiv = 125MHz / (PWM_RESO * 欲しいfreq)
    pwm_set_clkdiv(slice, 125000000.0 / (reso * SAMPLE_FREQ));
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

    initPWMIntr(PWM_INTR_PIN);

#ifdef USE_MCP4922
    pinMode(PIN_SPI1_SS, OUTPUT);
    MCP.setSPIspeed(20000000);
    MCP.begin(PIN_SPI1_SS);
#else
    initPWM(OUT_A);
    initPWM(OUT_B);
#endif

    initEEPROM();
    loadUserConfig(&userConfig);
    osc[0].setWave((Oscillator::Wave)userConfig.oscAWave);
    osc[0].addPhaseShift(userConfig.oscAPhaseShift);
    osc[0].addFolding(userConfig.oscAFolding);
    osc[1].setWave((Oscillator::Wave)userConfig.oscBWave);
    osc[1].addPhaseShift(userConfig.oscBPhaseShift);
    osc[1].addFolding(userConfig.oscBFolding);
}

void loop()
{
    int8_t enc0 = enc[0].getDirection(true);
    int8_t enc1 = enc[1].getDirection(true);
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint16_t pot0 = pot[0].analogRead(true);
    uint16_t pot1 = pot[1].analogRead(false);

    static uint16_t coarseA = (float)pot0 / rateRatio;
    static uint16_t coarseB = (float)pot0 / rateRatio;
    static uint16_t lastPot0 = pot0;
    static uint8_t unlock = 0;
    static uint8_t lastMenuIndex = 0;

    uint16_t voct = vOct.analogRead(false);
    uint16_t externalIn = gate.analogRead(false);
    // 0to5VのV/OCTの想定でmap変換。RP2040では抵抗分圧で5V->3.3Vにしておく
    float powVOct = (float)pow(2, map(voct, 0, ADC_RESO - userConfig.voctTune, 0, DAC_MAX_MILLVOLT) * 0.001);
    uint16_t freqencyA = coarseA * powVOct;
    osc[0].setFrequency(freqencyA);
    uint16_t freqencyB = coarseB * powVOct;
    osc[1].setFrequency(freqencyB);


    if (btn0 == 4)
    {
        externalInMode = (externalInMode + 1) % 3;
        requiresUpdate = 1;
    }
    if (externalInMode == 1)
    {
        uint16_t shift = externalIn / waveRatio;
        osc[0].setWave((Oscillator::Wave)shift);
    }
    else if (externalInMode == 2)
    {
        uint16_t shift = externalIn / foldRatio;
        osc[0].setFolding(shift);
        osc[0].setPhaseShift(shift);
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
        if (unlock)
        {
            coarseA = (float)pot0 / rateRatio;
        }
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        requiresUpdate |= osc[0].setNoteNameFromFrequency(coarseA);
        requiresUpdate |= osc[0].setWave((Oscillator::Wave)
                                             constrainCyclic((int)osc[0].getWave() + (int)enc0, 0, (int)Oscillator::Wave::MAX));

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
            coarseB = (float)pot0 / rateRatio;
        }
        // OLED描画更新でノイズが乗るので必要時以外更新しない
        requiresUpdate |= osc[1].setNoteNameFromFrequency(coarseB);
        requiresUpdate |= osc[1].setWave((Oscillator::Wave)
                                             constrainCyclic((int)osc[1].getWave() + (int)enc0, 0, (int)Oscillator::Wave::MAX));

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
        if (unlock)
        {
            int16_t tune = (pot0 / ((float)ADC_RESO / 400.0)) - (400 >> 1);
            requiresUpdate |= tune != userConfig.voctTune;
            userConfig.voctTune = tune;
        }
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

    sleep_us(1);
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
