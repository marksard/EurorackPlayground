/*!
 * Adjuster APP
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
#include "../../commonlib/common/PollingTimeEvent.hpp"
#include "../../commonlib/ui_common/SettingItem.hpp"
#include "../../commonlib/common/EdgeChecker.hpp"
// #include "../../commonlib/common/epmkii_gpio.h"
#include "../../commonlib/common/epmk_basicconfig.h"
#include "../../commonlib/common/pwm_wrapper.h"

#include "OscilloscopeLite.hpp"

// #define USE_MCP4922
#ifdef USE_MCP4922
#define SPI_CLOCK 20000000
#include "MCP_DAC.h"
// #define SPI_HARDWARE
#ifdef SPI_HARDWARE
MCP4922 MCP(&SPI1);
#else
MCP4922 MCP(PIN_SPI1_MOSI, PIN_SPI1_SCK); // MOSI, SCK
#endif
#endif

#define PWM_INTR_PIN D25 // PMW4 B

#define EC1B D3
#define EC1A D2
#define BTN1 D6
#define BTN2 D7
#define BTN3 D10 // Encoder Button

#define GATE A3
#define OUT1 D0 // ac
#define OUT2 D1 // dc
#define POT1 A0
#define VOCT A2

// rev110改専用
#ifndef USE_MCP4922
#define USE_110_kai
#ifdef USE_110_kai
#define OUT_A_BIAS D13
#define OUT_B_BIAS D12 // SPIのピンを使用
#endif
#endif
#define EXTRA_GATE D9

#define PWM_RESO 4096 // 12bit
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 8.0) // 32470.703125khz
static uint interruptSliceNum;

static const float semi2DacRatio = (PWM_RESO - 1) / 60.0;

// 標準インターフェース
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot;
static RotaryEncoder enc;
static Button buttons[3];
static SmoothAnalogRead vOct;
static EdgeChecker gate;

//////////////////////////////////////////
// entries
static int16_t vOctValue = 0;
static int16_t gateValue = 0;
static int16_t potValue = 0;
static int16_t btnAValue = 0;
static int16_t btnBValue = 0;
static int16_t bias = 0;
static int16_t outputValue = 0;
static int16_t outputSemiSelect = 0;
static OscilloscopeLite oscillo(SAMPLE_FREQ);

//////////////////////////////////////////

// 画面周り
#define MENU_MAX (1 + 1)
static int menuIndex = 0;
static uint8_t requiresUpdate = 1;
static uint8_t encMode = 0;
static uint8_t oscDataIndex = 0;
static bool oscBias = 0;
const char oscDataNames[][5] = {"VOCT"};
PollingTimeEvent updateOLED;

const char biasONOFF[][5] = {"OFF", "ON"};

SettingItem16 settings1[] =
    {
        SettingItem16(0, 60, 1, &outputSemiSelect, "sel semi: %d", NULL, 0),
        SettingItem16(0, 1, 1, &bias, "out bias: %s", biasONOFF, 2),
        SettingItem16(0, 4095, 1, &outputValue, "chk out: %d", NULL, 0),
};

SettingItem16 settings2[] =
    {
        SettingItem16(0, 4095, 1, &potValue, "chk pot: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &btnAValue, "chk btnA: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &btnBValue, "chk btnB: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &gateValue, "chk gate: %d", NULL, 0),
        SettingItem16(0, 4095, 1, &vOctValue, "chk voct: %d", NULL, 0),
};

static MenuSection16 menu[] = {
    {"OUTPUT CHK", settings1, sizeof(settings1) / sizeof(settings1[0])},
    {"INPUT CHK", settings2, sizeof(settings2) / sizeof(settings2[0])},
};

static MenuControl16 menuControl(menu, sizeof(menu) / sizeof(menu[0]));
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
        oscillo.draw(encMode, (char *)(oscDataNames[oscDataIndex]), oscBias);
        break;
    default:
        menuControl.draw(&u8g2, encMode);
        break;
    }

    u8g2.sendBuffer();
}

void checkVOct()
{
    // voct誤差表示用
    static int16_t lastVOctValue = 0;
    static bool flag = false;
    static int8_t count = 0;
    static int32_t vOctMean = 0;
    if (vOctValue > lastVOctValue + 30 || vOctValue < lastVOctValue - 30)
    {
        flag = true;
    }
    lastVOctValue = vOctValue;
    if (flag)
    {
        vOctMean += vOctValue;
        count++;
        if (count >= 16)
        {
            vOctMean = vOctMean >> 4;
            Serial.print(vOctMean);
            Serial.println("");
            flag = false;
            count = 0;
            vOctMean = 0;
            // outputSemiSelect++;
            // if (outputSemiSelect > 60)
            // {
            //     outputSemiSelect = 0;
            // }
        }
    }
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(LED1, HIGH);
    switch (oscDataIndex)
    {
    case 0:
        oscillo.write(vOctValue);
        break;
    default:
        break;
    }
    // gpio_put(LED1, LOW);
}

void setup()
{
    Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    // delay(500);

    analogReadResolution(12);

    pot.init(POT1);
    enc.init(EC1A, EC1B, true);
    buttons[0].init(BTN1);
    buttons[1].init(BTN2);
    buttons[2].init(BTN3);
    vOct.init(VOCT);
    gate.init(GATE);

#ifdef USE_MCP4922
    pinMode(PIN_SPI1_SS, OUTPUT);
    MCP.setSPIspeed(SPI_CLOCK);
    MCP.setBufferedMode(true);
    MCP.setGain(1);
    MCP.begin(PIN_SPI1_SS);
#ifdef SPI_HARDWARE
    SPI1.end();
    SPI1.begin();
#endif
#else
#ifdef USE_110_kai
    pinMode(OUT_A_BIAS, OUTPUT);
    pinMode(OUT_B_BIAS, OUTPUT);
    gpio_put(OUT_A_BIAS, LOW);
    gpio_put(OUT_B_BIAS, LOW);
#endif
    initPWM(OUT1, PWM_RESO);
    initPWM(OUT2, PWM_RESO);
#endif

    initPWMIntr(PWM_INTR_PIN, interruptPWM, &interruptSliceNum, SAMPLE_FREQ, INTR_PWM_RESO, CPU_CLOCK);

    oscillo.init(&u8g2, 0);
}

void loop()
{
    enc.getDirection();
    vOctValue = vOct.analogReadDirectFast();
    gateValue = gate.isEdgeHigh();
    potValue = pot.analogRead(false);
    // potValue = pot.analogReadDirectFast();
    // potValue = pot.analogReadDropLow4bit();

    outputValue = bias ? ((PWM_RESO >> 1) - 1) : constrain(((float)outputSemiSelect * semi2DacRatio), 0, PWM_RESO - 1);
    
#ifndef USE_MCP4922
    outputValue = constrain(outputValue - PWMCVDCOutputErrorLUT[outputSemiSelect], 0, PWM_RESO - 1);
    pwm_set_gpio_level(OUT1, outputValue);
    pwm_set_gpio_level(OUT2, outputValue);
#else
    MCP.fastWriteA(outputValue);
    MCP.fastWriteB(outputValue);
#endif

    vOctValue = vOctValue - VOCTInputErrorLUT[vOctValue];

    // static uint8_t dispCount = 0;
    // dispCount++;
    // if (dispCount == 0)
    // {
    //     Serial.print("pot:");
    //     Serial.print(potValue);
    //     Serial.print(" voct:");
    //     Serial.print(vOctValue);
    //     Serial.print(" cv1:");
    //     Serial.print(cv1Value);
    //     Serial.print(" cv2:");
    //     Serial.print(cv2Value);
    //     Serial.println();
    // }

    sleep_us(50);
}

void setup1()
{
    initOLED();
    updateOLED.setMills(33);
    updateOLED.start();
}

void loop1()
{
    int8_t encValue = enc.getValue();
    btnAValue = buttons[0].getState();
    btnBValue = buttons[1].getState();
    uint8_t btnREValue = buttons[2].getState();

    if (btnREValue == 2)
    {
        encMode = (encMode + 1) & 1;
        requiresUpdate |= 1;
    }
    else if (encMode == 0)
    {
        if (menuIndex >= MENU_MAX - 1)
        {
            requiresUpdate |= menuControl.select(encValue);
            if (menuControl.isUnder())
            {
                menuIndex--;
                requiresUpdate = true;
            }
        }
        else
        {
            int menu = 0;
            menu = constrain(menuIndex + encValue, 0, MENU_MAX - 1);
            requiresUpdate |= menuIndex != menu ? 1 : 0;
            menuIndex = menu;
        }

        encValue = 0;
    }

    switch (menuIndex)
    {
    case 0:
        requiresUpdate |= 1;
        if ((btnAValue == 3 && btnBValue == 2) || (btnAValue == 2 && btnBValue == 3))
        {
            oscBias = oscBias ? false : true;
        }
        else if (btnAValue == 2)
        {
            // oscDataIndex = constrainCyclic(oscDataIndex + 1, 0, 2);
        }
        else if (btnBValue == 2)
        {
            oscillo.setTrigger(oscillo.getTrigger() ? false : true);
        }
        oscillo.addVerticalScale(encValue);
        oscillo.setHorizontalScale(map(potValue, 0, ADC_RESO - 1, 0, SNAPSHOT_INDEX_MAX));
        break;
    default:
        if (btnAValue > 0 || btnBValue > 0)
        {
            requiresUpdate |= 1;
        }

        requiresUpdate |= menuControl.addValue2CurrentSetting(encValue);
        requiresUpdate = 1;
        checkVOct();
        break;
    }

    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
}
