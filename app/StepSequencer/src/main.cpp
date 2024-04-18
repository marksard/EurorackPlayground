/*!
 * Step Sequencer
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
#include "StepSeqModel.hpp"
#include "StepSeqView.hpp"

#define USE_MCP4922
#ifdef USE_MCP4922
#include "MCP_DAC.h"
MCP4922 MCP(&SPI1);
#endif

#include "StepSeqPlayControl.hpp"

#define TIMER_INTR_TM 62 // us == 16kHz (1/(60sec/(250BPM*3840ticks))
static repeating_timer timer;

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static StepSeqPlayControl sspc(&u8g2);
static Button buttons[2];
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];
static int8_t menuIndex = 0;

static const char *scale[] = {"maj", "dor", "phr", "lyd", "mix", "min", "loc"};

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
    static char disp_buf[10] = {0};

    u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_7x14B_tf);
    switch (menuIndex)
    {
    case 0:
        u8g2.drawStr(0, 2, "BPM/SCL");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "BPM   ");
        u8g2.drawStr(52, 8, "SCALE ");
        sprintf(disp_buf, "-->%03d", sspc.getBPM());
        u8g2.drawStr(92, 0, disp_buf);
        sprintf(disp_buf, "-->%s", scale[sspc.getScale()]);
        u8g2.drawStr(92, 8, disp_buf);
        break;
    case 1:
        u8g2.drawStr(0, 2, "PLY/STP");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "CW:PLY");
        u8g2.drawStr(52, 8, "------");
        u8g2.drawStr(92, 0, "SW:TST");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 2:
        u8g2.drawStr(0, 2, "SEQ RND");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "ADDLEN ");
        u8g2.drawStr(52, 8, "RSTLEN ");
        u8g2.drawStr(92, 0, "SW->RND");
        sprintf(disp_buf,   "LEN>% 1d", sspc.getGateLen());
        u8g2.drawStr(92, 8, disp_buf);
        break;
    case 3:
        u8g2.drawStr(0, 2, "KEY EDT");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "KEY   ");
        u8g2.drawStr(52, 8, "GATE  ");
        u8g2.drawStr(92, 0, "ACC   ");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 4:
        u8g2.drawStr(0, 2, "SEQ MOV");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "MOVE  ");
        u8g2.drawStr(52, 8, "RANGE ");
        u8g2.drawStr(92, 0, "RESET ");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 5:
        u8g2.drawStr(0, 2, "RANGE S");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "KEY S");
        u8g2.drawStr(52, 8, "GATE S");
        u8g2.drawStr(92, 0, "RESET ");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 6:
        u8g2.drawStr(0, 2, "RANGE E");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "KEY  E");
        u8g2.drawStr(52, 8, "GATE E");
        u8g2.drawStr(92, 0, "RESET ");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 7:
        u8g2.drawStr(0, 2, "DIR PLY");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "KEY DIR");
        u8g2.drawStr(52, 8, "GAT DIR");
        u8g2.drawStr(92, 0, "------");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 8:
        u8g2.drawStr(0, 2, "PPQ");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "PPQ   ");
        u8g2.drawStr(52, 8, "------");
        sprintf(disp_buf, "-->%03d", sspc.getPPQ());
        u8g2.drawStr(92, 0, disp_buf);
        u8g2.drawStr(92, 8, "P2>---");
        break;
    default:
        // u8g2.setFont(u8g2_font_5x8_tf);
        break;
    }

    // sprintf(disp_buf, "%s>%03d", "PPQ", sspc.getPPQ());
    // u8g2.drawStr(92, 8, disp_buf);

    sspc.updateDisplay();

    u8g2.sendBuffer();
}

void initPWM(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint potSlice = pwm_gpio_to_slice_num(gpio);
    // 最速設定（可能な限り高い周波数にしてRCの値をあげることなく平滑な電圧を得たい）
    // clockdiv = 125MHz / (PWM_RESO * 欲しいfreq)
    // 欲しいfreq = 125MHz / (PWM_RESO * clockdiv)
    pwm_set_clkdiv(potSlice, 1);
    // pwm_set_clkdiv(potSlice, 125000000.0 / (PWM_RESO * 10000.0));
    pwm_set_wrap(potSlice, PWM_RESO - 1);
    pwm_set_enabled(potSlice, true);
}

bool intrTimer(struct repeating_timer *t)
{
    sspc.updateProcedure();
    // static byte a = 0;
    // a = (a + 1)&1;
    // gpio_put(A2, a ? HIGH : LOW);
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

    pinMode(GATE_A, OUTPUT);
    pinMode(GATE_B, OUTPUT);

#ifdef USE_MCP4922
    pinMode(PIN_SPI1_SS, OUTPUT);
    MCP.setSPIspeed(20000000);
    MCP.begin(PIN_SPI1_SS);
#else
    initPWM(OUT_A);
    initPWM(OUT_B);
#endif

    Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    delay(500);

    // sspc.generateTestToneSequence();
    sspc.requestResetAllSequence();
    sspc.setBPM(128, 48);
    sspc.start();

    add_repeating_timer_us(-1 * TIMER_INTR_TM, intrTimer, NULL, &timer);
}

void loop()
{
    int8_t enc0 = enc[0].getDirection(true);
    int8_t enc1 = enc[1].getDirection(true);
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint16_t pot0 = pot[0].analogRead(false);
    uint16_t pot1 = pot[1].analogRead(false);

    menuIndex = map(pot0, 0, 4040, 0, 8);

    switch (menuIndex)
    {
    case 0:
        sspc.addBPM(enc0);
        sspc.addScale(enc1);
        break;
    case 1:
        if (enc0 >= 1)
        {
            sspc.start();
        }
        else if (enc0 < 0)
        {
            sspc.stop();
            sspc.reset();
        }
        if (btn0 == 2)
        {
            sspc.generateTestToneSequence();
        }
        break;
    case 2:
        sspc.addGateLen(enc0);
        if (enc1 != 0)
        {
            sspc.requestResetGate();
        }        
        if (btn0 == 2)
        {
            sspc.requestGenerateSequence();
        }
        if (btn0 == 4)
        {
            sspc.requestResetAllSequence();
        }
        if (btn1 == 2)
        {
            sspc.setLimitStepAtRandom();
        }
        if (btn1 == 4)
        {
            sspc.resetMode();
            sspc.resetRange();
            sspc.reset();
        }
        break;
    case 3:
    {
        uint8_t step = map(pot1, 0, 4040, 0, 15);
        sspc.setSettingPos(step);
        sspc.addNote(enc0);
        sspc.addGate(enc1);
        if (btn0 == 2)
        {
            sspc.toggleAcc();
        }
    }
    break;
    case 4:
        sspc.moveSeq(enc0);
        sspc.addGateKeyEnd(enc1, enc1);
        if (btn1 == 2)
        {
            sspc.reset();
        }
        break;
    case 5:
        sspc.addGateKeyStart(enc1, enc0);
        if (btn1 == 2)
        {
            sspc.reset();
        }
        break;
    case 6:
        sspc.addGateKeyEnd(enc1, enc0);
        if (btn1 == 2)
        {
            sspc.reset();
        }
        break;
    case 7:
        sspc.addKeyStepMode(enc0);
        sspc.addGateStepMode(enc1);
        break;
    case 8:
        sspc.addPPQ(enc0);
        break;
    default:
        break;
    }

    sleep_ms(1);
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
