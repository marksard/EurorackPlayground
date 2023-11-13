#include <Arduino.h>
#include <hardware/pwm.h>
#include <U8g2lib.h>
#include "../../commonlib/common/Button.hpp"
#include "../../commonlib/common/SmoothAnalogRead.hpp"
#include "../../commonlib/common/RotaryEncoder.hpp"
#include "gpio.h"
#include "StepSeqModel.hpp"
#include "StepSeqView.hpp"
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
        u8g2.drawStr(0, 2, "KEY EDT");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "E1>KEY");
        u8g2.drawStr(52, 8, "E2>GATE");
        u8g2.drawStr(92, 0, "SW>---");
        u8g2.drawStr(92, 8, "P2>STP");
        break;
    case 1:
        u8g2.drawStr(0, 2, "SEQ RND");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "E1>SEQ");
        u8g2.drawStr(52, 8, "E2>RANG");
        u8g2.drawStr(92, 0, "SW>RST");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 2:
        u8g2.drawStr(0, 2, "SEQ MOV");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "E1>MOV");
        u8g2.drawStr(52, 8, "E2>---");
        u8g2.drawStr(92, 0, "SW>---");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 3:
        u8g2.drawStr(0, 2, "RNG G");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "E1>STR");
        u8g2.drawStr(52, 8, "E2>END");
        u8g2.drawStr(92, 0, "SW>---");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 4:
        u8g2.drawStr(0, 2, "RNG KEY");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "E1>STR");
        u8g2.drawStr(52, 8, "E2>END");
        u8g2.drawStr(92, 0, "SW>---");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 5:
        u8g2.drawStr(0, 2, "DIR PLY");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "E1>KEY");
        u8g2.drawStr(52, 8, "E2>GATE");
        u8g2.drawStr(92, 0, "SW>---");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 6:
        u8g2.drawStr(0, 2, "PLY/STP");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "E1>PLY");
        u8g2.drawStr(52, 8, "E2>RST");
        u8g2.drawStr(92, 0, "SW>---");
        u8g2.drawStr(92, 8, "P2>---");
        break;
    case 7:
        u8g2.drawStr(0, 2, "BPM/SCL");
        u8g2.setFont(u8g2_font_5x8_tf);
        u8g2.drawStr(52, 0, "E1>BPM");
        u8g2.drawStr(52, 8, "E2>SCL");
        sprintf(disp_buf, "-->%03d", sspc.getBPM());
        u8g2.drawStr(92, 0, disp_buf);
        sprintf(disp_buf, "-->%s", scale[sspc.getScale()]);
        u8g2.drawStr(92, 8, disp_buf);
        break;
    default:
        u8g2.setFont(u8g2_font_5x8_tf);
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
    initPWM(OUT_A);
    initPWM(OUT_B);

    Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    delay(500);

    sspc.generateTestToneSequence();
    sspc.setBPM(128, 48);
    sspc.start();

    add_repeating_timer_us(-1 * TIMER_INTR_TM, intrTimer, NULL, &timer);
}

void loop()
{
    int8_t enc0 = enc[0].getDirection(true);
    int8_t enc1 = enc[1].getDirection(true);
    uint16_t pot0 = pot[0].analogRead(false);
    uint16_t pot1 = pot[1].analogRead(false);

    // if (buttons[0].getState() == 2)
    // {
    // }
    // if (buttons[1].getState() == 2)
    // {
    // }

    menuIndex = map(pot0, 0, 4040, 0, 7);

    switch (menuIndex)
    {
    case 0:
    {
        uint8_t step = map(pot1, 0, 4040, 0, 15);
        sspc.setSettingPos(step);
        sspc.addNote(enc0);
        sspc.addGate(enc1);
    }
    break;
    case 1:
        if (enc0 != 0)
            sspc.generateSequence();
        if (enc1 != 0)
            sspc.setLimitStepAtRandom();
        if (buttons[0].getState() == 2)
            sspc.reset();
        if (buttons[1].getState() == 2)
            sspc.resetRange();
        break;
    case 2:
        sspc.moveSeq(enc0);
        break;
    case 3:
        sspc.addGateLimit(enc0, enc1);
        break;
    case 4:
        sspc.addKeyLimit(enc0, enc1);
        break;
    case 5:
        sspc.addKeyStepMode(enc0);
        sspc.addGateStepMode(enc1);
        break;
    case 6:
        if (enc0 >= 1)
        {
            sspc.start();
        }
        else if (enc0 < 0)
        {
            sspc.stop();
        }
        if (enc1 >= 1)
        {
            sspc.reset();
        }
        break;
    case 7:
        sspc.addBPM(enc0);
        sspc.addScale(enc1);
    case 8:
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
