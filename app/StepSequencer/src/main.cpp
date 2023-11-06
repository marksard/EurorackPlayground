#include <Arduino.h>
#include <hardware/pwm.h>
#include <U8g2lib.h>
#include "../../commonlib/common/Button.hpp"
#include "../../commonlib/common/SmoothAnalogRead.hpp"
#include "../../commonlib/common/RotaryEncoder.hpp"
#include "StepSeqModel.hpp"
#include "StepSeqView.hpp"
#include "StepSeqPlayControl.hpp"

#define TIMER_INTR_TM 2400      // us == 400Hz (1/(60sec/(250BPM*96PPQ))
static repeating_timer timer;

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static StepSeqPlayControl sspc(&u8g2);
static Button buttons[2];
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];

#define OUT_A D0
#define GATE_A A3
#define PWM_RESO 4096

#define ENC0A 2
#define ENC0B 3
#define ENC1A 6
#define ENC1B 7
#define SW0 10
#define SW1 11
#define POT0 A0
#define POT1 A1

void initOLED()
{
    u8g2.begin();
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.setContrast(40);
    u8g2.setFontPosTop();
    u8g2.setDrawColor(2);
    //   u8g2.setFlipMode(1);
}


void dispOLED()
{
    u8g2.clearBuffer();
    sspc.updateDisplay();
    u8g2.sendBuffer();
}

void initPWM()
{
    gpio_set_function(OUT_A, GPIO_FUNC_PWM);
    uint potSlice = pwm_gpio_to_slice_num(OUT_A);
    // 31.25kHz ビットは設定による
    pwm_set_clkdiv(potSlice, 1);
    // pwm_set_clkdiv(potSlice, 3.90625);
    pwm_set_wrap(potSlice, PWM_RESO);
    pwm_set_enabled(potSlice, true);
}

bool intrTimer(struct repeating_timer *t)
{
    sspc.updateProcedure();
    return true;
}

void setup()
{
    analogReadResolution(12);

    pinMode(GATE_A, OUTPUT);
    enc[0].init(ENC0A, ENC0B);
    enc[1].init(ENC1A, ENC1B);
    pot[0].init(POT0);
    pot[1].init(POT1);
    buttons[0].init(SW0);
    buttons[1].init(SW1);

    initPWM();

    Serial.begin(9600);
    // while (!Serial)
    // {
    // } // Wait for serial port to connect (for Leonardo boards)
    delay(500);

    sspc.testTone();
    sspc.setBPM(64, 24);
    sspc.start();

    add_repeating_timer_us(-1 * TIMER_INTR_TM, intrTimer, NULL, &timer);
}

void loop()
{
    int8_t enc0 = enc[0].getDirection(true);
    int8_t enc1 = enc[1].getDirection(true);
    uint8_t step = map(pot[0].analogReadDirect(), 0, 4095, 0, 15);
    uint8_t bpm = map(pot[1].analogRead(false), 0, 4096, 0, 255);
    sspc.setSettingPos(step);
    sspc.setBPM(bpm, 24);
    sspc.addGate(enc0);
    sspc.addNote(enc1);

    if (buttons[0].getState() == 2)
    {
        sspc.test();
    }
    if (buttons[1].getState() == 2)
    {
        sspc.testChangeLength();
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
    sleep_ms(1);
}
