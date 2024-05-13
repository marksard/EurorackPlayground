/*!
 * Envelope ADSR
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
#include "EnvelopeADSR.hpp"
#include "EnvelopeController.hpp"

#define CPU_CLOCK 125000000.0
// #define CPU_CLOCK 133000000.0 // 標準めいっぱい
#define SPI_CLOCK 20000000 * 2 // 20MHz上限なんだが24MHzあたりに張り付かせるためにこの数値をセット

#define INTR_PWM_RESO 1024
#define PWM_RESO 2048 // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096

#ifdef USE_MCP4922
#include "MCP_DAC.h"
MCP4922 MCP(&SPI1);
// pwm_set_clkdivの演算で結果的に3
// SPI処理が重めのため3
#define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO / 3)
#else
// pwm_set_clkdivの演算で結果的に1
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO)
#define SAMPLE_FREQ 120000
#endif

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static Button buttons[2];
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];

static uint interruptSliceNum;

static EnvelopeADSR env[2];
EnvelopeController envCont[2];

static uint8_t envSelect = 0;


void initOLED()
{
    u8g2.begin();
    u8g2.setContrast(40);
    u8g2.setFontPosTop();
    u8g2.setDrawColor(2);
    //   u8g2.setFlipMode(1);
    u8g2.setFont(u8g2_font_8x13B_tf);
}

void dispOLED()
{
    u8g2.clearBuffer();
    u8g2.drawStr(0, 0, "Env. Generator");
    envCont[0].updateDisp(&u8g2, 0, 16, 127, 38, envSelect == 0, pot[0].getValue(), pot[1].getValue());
    u8g2.drawLine(0, 40, 127, 40);
    envCont[1].updateDisp(&u8g2, 0, 41, 127, 63, envSelect == 1, pot[0].getValue(), pot[1].getValue());

    u8g2.sendBuffer();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(GATE_A, HIGH);

    uint16_t valueA = env[0].getLevel();
    uint16_t valueB = env[1].getLevel();

#ifdef USE_MCP4922
    MCP.fastWriteA(valueA);
    MCP.fastWriteB(valueB);
    // Serial.println(valueA);
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

    pinMode(GATE_A, INPUT);
    pinMode(GATE_B, INPUT);

    // add_repeating_timer_us(-1 * TIMER_INTR_TM, intrTimer, NULL, &timer);
    env[0].init((double)PWM_RESO);
    env[0].set(5, 1500, 0, 400);
    env[1].init((double)PWM_RESO);
    env[1].set(5, 1500, 2000, 50);

    envCont[0].init(&env[0]);
    envCont[1].init(&env[1]);

#ifdef USE_MCP4922
    pinMode(PIN_SPI1_SS, OUTPUT);
    MCP.setSPIspeed(SPI_CLOCK);
    MCP.begin(PIN_SPI1_SS);
#else
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
    uint16_t pot0 = pot[0].analogRead(true);
    uint16_t pot1 = pot[1].analogRead(true);

    uint8_t gate = digitalRead(GATE_B);
    env[0].next(gate);
    env[1].next(gate);

    if (btn0 == 2)
        envSelect = 0;
    if (btn1 == 2)
        envSelect = 1;

    switch (envSelect)
    {
    case 0:
        envCont[1].lock();
        envCont[0].updeteControl(pot0, pot1, enc0, enc1);
        break;
    case 1:
    default:
        envCont[0].lock();
        envCont[1].updeteControl(pot0, pot1, enc0, enc1);
        break;
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
