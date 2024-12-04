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
#include "StepSeqPlayControl.hpp"

#define CPU_CLOCK 125000000.0
// #define CPU_CLOCK 133000000.0 // 標準めいっぱい
#define SPI_CLOCK 20000000 * 2 // 20MHz上限なんだが24MHzあたりに張り付かせるためにこの数値をセット

#define INTR_PWM_RESO 1024
#define PWM_RESO 4096 // 11bit
#define DAC_MAX_MILLVOLT 5000 // mV
#define ADC_RESO 4096
#define VCO_MAX_COARSE_FREQ 330
#define LFO_MAX_COARSE_FREQ 10

#ifdef USE_MCP4922
#include "MCP_DAC.h"
MCP4922 MCP(&SPI1);
#define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO / 20)
#else
// pwm_set_clkdivの演算で結果的に1
// #define SAMPLE_FREQ (CPU_CLOCK / INTR_PWM_RESO)
#define SAMPLE_FREQ 16000
#endif

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];
static Button buttons[2];

static StepSeqPlayControl sspc(&u8g2);

#define MENU_MAX (16)
static int8_t menuIndex = 0;
static uint8_t requiresUpdate = 1;
PollingTimeEvent updateOLED;

static uint interruptSliceNum;

static const char *scaleNames[] = {"maj", "dor", "phr", "lyd", "mix", "min", "loc", "blu", "spa", "luo"};
static const char *seqSyncModes[] = {"INT", "GATE"};

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
    static char disp_buf[10] = {0};

    if (requiresUpdate)
        u8g2.clearBuffer();

    u8g2.setFont(u8g2_font_7x14B_tf);
    switch (menuIndex)
    {
    case 0:
        u8g2.drawStr(0, 0, "GENERATE :RST<>GEN");
        break;
    case 1:
        sprintf(disp_buf,  "GATE SHFT: %1d", sspc.getGateLen());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 2:
        sprintf(disp_buf,  "OCT  SHFT: %1d", sspc.getOctave());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 3:
        u8g2.drawStr(0, 0, "ROTATE   : L <> R");
        break;
    case 4:
        u8g2.drawStr(0, 0, "STEP LEN :1 to 16");
        break;
    case 5:
        u8g2.drawStr(0, 0, "EDIT NOTE:00to 50");
        break;
    case 6:
        u8g2.drawStr(0, 0, "EDIT GATE: - to G");
        break;
    case 7:
        u8g2.drawStr(0, 0, "EDIT ACC : _ or *");
        break;
    case 8:
        sprintf(disp_buf,  "GEN OCT UNDER: %d", sspc.getOctUnder());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 9:
        sprintf(disp_buf,  "GEN OCT UPPER: %d", sspc.getOctUpper());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 10:
        sprintf(disp_buf,  "GEN GATE MIN : %d", sspc.getGateMin());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 11:
        sprintf(disp_buf,  "GEN GATE MAX : %d", sspc.getGateMax());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 12:
        sprintf(disp_buf,  "GEN GATE INI : %d", sspc.getGateInitial());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 13:
        sprintf(disp_buf,  "BPM : %d", sspc.getBPM());
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 14:
        sprintf(disp_buf,  "SCALE : %s", scaleNames[sspc.getScale()]);
        u8g2.drawStr(0, 0, disp_buf);
        break;
    case 15:
        sprintf(disp_buf,  "SYNC : %s", seqSyncModes[sspc.getClockMode()]);
        u8g2.drawStr(0, 0, disp_buf);
        break;
    default:
        break;
    }

    u8g2.setFont(u8g2_font_5x8_tf);
    sspc.updateDisplay();

    if (requiresUpdate)
        u8g2.sendBuffer();
    else
        u8g2.updateDisplay();
}

void interruptPWM()
{
    pwm_clear_irq(interruptSliceNum);
    // gpio_put(GATE_A, HIGH);

    sspc.updateProcedure();

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

    pot[0].init(POT0);
    pot[1].init(POT1);
    enc[0].init(ENC0A, ENC0B);
    enc[1].init(ENC1A, ENC1B);
    buttons[0].init(SW0);
    buttons[1].init(SW1);

#ifdef USE_MCP4922
    pinMode(PIN_SPI1_SS, OUTPUT);
    MCP.setSPIspeed(SPI_CLOCK);
    // SPI1.begin(); // MCP_DAC 0.5.0 breaking change stop call spi.begin()
    MCP.begin(PIN_SPI1_SS);
#else
    initPWM(OUT_A);
    initPWM(OUT_B);
#endif

    // sspc.generateTestToneSequence();
    sspc.setClockMode(StepSeqPlayControl::CLOCK::EXT);
    sspc.requestResetAllSequence();
    sspc.setBPM(133, 48);
    sspc.start();

    initPWMIntr(PWM_INTR_PIN);
}

void loop()
{
    uint16_t pot0 = pot[0].analogReadDropLow4bit();
    uint16_t pot1 = pot[1].analogReadDropLow4bit();
    int8_t enc0 = enc[0].getDirection(true);
    int8_t enc1 = enc[1].getDirection(true);
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();

    int menu = constrain(menuIndex + enc0, 0, MENU_MAX - 1);
    requiresUpdate |= menuIndex != menu ? 1 : 0;
    menuIndex = menu;

    uint8_t step = map(pot0, 0, 4095, 0, 15);
    sspc.setSettingPos(step);

    if (btn0 == 1)
    {
        if (sspc.isStart())
        {
            sspc.stop();
            sspc.reset();
        }
        else
        {
            sspc.start();
        }
    }

    switch (menuIndex)
    {
    case 0:
        if (enc1 == 1)
        {
            sspc.requestGenerateSequence();
        }
        else if (enc1 == -1)
        {
            sspc.resetAllSequence();
        }
        break;
    case 1:
        sspc.addGateLen(enc1);
        break;
    case 2:
        sspc.addOctave(enc1);
        break;
    case 3:
        sspc.moveSeq(enc1);
        break;
    case 4:
        sspc.addGateKeyEnd(enc1, enc1);
        break;
    case 5:
        sspc.addNote(enc1);
        break;
    case 6:
        sspc.addGate(enc1);
        break;
    case 7:
        if (enc1 == 1)
        {
            sspc.toggleAcc();
        }
        break;
    case 8:
        sspc.addOctUnder(enc1);
        break;
    case 9:
        sspc.addOctUpper(enc1);
        break;
    case 10:
        sspc.addGateMin(enc1);
        break;
    case 11:
        sspc.addGateMax(enc1);
        break;
    case 12:
        sspc.addGateInitial(enc1);
        break;
    case 13:
        sspc.addBPM(enc1);
        break;
    case 14:
        sspc.addScale(enc1);
        break;
    case 15:
        sspc.setClockMode((StepSeqPlayControl::CLOCK)constrain(sspc.getClockMode() + enc1, 0, (int)StepSeqPlayControl::CLOCK::EXT));
        break;
    default:
        break;
    }

    sleep_ms(1);
}

void setup1()
{
    initOLED();
    updateOLED.setMills(33);
    updateOLED.start();
}

void loop1()
{
    if (!updateOLED.ready())
    {
        sleep_ms(1);
        return;
    }

    dispOLED();
}
