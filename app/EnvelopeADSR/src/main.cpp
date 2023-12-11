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

#define TIMER_INTR_TM 62 // us == 16kHz (1/(60sec/(250BPM*3840ticks))
static repeating_timer timer;

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
static Button buttons[2];
static SmoothAnalogRead pot[2];
static RotaryEncoder enc[2];
static EnvelopeADSR env[2];

static const double envTime2DispRatio = 27.0 / 2000.0;
static const double envLevel2DispRatio = 22.0 / 4096.0;
static const double envTimeRatio = 2000.0 / 4096.0;

class EnvelopController
{
public:
    EnvelopController()
    {
    }

    void init(EnvelopeADSR *pEnv)
    {
        _pEnv = pEnv;
        _lastAttack = pEnv->getAttack() * envTimeRatio;
        _lastDecay = pEnv->getDecay() * envTimeRatio;
        _lastSuatain = pEnv->getSustain();
        _lastRelease = pEnv->getRelease() * envTimeRatio;
        _change = false;
        _lock = true;
        _envSelect = 0;
        _envSelectLast = 0;
    }

    template <typename su = uint8_t>
    su constrainCyclic(su value, su min, su max)
    {
        if (value > max)
            return min;
        if (value < min)
            return max;
        return value;
    }

    void updeteControll(uint16_t pot, int8_t selectorDelta)
    {
        _envSelect = constrainCyclic(_envSelect + selectorDelta, 0, 3);
        if (_envSelect != _envSelectLast)
        {
            _lock = true;
        }
        _envSelectLast = _envSelect;

        switch (_envSelect)
        {
        case 0:
            if (pot >= _lastAttack - 5 && pot < _lastAttack + 5 && _lock)
            {
                _lock = false;
            }
            if (!_lock)
            {
                _pEnv->setAttack(pot * envTimeRatio);
                _lastAttack = pot;
            }
            break;
        case 1:
            if (pot >= _lastDecay - 5 && pot < _lastDecay + 5 && _lock)
            {
                _lock = false;
            }
            if (!_lock)
            {
                _pEnv->setDecay(pot * envTimeRatio);
                _lastDecay = pot;
            }
            break;
        case 2:
            if (pot >= _lastSuatain - 5 && pot < _lastSuatain + 5 && _lock)
            {
                _lock = false;
            }
            if (!_lock)
            {
                _pEnv->setSustain(pot);
                _lastSuatain = pot;
            }
            break;
        case 3:
            if (pot >= _lastRelease - 5 && pot < _lastRelease + 5 && _lock)
            {
                _lock = false;
            }
            if (!_lock)
            {
                _pEnv->setRelease(pot * envTimeRatio);
                _lastRelease = pot;
            }
            break;

        default:
            break;
        }
    }

    void updateDisp(U8G2 *_pU8g2, uint16_t pot, uint8_t left, uint8_t top, uint8_t right, uint8_t bottom)
    {
        double attack = (double)_pEnv->getAttack() * envTime2DispRatio;
        double decay = (double)_pEnv->getDecay() * envTime2DispRatio;
        double sustain = (double)_pEnv->getSustain() * envLevel2DispRatio;
        double release = (double)_pEnv->getRelease() * envTime2DispRatio;

        uint8_t fontLeft = left;
        left = left + 20;
        // uint8_t left = 28;
        // uint8_t top = 5;
        // uint8_t right = 100;
        // uint8_t bottom = 25;
        uint8_t attackWidth = left + attack;
        uint8_t decayWidth = attackWidth + decay;
        uint8_t sustainLevel = top + (bottom - top) - sustain;
        uint8_t decayLevel = decay > 0 ? top : sustainLevel;
        uint8_t releaseWidth = right - release;
        _pU8g2->drawLine(left, bottom, attackWidth, decayLevel);
        _pU8g2->drawLine(attackWidth, decayLevel, decayWidth, sustainLevel);
        _pU8g2->drawLine(decayWidth, sustainLevel, releaseWidth, sustainLevel);
        _pU8g2->drawLine(releaseWidth, sustainLevel, right, bottom);
        _pU8g2->setFont(u8g2_font_lucasarts_scumm_subtitle_o_tf);

        switch (_envSelect)
        {
        case 0:
            _pU8g2->drawStr(fontLeft, top + 5, "A");
            attack = (double)pot * envTimeRatio * envTime2DispRatio;
            for (byte y = top; y <= bottom; y += 4)
            {
                _pU8g2->drawVLine(left + attack, y, 2);
            }
            break;
        case 1:
            _pU8g2->drawStr(fontLeft, top + 5, "D");
            decay = (double)pot * envTimeRatio * envTime2DispRatio;
            for (byte y = top; y <= bottom; y += 4)
            {
                _pU8g2->drawVLine(attackWidth + decay, y, 2);
            }
            break;
        case 2:
            _pU8g2->drawStr(fontLeft, top + 5, "S");
            sustain = (double)pot * envLevel2DispRatio;
            for (byte x = left; x <= right; x += 4)
            {
                _pU8g2->drawHLine(x, top + (bottom - top) - sustain, 2);
            }
            break;
        case 3:
            _pU8g2->drawStr(fontLeft, top + 5, "R");
            release = (double)pot * envTimeRatio * envTime2DispRatio;
            for (byte y = top; y <= bottom; y += 4)
            {
                _pU8g2->drawVLine(right - release, y, 2);
            }
            break;

        default:
            break;
        }
    }

private:
    EnvelopeADSR *_pEnv;
    uint16_t _lastAttack;
    uint16_t _lastDecay;
    uint16_t _lastSuatain;
    uint16_t _lastRelease;
    uint8_t _change;
    uint8_t _lock;
    int8_t _envSelect;
    int8_t _envSelectLast;
};

EnvelopController envCont[2];

void initOLED()
{
    u8g2.begin();
    u8g2.setContrast(40);
    u8g2.setFontPosTop();
    u8g2.setDrawColor(2);
    //   u8g2.setFlipMode(1);
    // u8g2.setFont(u8g2_font_5x8_tf);
}

void dispOLED()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_8x13B_tf);
    u8g2.drawStr(0, 0, "2 Envelope Gen");
    envCont[0].updateDisp(&u8g2, pot[0].getValue(), 10, 16, 117, 38);
    envCont[1].updateDisp(&u8g2, pot[1].getValue(), 10, 41, 117, 63);

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

    pinMode(GATE_A, INPUT);
    pinMode(GATE_B, INPUT);
    initPWM(OUT_A);
    initPWM(OUT_B);

    Serial.begin(9600);
    // while (!Serial)
    // {
    // }
    delay(500);

    // add_repeating_timer_us(-1 * TIMER_INTR_TM, intrTimer, NULL, &timer);
    env[0].init((double)PWM_RESO);
    env[0].set(0, 400, 2048, 200);
    env[1].init((double)PWM_RESO);
    env[1].set(0, 400, 2048, 200);

    envCont[0].init(&env[0]);
    envCont[1].init(&env[1]);
}

void loop()
{
    int8_t enc0 = enc[0].getDirection(true);
    int8_t enc1 = enc[1].getDirection(true);
    uint8_t btn0 = buttons[0].getState();
    uint8_t btn1 = buttons[1].getState();
    uint16_t pot0 = pot[0].analogRead(true);
    uint16_t pot1 = pot[1].analogRead(true);

    uint8_t gate = digitalRead(GATE_B);
    env[0].next(gate);
    env[1].next(gate);

    envCont[0].updeteControll(pot0, enc0);
    envCont[1].updeteControll(pot1, enc1);

    pwm_set_gpio_level(OUT_A, env[0].getLevel());
    pwm_set_gpio_level(OUT_B, env[1].getLevel());

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
