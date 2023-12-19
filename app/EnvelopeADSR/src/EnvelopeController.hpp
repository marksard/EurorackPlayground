/*!
 * Envelope Contoller
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#include "EnvelopeADSR.hpp"

#define MAX_SUSTAIN_LEVEL 4095
#define MAX_ADC_LEVEL 4096.0

class EnvelopeController
{
public:
    EnvelopeController()
    {
    }

    void init(EnvelopeADSR *pEnv)
    {
        setMaxAttackTime(2000);
        setMaxDecayTime(2000);
        setMaxReleaseTime(2000);

        _pEnv = pEnv;
        _lastPot0 = pEnv->getAttack();
        _lastEnc0 = pEnv->getDecay();
        _lastPot1 = pEnv->getSustain();
        _lastEnc1 = pEnv->getRelease();
        _change = false;
        _pot0Lock = true;
        _pot1Lock = true;

        updeteControl(0, 0, 0, 0);
        lock();   
    }

    void lock()
    {
        _pot0Lock = true;
        _pot1Lock = true;
    }

    void setMaxAttackTime(uint16_t value)
    {
        _attackTime = value;
        _attackTimeRatio = value / MAX_ADC_LEVEL;
    }
    void setMaxDecayTime(uint16_t value)
    {
        _decayTime = value;
        _decayTimeRatio = value / MAX_ADC_LEVEL;
    }
    void setMaxReleaseTime(uint16_t value)
    {
        _releaseTime = value;
        _releaseTimeRatio = value / MAX_ADC_LEVEL;
    }

    void updeteControl(uint16_t pot0, uint16_t pot1, int8_t enc0, int8_t enc1)
    {
        if (pot0 >= _lastPot0 - 5 && pot0 < _lastPot0 + 5 && _pot0Lock)
        {
            _pot0Lock = false;
        }
        if (!_pot0Lock)
        {
            _pEnv->setAttack(pot0 * _attackTimeRatio);
            _lastPot0 = pot0;
        }

        if (pot1 >= _lastPot1 - 5 && pot1 < _lastPot1 + 5 && _pot1Lock)
        {
            _pot1Lock = false;
        }
        if (!_pot1Lock)
        {
            _pEnv->setSustain(pot1);
            _lastPot1 = pot1;
        }

        _lastEnc0 = constrain((enc0 * 16) + _lastEnc0, 0, MAX_SUSTAIN_LEVEL);
        _pEnv->setDecay(_lastEnc0 * _decayTimeRatio);

        _lastEnc1 = constrain((enc1 * 16) + _lastEnc1, 0, MAX_SUSTAIN_LEVEL);
        _pEnv->setRelease(_lastEnc1 * _releaseTimeRatio);
    }

    void updateDisp(U8G2 *_pU8g2, uint8_t left, uint8_t top, uint8_t right, uint8_t bottom, uint8_t selected, uint16_t pot0, uint16_t pot1)
    {
        double envWidth = (right - left) / 4; // 幅を4分割
        uint8_t envHeight = bottom - top;
        double attackTime2DispRatio = envWidth / _attackTime;
        double ｌevel2DispRatio = envHeight / MAX_ADC_LEVEL;

        double attack = (double)_pEnv->getAttack() * attackTime2DispRatio;
        double decay = (double)_pEnv->getDecay() * (envWidth / _decayTime);
        double sustain = (double)_pEnv->getSustain() * ｌevel2DispRatio;
        double release = (double)_pEnv->getRelease() * (envWidth / _releaseTime);

        uint8_t envLeft = left;
        uint8_t envRight = left + 8;
        uint8_t envTop = top;
        uint8_t envBottom = bottom;
        uint8_t sustainLevel = envTop + (envBottom - envTop) - sustain;
        uint8_t decayLevel = decay > 0 ? envTop : sustainLevel;

        if (selected)
            _pU8g2->drawBox(envLeft, envTop, envRight - envLeft - 2, envBottom - envTop);

        envLeft = envRight;
        envRight = envLeft + envWidth;
        _pU8g2->drawLine(envRight - attack, envBottom, envRight, decayLevel);
        drawVlineDashed(_pU8g2, envLeft, envTop, envBottom, 2, 4);
        if (selected)
            drawVlineDashed(_pU8g2, envRight - (pot0 * _attackTimeRatio * attackTime2DispRatio), envTop, envBottom, 2, 2);
        envLeft = envRight;
        envRight = envLeft + envWidth;
        _pU8g2->drawLine(envLeft, decayLevel, envLeft + decay, sustainLevel);
        _pU8g2->drawLine(envLeft + decay, sustainLevel, envRight, sustainLevel);
        drawVlineDashed(_pU8g2, envLeft, envTop, envBottom, 2, 4);
        envLeft = envRight;
        envRight = envLeft + envWidth;
        _pU8g2->drawLine(envLeft, sustainLevel, envRight, sustainLevel);
        drawVlineDashed(_pU8g2, envLeft, envTop, envBottom, 2, 4);
        if (selected)
            drawHlineDashed(_pU8g2, envTop + (envBottom - envTop) - (pot1 * ｌevel2DispRatio), envLeft, envRight, 2, 2);

        envLeft = envRight;
        envRight = envLeft + envWidth;
        _pU8g2->drawLine(envLeft, sustainLevel, envLeft + release, envBottom);
        drawVlineDashed(_pU8g2, envLeft, envTop, envBottom, 2, 4);
        drawVlineDashed(_pU8g2, right, envTop, envBottom, 2, 4);
    }

private:
    void drawVlineDashed(U8G2 *_pU8g2, uint8_t x, uint8_t top, uint8_t bottom, uint8_t line, uint8_t space)
    {
        for (byte y = top; y <= bottom; y += line + space)
        {
            _pU8g2->drawVLine(x, y, line);
        }
    }

    void drawHlineDashed(U8G2 *_pU8g2, uint8_t y, uint8_t left, uint8_t right, uint8_t line, uint8_t space)
    {
        for (byte x = left; x <= right; x += line + space)
        {
            _pU8g2->drawHLine(x, y, line);
        }
    }

private:
    EnvelopeADSR *_pEnv;
    uint16_t _lastPot0;
    int16_t _lastEnc0;
    uint16_t _lastPot1;
    uint16_t _lastEnc1;
    uint8_t _change;
    uint8_t _pot0Lock;
    uint8_t _pot1Lock;

    uint16_t _attackTime;
    uint16_t _decayTime;
    uint16_t _releaseTime;
    double _attackTimeRatio;
    double _decayTimeRatio;
    double _releaseTimeRatio;
};
