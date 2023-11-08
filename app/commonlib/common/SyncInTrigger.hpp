/*!
 * TriggerInterface
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
#include "TriggerInterface.hpp"

class SyncInTrigger : public TriggerInterface
{
public:
    SyncInTrigger(byte pin)
        : SyncInTrigger()
    {
        setPin(pin);
    }

    SyncInTrigger()
    {
        _start = 0;
    }

    void start() override
    {
        pinMode(_pin, INPUT);
        _start = 1;
    }

    void stop() override
    {
        _start = 0;
    }

    bool ready() override
    {
        if (!_start)
            return false;

        static byte valueOld = 0;
        bool result = false;
        byte value = readPin();
        if (value != 0 && valueOld == 0)
        {
            result = true;
        }
        valueOld = value;

        return result;
    }

    bool isStart() override
    {
        return _start ? true : false;
    }

    void setMills(int millSec) override {}
    bool setBPM(byte bpm, byte bpmReso) override
    {
        if (_bpm == bpm && _bpmReso == bpmReso)
            return false;
        _bpm = bpm;
        _bpmReso = bpmReso;
        return true;
    }
    bool setBPM(byte bpm) override { return setBPM(bpm, _bpmReso); }
    byte getBPM() override { return _bpm; }
    byte getBPMReso() override { return _bpmReso; }

    void setPin(byte pin)
    {
        _pin = pin;
    }

protected:
    byte _start;
    byte _pin;
    byte _bpm;
    byte _bpmReso;

    /// @brief ピン値読込
    /// @return
    virtual byte readPin()
    {
        return digitalRead(_pin);
    }
};
