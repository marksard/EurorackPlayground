/*!
 * PollingTimeEvent
 * ポーリングベースのイベント検出クラス
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
#include "TriggerInterface.hpp"

class PollingTimeEvent : public TriggerInterface
{
public:
    PollingTimeEvent()
    {
        _start = 0;
        setBPM(133, 4);
    }

    void start() override
    {
        if (_start)
            return;
        _lastMillis = millis();
        _start = 1;
    }

    void stop() override
    {
        _lastMillis = millis();
        _start = 0;
    }

    bool ready() override
    {
        if (!_start)
            return false;

        uint32_t now = millis();
        if ((int)(now - _lastMillis) >= triggerTime)
        {
            // Serial.print(_lastMillis);
            // Serial.print(",");
            // Serial.print(now);
            // Serial.print(",");
            // Serial.println(triggerTime);
            _lastMillis = now;
            return true;
        }

        return false;
    }

    bool isStart() override
    {
        return _start ? true : false;
    }

    void setMills(int millSec) override
    {
        triggerTime = millSec;
    }

    void setBPM(byte bpm, byte bpmReso) override
    {
        if (_bpm == bpm)
            return;
        _bpm = bpm;
        triggerTime = (int)((60.0 / (bpm * bpmReso)) * 1000.0);
    }

    byte getBPM() { return _bpm; }

private:
    byte _start;
    byte _bpm;
    int triggerTime;
    uint32_t _lastMillis;
};
