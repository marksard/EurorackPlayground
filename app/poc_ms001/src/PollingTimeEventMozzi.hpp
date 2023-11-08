/*!
 * PollingTimeEventMozzi
 * Copyright 2023 marksard
 */ 

#pragma once

#include <Arduino.h>
#include <EventDelay.h>
#include "../../commonlib/common/TriggerInterface.hpp"

class PollingTimeEventMozzi : public TriggerInterface
{
public:
    PollingTimeEventMozzi()
    {
        _start = 0;
        setBPM(133, 4);
    }

    void start() override
    {
        if(_start)return;
        _ed.start();
        _start = 1;
    }

    void stop() override
    {
        _start = 0;
    }

    bool ready() override
    {
        if (!_start) return false;
        bool result = _ed.ready();
        if (result) _ed.start();
        return result;
    }

    void setMills(int millSec) override
    {
        _ed.set(millSec);
    }

    bool setBPM(byte bpm, byte bpmReso) override
    {
        if (_bpm == bpm && _bpmReso == bpmReso)
            return false;
        _bpm = bpm;
        _bpmReso = bpmReso;
        int triggerTime = (int)((60.0 / (bpm * bpmReso)) * 1000.0);
        _ed.set(triggerTime);
        return true;
    }
    bool setBPM(byte bpm) override { return setBPM(bpm, _bpmReso); }
    byte getBPM() override { return _bpm; }
    byte getBPMReso() override { return _bpmReso; }

private:
    EventDelay _ed;
    byte _start;
    byte _bpm;
    byte _bpmReso;
};
