#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#include "../../commonlib/common/TriggerInterface.hpp"
#include "../../commonlib/common/PollingTimeEvent.hpp"
#include "gpio.h"
#include "StepSeqModel.hpp"
#include "StepSeqView.hpp"

static float voltPerTone = 4096.0 / 12.0 / 5.0;

class StepSeqPlayControl
{
public:
    StepSeqPlayControl(U8G2 *pU8g2)
        : _ssm(), _ssv(pU8g2, 0, 16)
        , _settingPos(DEF_MAX_STEP_M1, 0, DEF_MAX_STEP_M1)
    {
        _pTrigger = &_polling;

        _seqReadyCount = 0;
        _seqReadyCountMax = 0;
        _seqGateOffStep = 0;
        _ppq = 0;
    }

    void start()
    {
        _pTrigger->start();
    }

    void stop()
    {
        _pTrigger->stop();
    }

    void reset()
    {
        _ssm.keyStep.resetPlayStep();
        _ssm.gateStep.resetPlayStep();
    }

    void setSyncMode()
    {
        bool result = _pTrigger->ready();
        if (result)
        {
        }
    }

    void addBPM(int8_t value)
    {
        if (value == 0) return;
        uint8_t bpm = constrain(_pTrigger->getBPM() + value, 0, 255);
        _pTrigger->setBPM(bpm);
    }

    void setBPM(byte bpm, byte bpmReso)
    {
        if (_pTrigger->setBPM(bpm, bpmReso))
        {
            _seqReadyCountMax = bpmReso / 4;
            _seqGateOffStep = (float)_seqReadyCountMax / 4.0;
        }
    }

    void setPPQ() {}
    uint8_t getBPM() { return _pTrigger->getBPM(); }
    uint8_t getPPQ() { return _ppq; }

    void setSettingPos(int8_t value)
    {
        _settingPos.set(value);
    }

    void addGate(int8_t value)
    {
        uint8_t pos = _settingPos.get();
        uint8_t current = _ssm.getGate(pos);
        _ssm.setGate(pos, 
        (StepSeqModel::Gate)constrain((current + value), 
            StepSeqModel::Gate::_, StepSeqModel::Gate::T));
    }

    void addNote(int8_t value)
    {
        uint8_t pos = _settingPos.get();
        uint8_t currentOct = _ssm.getOctave(pos) * MAX_SCALE_KEY;
        uint8_t currentKey = _ssm.getKey(pos);
        int8_t note = constrain((int8_t)(currentOct + currentKey + value), 0, 35); // 7key*5oct
        _ssm.setOctave(pos, note / MAX_SCALE_KEY);
        _ssm.setKey(pos, note % MAX_SCALE_KEY);
    }

    void addGateLimit(int8_t min, int8_t max)
    {
        _ssm.gateStep.pos.setLimit(_ssm.gateStep.pos.getMin() + min, 
                _ssm.gateStep.pos.getMax() + max);
    }

    void addKeyLimit(int8_t min, int8_t max)
    {
        _ssm.keyStep.pos.setLimit(_ssm.keyStep.pos.getMin() + min, 
                _ssm.keyStep.pos.getMax() + max);
    }

    void addGateStepMode(int8_t value)
    {
        _ssm.gateStep.addMode(value);        
    }

    void addKeyStepMode(int8_t value)
    {
        _ssm.keyStep.addMode(value);        
    }

    void moveSeq(int8_t value)
    {
        if (value == 0) return;
        _ssm.moveSeq(value > 0 ? StepSeqModel::SeqMove::RIGHT : StepSeqModel::SeqMove::LEFT);
    }

    void updateProcedure()
    {
        if (!_pTrigger->ready())
        {
            return;
        }

        if (_seqReadyCount >= _seqReadyCountMax)
        {
            _seqReadyCount = 0;
            _ssm.keyStep.nextPlayStep();
            _ssm.gateStep.nextPlayStep();
        }
        else if (_seqReadyCount > _seqGateOffStep * 3)
        {
        }
        else if (_seqReadyCount >= _seqGateOffStep * 3 &&
                 _ssm.getPlayGate() == StepSeqModel::Gate::L)
        {
            gpio_put(GATE_A, LOW);
        }
        else if (_seqReadyCount >= _seqGateOffStep * 2 &&
                 _ssm.getPlayGate() == StepSeqModel::Gate::H)
        {
            gpio_put(GATE_A, LOW);
        }
        else if (_seqReadyCount >= _seqGateOffStep &&
                 _ssm.getPlayGate() == StepSeqModel::Gate::S)
        {
            gpio_put(GATE_A, LOW);
        }

        if (_seqReadyCount == 0)
        {
            uint16_t voct = _ssm.getPlayNote() * voltPerTone;
            pwm_set_gpio_level(OUT_A, voct);
            pwm_set_gpio_level(OUT_B, voct);
            uint8_t gate = _ssm.getPlayGate() != StepSeqModel::Gate::_ ? HIGH : LOW;
            gpio_put(GATE_A, gate);
        }
        _seqReadyCount++;
    }

    void updateDisplay()
    {
        uint8_t key = _ssm.keyStep.pos.get();
        uint8_t gate = _ssm.gateStep.pos.get();
        uint8_t keyStart = _ssm.keyStep.pos.getMin();
        uint8_t keyEnd = _ssm.keyStep.pos.getMax();
        uint8_t gateStart = _ssm.gateStep.pos.getMin();
        uint8_t gateEnd = _ssm.gateStep.pos.getMax();

        _ssv.dispSteps(keyStart, keyEnd, gateStart, gateEnd, _ssm._octaves, _ssm._keys, (uint8_t*)_ssm._gates);
        _ssv.dispKeyPos(key);
        _ssv.dispGatePos(gate);
        _ssv.dispSettingPos(_settingPos.get());
    }

    void generateTestToneSequence()
    {
        ::generateTestToneSequence(&_ssm);
    }

    void generateSequence()
    {
        _seqReadyCount = 0;
        ::generateSequence(&_ssm);
        _ssm.keyStep.setMode(Step::Mode::Forward);
        _ssm.gateStep.setMode(Step::Mode::Forward);
        _ssm.keyStep.pos.setLimit(0, 16);
        _ssm.gateStep.pos.setLimit(0, 16);
        _ssm.keyStep.resetPlayStep();
        _ssm.gateStep.resetPlayStep();
        _ssm.printSeq();
    }

    void setLimitStepAtRandom()
    {
        _ssm.keyStep.resetPlayStep();
        _ssm.gateStep.resetPlayStep();
        _ssm.keyStep.pos.setLimit(random(0, 3), random(4, 16));
        _ssm.gateStep.pos.setLimit(random(0, 3), random(4, 16));
        _ssm.keyStep.setMode((Step::Mode)random((long)Step::Mode::Max));
        _ssm.gateStep.setMode((Step::Mode)random((long)Step::Mode::Max));
    }

private:
    StepSeqModel _ssm;
    StepSeqView _ssv;
    U8G2 *_pU8g2;
    TriggerInterface *_pTrigger;
    PollingTimeEvent _polling;

    uint8_t _seqReadyCount;
    uint8_t _seqReadyCountMax;
    float _seqGateOffStep;
    LimitValue<int8_t> _settingPos;

    uint8_t _ppq;
};
