/*!
 * StepSeqPlayControl
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>
#include <U8g2lib.h>
#include "../../commonlib/common/TriggerInterface.hpp"
#include "../../commonlib/common/PollingTimeEvent.hpp"
#include "../../commonlib/common/SyncInTrigger.hpp"
#include "gpio.h"
#include "StepSeqModel.hpp"
#include "StepSeqView.hpp"

static float voltPerTone = 4095.0 / 12.0 / 5.0;

#ifdef USE_MCP4922
#include "MCP_DAC.h"
extern MCP4922 MCP;
#endif
class StepSeqPlayControl
{
public:
    enum CLOCK
    {
        INT = 0,
        EXT,
        IGNORE,
    };

public:
    StepSeqPlayControl(U8G2 *pU8g2)
        : _ssm(), _ssv(pU8g2, 0, 16)
        , _settingPos(DEF_MAX_STEP_M1, 0, DEF_MAX_STEP_M1)
    {
        _pTrigger = NULL;
        _clock = CLOCK::IGNORE;
        _seqReadyCount = 0;
        _seqReadyCountMax = 0;
        _syncCount = 0;
        _seqGateOffStep = 0;
        _ppq = 4;
        _requestGenerateSequence = false;
        _requestResetAllSequence = false;
        _requestResetGate = false;
        _auto_generative = false;

        // setClockMode(CLOCK::INT);
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
        _seqReadyCount = 0;
        _syncCount = 0;
        _ssm.keyStep.resetPlayStep();
        _ssm.gateStep.resetPlayStep();
    }

    void resetRange()
    {
        _ssm.keyStep.pos.setLimit(0, 16);
        _ssm.gateStep.pos.setLimit(0, 16);
    }

    void resetMode()
    {
        _ssm.keyStep.setMode(Step::Mode::Forward);
        _ssm.gateStep.setMode(Step::Mode::Forward);
    }

    void setClockMode(CLOCK clock)
    {
        if (_clock == clock) return;

        _clock = clock;
        if (clock == CLOCK::INT)
        {
            pinMode(GATE_B, OUTPUT);
            if (_pTrigger != NULL)
            {
                _seqReadyCountMax = 48 / 4;
                _seqGateOffStep = (float)_seqReadyCountMax / 4.0;
                if (_pTrigger->isStart())
                {
                    _polling.start();
                }
            }
            _pTrigger = &_polling;
        }
        // else {
        //     _syncIn.setPin(GATE_B);
        //     if (_pTrigger != NULL)
        //     {
        //         _seqReadyCountMax = 4 / 4;
        //         _seqGateOffStep = (float)_seqReadyCountMax / 4.0;
        //         if (_pTrigger->isStart())
        //         {
        //             _syncIn.start();
        //         }
        //     }
        //     _pTrigger = &_syncIn;
        // }
    }

    CLOCK getClockMode() { return _clock; }

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

    uint8_t getBPM() { return _pTrigger->getBPM(); }
    int8_t getScale() { return _ssm._scaleIndex.get(); }


    void addPPQ(int8_t value)
    {
        if (value == 0) return;
        uint8_t ppq = constrain(_ppq + value, 1 , 4);
        _ppq = ppq;
    }

    void setPPQ(uint8_t value) { _ppq = value;}
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
            StepSeqModel::Gate::_, StepSeqModel::Gate::G));
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

    void toggleAcc()
    {
        uint8_t pos = _settingPos.get();
        uint8_t current = _ssm.getAcc(pos);
        _ssm.setAcc(pos, (current + 1) & 1);
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

    void addGateKeyStart(int8_t gate, int8_t key)
    {
        _ssm.gateStep.pos.setLimit(_ssm.gateStep.pos.getMin() + gate,
                _ssm.gateStep.pos.getMax());
        _ssm.keyStep.pos.setLimit(_ssm.keyStep.pos.getMin() + key,
                _ssm.keyStep.pos.getMax());
    }

    void addGateKeyEnd(int8_t gate, int8_t key)
    {
        _ssm.gateStep.pos.setLimit(_ssm.gateStep.pos.getMin(),
                _ssm.gateStep.pos.getMax() + gate);
        _ssm.keyStep.pos.setLimit(_ssm.keyStep.pos.getMin(),
                _ssm.keyStep.pos.getMax() + key);
    }

    void addGateStepMode(int8_t value)
    {
        _ssm.gateStep.addMode(value);
    }

    void addKeyStepMode(int8_t value)
    {
        _ssm.keyStep.addMode(value);
    }

    void addScale(int8_t value)
    {
        _ssm._scaleIndex.add(value);
    }

    void moveSeq(int8_t value)
    {
        if (value == 0) return;
        _ssm.moveSeq(value > 0 ? StepSeqModel::SeqMove::RIGHT : StepSeqModel::SeqMove::LEFT);
    }

    void addGateLen(int8_t value)
    {
        _ssm.gateLenAdder.add(value);
    }

    int8_t getGateLen()
    {
        return _ssm.gateLenAdder.get();
    }

    void addOctave(int8_t value)
    {
        _ssm.octaveAdder.add(value);
    }

    int8_t getOctave()
    {
        return _ssm.octaveAdder.get();
    }

    void setAutoGenerative(int8_t value)
    {
        _auto_generative = value ? true : false;
    }

    int8_t getAutoGenerative()
    {
        return _auto_generative ? 1 : 0;
    }

    void setSyncOut(int8_t value)
    {
        if (_clock == CLOCK::INT)
        {
            gpio_put(GATE_B, value);
        }
    }

    void setAcc(int8_t value)
    {
        uint16_t out = value ? 4095 : 0;
#ifdef USE_MCP4922
            MCP.fastWriteB(value);
#else
            pwm_set_gpio_level(OUT_B, value);
#endif
    }

    void setVOct(uint16_t value)
    {
#ifdef USE_MCP4922
            MCP.fastWriteA(value);
#else
            pwm_set_gpio_level(OUT_A, voct);
#endif
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
            setSyncOut(LOW);
        }
        else if (_seqReadyCount >= _seqGateOffStep * 3 &&
                 _ssm.getPlayGate() == StepSeqModel::Gate::L)
        {
            gpio_put(GATE_A, LOW);
            setSyncOut(LOW);
            setAcc(LOW);
        }
        else if (_seqReadyCount >= _seqGateOffStep * 2 &&
                 _ssm.getPlayGate() == StepSeqModel::Gate::H)
        {
            gpio_put(GATE_A, LOW);
            setSyncOut(LOW);
            setAcc(LOW);
        }
        else if (_seqReadyCount >= _seqGateOffStep &&
                 _ssm.getPlayGate() == StepSeqModel::Gate::S)
        {
            gpio_put(GATE_A, LOW);
            setSyncOut(LOW);
            setAcc(LOW);
        }

        if (_seqReadyCount == 0)
        {
            if (_ssm.gateStep.pos.getMin() == _ssm.gateStep.pos.get())
            {
                if (_requestGenerateSequence)
                {
                    _requestGenerateSequence = false;
                    generateSequence();
                }
                if (_requestResetAllSequence)
                {
                    _requestResetAllSequence = false;
                    resetAllSequence();
                }
                if (_requestResetGate)
                {
                    _requestResetGate = false;
                    resetGate();
                }
                if (_auto_generative)
                {
                    generateSequence(false);
                    setLimitStepAtRandom();
                }
            }

            uint16_t voct = _ssm.getPlayNote() * voltPerTone;
            setVOct(voct);
            uint8_t gate = _ssm.getPlayGate() != StepSeqModel::Gate::_ ? HIGH : LOW;
            gpio_put(GATE_A, gate);
            _syncCount--;
            if (_syncCount < 0)
            {
                setSyncOut(HIGH);
                _syncCount = 4 - _ppq;
            }

            if (_ssm.getPlayAcc())
            {
                setAcc(HIGH);
            }
            else {
                setAcc(LOW);
            }
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

        _ssv.dispSteps(keyStart, keyEnd, gateStart, gateEnd, _ssm._octaves, _ssm._keys, (uint8_t*)_ssm._gates, (uint8_t*)_ssm._accs, _ssm.gateLenAdder.get(), _ssm.octaveAdder.get());
        _ssv.dispKeyPos(key);
        _ssv.dispGatePos(gate);
        _ssv.dispSettingPos(_settingPos.get());
    }

    void generateTestToneSequence()
    {
        _syncCount = 0;
        _seqReadyCount = 0;
        ::generateTestToneSequence(&_ssm);
    }

    void requestGenerateSequence()
    {
        _requestGenerateSequence = true;
    }

    void requestResetAllSequence()
    {
        _requestResetAllSequence = true;
    }

    void requestResetGate()
    {
        _requestResetGate = true;
    }

    void resetAllSequence()
    {
        _syncCount = 0;
        ::resetSequence(&_ssm);
    }

    void resetGate()
    {
        _syncCount = 0;
        ::resetGate(&_ssm);
    }

    void generateSequence(bool resetSyncCount = true)
    {
        if (resetSyncCount)
            _syncCount = 0;
        _seqReadyCount = 0;
        ::generateSequence(&_ssm);
        _ssm.keyStep.setMode(Step::Mode::Forward);
        _ssm.gateStep.setMode(Step::Mode::Forward);
        // _ssm.keyStep.pos.setLimit(0, 16);
        // _ssm.gateStep.pos.setLimit(0, 16);
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
    SyncInTrigger _syncIn;
    CLOCK _clock;

    uint8_t _seqReadyCount;
    uint8_t _seqReadyCountMax;
    int8_t _syncCount;
    float _seqGateOffStep;
    LimitValue<int8_t> _settingPos;

    bool _requestGenerateSequence;
    bool _requestResetAllSequence;
    bool _requestResetGate;
    bool _auto_generative;

    uint8_t _ppq;
};
