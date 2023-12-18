/*!
 * Envelope ADSR
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

#define EXP_CURVE(count, time, ratio) (exp(-1 * count * (ratio / time)))

class EnvelopeADSR
{
public:
    EnvelopeADSR()
    {
    }

    void init(double maxLevel)
    {
        _attack = 0;
        _decay = 0;
        _sustain = 0;
        _release = 0;
        _attack_curve_ratio = 3.0;
        _decay_curve_ratio = 3.0;
        _release_curve_ratio = 4.0;
        _stageCount = 0;
        _stage = 0;
        _lastGate = 0;
        _lastMsec = 0;
        _level = 0;
        _lastLevel = 0;
        _maxLevel = maxLevel;
    }

    /// @brief 
    /// @param attack reso:maxLevel unit: millisec
    /// @param decay reso:maxLevel unit: millisec
    /// @param sustain reso:maxLevel unit: output level(_maxLevel)
    /// @param release reso:maxLevel unit: millisec
    void set(uint16_t attack, uint16_t decay, uint16_t sustain, uint16_t release)
    {
        _attack = attack;
        _decay = decay;
        _sustain = min(sustain, _maxLevel);
        _release = release;
    }

    void setAttack(uint16_t value) { _attack = value; }
    void setDecay(uint16_t value) { _decay = value; }
    void setSustain(uint16_t value) { _sustain = min(value, _maxLevel); }
    void setRelease(uint16_t value) { _release = value; }
    
    void setAttackCurveRatio(double value) { _attack_curve_ratio = value; }
    void setDecayCurveRatio(double value) { _decay_curve_ratio = value; }
    void setReleaseCurveRatio(double value) { _release_curve_ratio = value; }

    void next(uint8_t gate)
    {
        ulong msec = millis();

        if (_lastGate == 0 && gate == 1)
        {
            _lastGate = gate;
            _stage = 0;
            _stageCount = 0;
            _lastMsec = msec;
            _level = 0;
            // Serial.println("gate on");
        }
        else if (_lastGate == 1 && gate == 0)
        {
            _lastGate = gate;
            _stage = 3;
            _stageCount = 0;
            _lastMsec = msec;
            // Serial.println("gate off");
        }
        int16_t stepTime = msec - _lastMsec;

        switch (_stage)
        {
        case 0:
            // Serial.print("attack ");
            _stageCount += stepTime;
            // _level = min((double)_stageCount * (_maxLevel / _attack), _maxLevel);
            if (_decay < 2)
            {
                _level = _sustain - (EXP_CURVE(_stageCount, _attack, _attack_curve_ratio) * _sustain);
            }
            else
            {
                _level = _maxLevel - (EXP_CURVE(_stageCount, _attack, _attack_curve_ratio) * _maxLevel);
            }
            _lastLevel = _level;
            if (_attack < _stageCount)
            {
                _stage = 1;
                _stageCount = 0;
            }
            break;
        case 1:
            // Serial.print("decay");
            _stageCount += stepTime;

            // linear curve
            // _level = max(_maxLevel - (_stageCount * (_maxLevel / _decay)), _sustain);
            // exponential curve
            _level = (EXP_CURVE(_stageCount, _decay, _decay_curve_ratio) * (_maxLevel - _sustain)) + _sustain;
            
            _lastLevel = _level;
            if (_decay < _stageCount)
            {
                _stage = 2;
                _stageCount = 0;
            }
            break;
        case 2:
            _level = _sustain;
            // Serial.print("sustain");
            break;
        case 3:
            // Serial.print("release");
            _stageCount += stepTime;
            // linear curve
            // _level = max(_lastLevel - _stageCount * (_maxLevel / _release), 0);
            // exponential curve
            _level = (EXP_CURVE(_stageCount, _release, _release_curve_ratio) * (_lastLevel));
            if (_release < _stageCount)
            {
                _stage = 4;
                _stageCount = 0;
                _level = 0;
            }
            break;
        default:
            return;
            break;
        }
        // Serial.print(" ");
        // Serial.print(_sustain);
        // Serial.print(",");
        // Serial.print(_stageCount);
        // Serial.print(",");
        // Serial.print(_level);
        // Serial.println();
        _lastMsec = msec;
    }

    uint16_t getLevel() { return (uint16_t)_level; }
    uint16_t getAttack() { return _attack; }
    uint16_t getDecay() { return _decay; }
    uint16_t getSustain() { return _sustain; }
    uint16_t getRelease() { return _release; }

private:
    uint16_t _attack;
    uint16_t _decay;
    uint16_t _sustain;
    uint16_t _release;
    double _attack_curve_ratio;
    double _decay_curve_ratio;
    double _release_curve_ratio;
    uint16_t _stageCount;
    uint8_t _stage;
    uint8_t _lastGate;
    ulong _lastMsec;
    double _level;
    double _lastLevel;
    double _maxLevel;
};
