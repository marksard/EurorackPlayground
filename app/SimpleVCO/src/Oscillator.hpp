/*!
 * Oscillator class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once
#include <Arduino.h>
#include "sine_12bit_4096.h"
#include "tri_wavefold_12bit_4096.h"
#include "note.h"

#define OSC_RESO 4096
#define OSC_RESO_BIT 12
#define OSC_WAVE_BIT 32
#define OSC_WAVE_BIT32 4294967296 // 2^32

template <typename vs = int8_t>
class LimitValue
{
public:
    LimitValue(vs limitMin, vs limitMax)
    {
        _value = 0;
        _min = 0;
        _max = 0;
        _limitMax = limitMax;
        _limitMin = limitMin;
        setLimit(_min, _max);
    }

    LimitValue(vs limitMin, vs limitMax, vs min, vs max)
    {
        _value = 0;
        _limitMax = limitMax;
        _limitMin = limitMin;
        setLimit(min, max);
    }

    void set(vs value) { _value = constrain(value, _min, _max); }
    vs get() { return _value; }
    void add(vs value) { _value = constrain(_value + value, _min, _max); }

    void setLimit(vs min, vs max)
    {
        if (min == _min && max == _max) return;
        _min = MIN(MAX(min, _limitMin), _max);
        _max = MAX(MIN(max, _limitMax), _min);
        set(_value);
    }
    vs getMin() { return _min; }
    vs getMax() { return _max; }
    vs getDiff() { return _max - _min; }

private:
    vs _value;
    vs _min;
    vs _max;
    vs _limitMax;
    vs _limitMin;
};

class Oscillator
{
public:
    enum Wave
    {
        SQU,
        DRAMP,
        URAMP,
        SINE_RAMP,
        PH_RAMP,
        TRI,
        TRI_FOLD,
        SINE,
        NOISE,
        MAX = NOISE,
    };
    const char waveName[Wave::MAX+1][9] = {"SQUARE  ", "DN-RAMP ", "UP-RAMP ", "SI-RAMP ", "PH-RAMP ", "TRIANGLE", "TRI-FOLD", "  SINE  ", "W-NOISE "};
    // 上位12ビット(0~4095)をindex範囲にする
    const uint32_t indexBit = OSC_WAVE_BIT - OSC_RESO_BIT;

public:
    Oscillator()
    : _phaseShift(0, 99, 0, 99)
    {
    }

    void init(float timer_timing)
    {
        uint32_t reso = OSC_RESO;
        _phaseAccum = 0;
        _phaseAccum2 = 0;
        _tuningWordM = 0;
        _tuningWordM2 = 0;
        _wave = Wave::SQU;
        _noteNameIndex = 0;
        _pulseWidth = reso / 2;
        _reso = reso;
        _resom1 = reso -1;
        _intrruptClock = 1000000.0 / (float)timer_timing; // == 1sec / 10us == 1000000us / 10us == 100kHz
        _halfReso = _reso / 2;
    }

    // value範囲＝DAC、PWM出力範囲：0-4095(12bit)
    // index範囲：0-4095(12bit)
    // とした。sine以外は単純な演算のみで済む
    uint16_t getWaveValue()
    {
        _phaseAccum = _phaseAccum + _tuningWordM;
        uint32_t index = _phaseAccum >> indexBit;
        _phaseAccum2 = _phaseAccum2 + _tuningWordM2;
        uint32_t indexPhase = _phaseAccum2 >> indexBit;
        uint16_t indexOffset = (index + 3030) % OSC_RESO;
        uint16_t value = 0;
        switch (_wave)
        {
        case Wave::SQU:
            value = index < _pulseWidth ? _resom1 : 0;
            break;
        case Wave::DRAMP:
            value = _resom1 - index;
            break;
        case Wave::URAMP:
            value = index;
            break;
        case Wave::TRI:
            value = index < _halfReso ? index * 2 : (_resom1 - index) * 2;
            break;
        case Wave::TRI_FOLD:
            value = tri_wavefold_12bit_4096[index];
            break;
        case Wave::SINE:
            value = sine_12bit_4096[index];
            break;
        case Wave::NOISE:
            value = random(0, OSC_RESO);
            break;
        case Wave::SINE_RAMP:
            value = ((index + sine_12bit_4096[indexOffset]) >> 1) % OSC_RESO;
            break;
        case Wave::PH_RAMP:
            value = ((index * indexPhase) >> 11) % OSC_RESO;
            break;
        default:
            value = 0;
            break;
        }

        return value;
    }

    void setFrequency(uint16_t freqency)
    {
        // チューニングワード値 = 2^N(ここでは32bitに設定) * 出力したい周波数 / クロック周波数
        _tuningWordM = OSC_WAVE_BIT32 * freqency / _intrruptClock;
        _tuningWordM2 = OSC_WAVE_BIT32 * (freqency+_phaseShift.get()) / _intrruptClock;
    }

    void addPhaseShift(int8_t phaseShift)
    {
        _phaseShift.add(phaseShift);
    }

    int8_t getPhaseShift() {return _phaseShift.get();}

    bool setNoteNameFromFrequency(uint16_t freqency)
    {
        uint8_t noteNameIndex = 0;
        for (int i = 127; i >= 0; --i)
        {
            if (noteFreq[i] <= (float)freqency)
            {
                noteNameIndex = i + 1;
                break;
            }
        }
        bool result = _noteNameIndex != noteNameIndex;
        _noteNameIndex = noteNameIndex;
        return result;
    }

    bool setWave(Wave value)
    {
        bool result = _wave != value;
        _wave = value;
        return result;
    }

    Wave getWave() { return _wave; }
    const char *getWaveName() { return waveName[_wave]; }
    const char *getNoteName() { return noteName[_noteNameIndex]; }

private:
    uint32_t _phaseAccum;
    uint32_t _tuningWordM;
    uint32_t _phaseAccum2;
    uint32_t _tuningWordM2;
    Wave _wave;
    uint16_t _pulseWidth;
    uint8_t _noteNameIndex;
    uint32_t _reso;
    uint32_t _resom1;
    float _intrruptClock;
    uint16_t _halfReso;
    LimitValue<int8_t> _phaseShift;
};