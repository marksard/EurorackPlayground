/*!
 * Oscillator class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once
#include <Arduino.h>
#include "note.h"

#define WAVE_LENGTH 4096
#define WAVE_LENGTH_BIT 12

#ifdef USE_MCP4922
#include "sine_12bit_4096.h"
#define WAVE_INDEX_DIV_BIT 0
#define WAVE_HEIGHT 4096
#else
#include "sine_11bit_4096.h"
#define WAVE_INDEX_DIV_BIT 1
#define WAVE_HEIGHT 2048
#endif

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
        SINE,
        NOISE,
        MAX = NOISE,
    };
    const char waveName[Wave::MAX+1][9] = {"SQUARE  ", "DN-RAMP ", "UP-RAMP ", "SI-RAMP ", "PH-RAMP ", "TRIANGLE", "  SINE  ", "W-NOISE "};
    // 上位12ビット(0~4095)をindex範囲にする
    const uint32_t indexBit = OSC_WAVE_BIT - WAVE_LENGTH_BIT;

public:
    Oscillator()
    : _phaseShift(0, 99, 0, 99)
    , _folding(0, 99, 0, 99)
    {
    }

    void init(float clock)
    {
        _phaseAccum = 0;
        _phaseAccum2 = 0;
        _tuningWordM = 0;
        _tuningWordM2 = 0;
        _wave = Wave::SQU;
        _noteNameIndex = 0;
        _widthHalf = WAVE_LENGTH >> 1;
        _widthM1 = WAVE_LENGTH -1;
        _heightHalf = WAVE_HEIGHT >> 1;
        _heightM1 = WAVE_HEIGHT -1;
        _interruptClock = clock;
        // _halfReso = _reso >> 1;
    }

    // value範囲＝DAC、PWM出力範囲：0-4095(12bit)
    // index範囲：0-4095(12bit)
    // とした。sine以外は単純な演算のみで済む
    uint16_t getWaveValue()
    {
        _phaseAccum = _phaseAccum + _tuningWordM;
        uint32_t index = _phaseAccum >> indexBit;
        _phaseAccum2 = _phaseAccum2 + _tuningWordM2;
        uint32_t indexHeight = (index >> WAVE_INDEX_DIV_BIT);
        uint32_t indexPhase = _phaseAccum2 >> indexBit;
        uint16_t indexOffset = (index + 3030) % WAVE_LENGTH;
        uint16_t value = 0;
        switch (_wave)
        {
        case Wave::SQU:
            value = index < _widthHalf ? _heightM1 : 0;
            break;
        case Wave::DRAMP:
            value = _heightM1 - indexHeight;
            break;
        case Wave::URAMP:
            value = indexHeight;
            break;
        case Wave::TRI:
            // value = index < _halfReso ? index * 2 : (_resom1 - index) * 2;
            value = index < _widthHalf ? indexHeight * 2 : (_heightM1 - indexHeight) * 2;
            value = applyEzFolding(value);
            break;
        case Wave::SINE:
#ifdef USE_MCP4922
            value = sine_12bit[index];
#else
            value = sine_11bit[index];
#endif
            value = applyEzFolding(value);
            break;
        case Wave::NOISE:
            value = random(0, WAVE_HEIGHT);
            break;
        case Wave::SINE_RAMP:
#ifdef USE_MCP4922
            value = ((indexHeight + sine_12bit[indexOffset]) >> 1) % WAVE_HEIGHT;
#else
            value = ((indexHeight + sine_11bit[indexOffset]) >> 1) % WAVE_HEIGHT;
#endif
            break;
        case Wave::PH_RAMP:
            value = ((indexHeight * indexPhase) >> 11) % WAVE_HEIGHT;
            break;
        default:
            value = 0;
            break;
        }

        return value;
    }

    void setFrequency(float freqency)
    {
        // チューニングワード値 = 2^N(ここでは32bitに設定) * 出力したい周波数 / クロック周波数
        _tuningWordM = OSC_WAVE_BIT32 * ((float)freqency / _interruptClock);
        _tuningWordM2 = OSC_WAVE_BIT32 * ((float)(freqency+_phaseShift.get()) / _interruptClock);
    }

    void addPhaseShift(int8_t value)
    {
        _phaseShift.add(value);
    }

    bool setPhaseShift(int8_t value)
    {
        bool result = _phaseShift.get() != value;
        _phaseShift.set(value);
        return result;
    }

    int8_t getPhaseShift() {return _phaseShift.get();}

    void addFolding(int8_t value)
    {
        _folding.add(value);
    }

    bool setFolding(int8_t value)
    {
        bool result = _folding.get() != value;
        _folding.set(value);
        return result;
    }

    int8_t getFolding() {return (_folding.get());}

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
    uint16_t _widthHalf;
    uint16_t _widthM1;
    uint32_t _heightHalf;
    uint32_t _heightM1;
    uint8_t _noteNameIndex;
    float _interruptClock;
    uint16_t _halfReso;
    LimitValue<int8_t> _phaseShift;
    LimitValue<int8_t> _folding;

    uint16_t applyEzFolding(uint16_t value)
    {
        if (value > (_heightM1 - (_folding.get() * 15))) value = (_heightM1 - (_folding.get() * 15)) - (value - (_heightM1 - (_folding.get() * 15)));
        if (value < (_folding.get() * 15)) value = (_folding.get() * 15) + ((_folding.get() * 15) - value);
        value = map(value, (_folding.get() * 15), (_heightM1 - (_folding.get() * 15)), 0, _heightM1);
        return value;
    }
};
