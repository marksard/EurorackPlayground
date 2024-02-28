/*!
 * Oscillator class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once
#include <Arduino.h>
#include "sine_12bit_4096.h"
#include "note.h"

#define OSC_RESO 4096
#define OSC_RESO_BIT 12
#define OSC_WAVE_BIT 32
#define OSC_WAVE_BIT32 4294967296 // 2^32

class Oscillator
{
public:
    enum Wave
    {
        SQU,
        USAW,
        DSAW,
        TRI,
        SINE,
        NOISE,
        MAX = NOISE,
    };
    const char waveName[6][7] = {"SQUARE", "UP-SAW", "DN-SAW", "TRIANG", " SINE ", "NOISE "};
    // 上位12ビット(0~4095)をindex範囲にする
    const uint32_t indexBit = OSC_WAVE_BIT - OSC_RESO_BIT;

public:
    Oscillator()
    {
    }

    void init(uint16_t timer_timing)
    {
        uint32_t reso = OSC_RESO;
        _phaseAccum = 0;
        _tuningWordM = 0;
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
        uint16_t value = 0;
        switch (_wave)
        {
        case Wave::SQU:
            value = index < _pulseWidth ? _resom1 : 0;
            break;
        case Wave::USAW:
            value = index;
            break;
        case Wave::DSAW:
            value = _resom1 - index;
            break;
        case Wave::TRI:
            value = index < _halfReso ? index * 2 : (_resom1 - index) * 2;
            break;
        case Wave::SINE:
            value = sine_12bit_4096[index];
            break;
        case Wave::NOISE:
            value = random(0, OSC_RESO);
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
    }

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
    Wave _wave;
    uint16_t _pulseWidth;
    uint8_t _noteNameIndex;
    uint32_t _reso;
    uint32_t _resom1;
    float _intrruptClock;
    uint16_t _halfReso;
};