#pragma once
#include <Arduino.h>

#define DEF_MAX_STEP 16
#define DEF_MAX_STEP_M1 (DEF_MAX_STEP - 1)
#define MAX_SCALE_KEY 7
#define MAX_SCALES 7

// スケール
static const uint8_t scales[MAX_SCALES][MAX_SCALE_KEY] =
    {
        {0, 2, 4, 5, 7, 9, 11}, // ionian / major
        {0, 2, 3, 5, 7, 9, 10}, // dorian
        {0, 1, 3, 5, 7, 8, 10}, // phrygian
        {0, 2, 4, 6, 7, 9, 11}, // lydian
        {0, 2, 4, 5, 7, 9, 10}, // mixolydian
        {0, 2, 3, 5, 7, 8, 10}, // aeolian / natural minor
        {0, 1, 3, 5, 6, 8, 10}, // locrian
};

#define MAX_GATE_TIMINGS 4
#define MAX_GATE_STEP 16
// メロディーを成立しやすくするための発音タイミングマップ
// この上にランダムでタイミングを追加してランダムかつメロディーを成立しやすく
const uint8_t gateMap[MAX_GATE_TIMINGS][MAX_GATE_STEP] = {
    {1, 0, 1, 1,
     1, 1, 0, 1,
     0, 1, 1, 0,
     1, 0, 0, 1,},
    {0, 1, 0, 1,
     1, 0, 1, 0,
     0, 1, 1, 0,
     0, 1, 0, 1,},
    {1, 1, 0, 1,
     1, 1, 1, 1,
     1, 1, 1, 1,
     1, 0, 1, 1,},
    {1, 0, 1, 0,
     1, 0, 0, 1,
     0, 1, 1, 0,
     1, 0, 0, 1}};


template <typename vs = uint8_t>
void initArray(vs *pArray, vs size)
{
    for (vs i = 0; i < size; ++i)
        pArray[i] = 0;
}

template <typename vs = uint8_t>
void printArray(vs *pArray, vs size)
{
    for (vs i = 0; i < size; ++i)
    {
        Serial.print(pArray[i]);
        Serial.print(",");
    }

    Serial.println("");
}

template <typename vs = int8_t>
class LimitValue
{
public:
    LimitValue(vs limitMax)
    {
        _value = 0;
        _min = 0;
        _max = 0;
        _limitMax = limitMax;
        setLimit(_min, _max);
    }

    LimitValue(vs limitMax, vs min, vs max)
    {
        _value = 0;
        _limitMax = limitMax;
        setLimit(min, max);
    }

    void set(vs value) { _value = constrain(value, _min, _max); }
    vs get() { return _value; }
    void add(vs value) { _value = constrain(_value + value, _min, _max); }

    void setLimit(vs min, vs max)
    {
        _min = MAX(min, 0);
        _max = MIN(max, _limitMax);
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
};

class Step
{
public:
    static const uint8_t MAX_STEP = DEF_MAX_STEP;
    static const uint8_t MAX_STEP_M1 = DEF_MAX_STEP_M1;
    enum Mode
    {
        Forward,
        Reverse,
        TurnBack,
        Max,
    };

public:
    Step()
        : pos(MAX_STEP_M1, 0, MAX_STEP_M1)
    {
    }

    void NextPlayStep()
    {
        if (_mode == Mode::Forward)
        {
            if (_playCount + 1 > pos.getMax())
                _playCount = pos.getMin();
            else
                _playCount++;
            pos.set(_playCount);
        }
        else if (_mode == Mode::Reverse)
        {
            if (_playCount - 1 < 0)
                _playCount = pos.getMax();
            else
                _playCount--;
            pos.set(_playCount);
        }
        else if (_mode == Mode::TurnBack)
        {
            if (_playCount + 1 > pos.getDiff() + 1)
                pos.add(-1);
            else
                pos.add(1);

            if (_playCount > pos.getDiff() * 2)
                _playCount = 0;
            else
                _playCount++;
        }
    }

    void ResetPlayStep()
    {
        if (_mode == Mode::Forward)
        {
            _playCount = 0;
            pos.set(_playCount);
        }
        else if (_mode == Mode::Reverse)
        {
            _playCount = pos.getMax();
            pos.set(_playCount);
        }
        else if (_mode == Mode::TurnBack)
        {
            _playCount = 0;
            pos.set(_playCount);
        }
    }

    void SetMode(Mode mode) { _mode = mode; }
    uint8_t GetMode() { return _mode; }
    uint8_t GetPlayCount() { return _playCount; }

public:
    LimitValue<int8_t> pos;

private:
    Mode _mode;
    int16_t _playCount;
};

class StepSeqModel
{
public:
    static const uint8_t MAX_STEP = DEF_MAX_STEP;
    static const uint8_t MAX_STEP_M1 = DEF_MAX_STEP_M1;
    enum Gate
    {
        _, // None
        S, // Short
        H, // Half
        L, // Long
        T, // Tie
        Max,
    };

public:
    StepSeqModel() : _scaleIndex(MAX_SCALES, 0, MAX_SCALES)
    {
        initArray(_keys, MAX_STEP);
        initArray(_octaves, MAX_STEP);
        initArray((uint8_t *)_gates, MAX_STEP);
        _scaleIndex.set(2);
    }

    void setKey(uint8_t step, uint8_t value) { _keys[constrain(step, 0, MAX_STEP)] = value; }
    void setOctave(uint8_t step, uint8_t value) { _octaves[constrain(step, 0, MAX_STEP)] = value; }
    void setGate(uint8_t step, StepSeqModel::Gate value) { _gates[constrain(step, 0, MAX_STEP)] = value; }
    uint8_t getKey(uint8_t value) { return _keys[value]; }
    uint8_t getOctave(uint8_t value) { return _octaves[value]; }
    uint8_t getGate(uint8_t value) { return _gates[value]; }

    uint8_t getPlayKey() { return _keys[keyStep.pos.get()]; }
    uint8_t getPlayOctave() { return _octaves[keyStep.pos.get()]; }
    uint8_t getPlayGate() { return _gates[gateStep.pos.get()]; }

    uint8_t getPlayNote() { return (getPlayOctave() * 12) + scales[_scaleIndex.get()][getPlayKey()]; }

    void printSeq()
    {
        printArray(_keys, MAX_STEP);
        printArray(_octaves, MAX_STEP);
        printArray((uint8_t *)_gates, MAX_STEP);
        for (byte i = 0; i < StepSeqModel::MAX_STEP * 3; ++i)
        {
            Serial.print("i:");
            Serial.print(i);
            Serial.print(", ");
            Serial.print("Key Step:");
            Serial.print(keyStep.pos.get());
            Serial.print(", ");
            Serial.print("Gate Step:");
            Serial.print(gateStep.pos.get());
            Serial.print(", ");
            Serial.print("Key:");
            Serial.print(getPlayKey());
            Serial.print(", ");
            Serial.print("Octave:");
            Serial.print(getPlayOctave());
            Serial.print(", ");
            Serial.print("Gate:");
            Serial.print(getPlayGate());
            Serial.print(", ");
            Serial.print("Note:");
            Serial.print(getPlayNote());
            Serial.print(", ");
            Serial.println();
            keyStep.NextPlayStep();
            gateStep.NextPlayStep();
        }
    }

public:
    Step keyStep;
    Step gateStep;

public:
    uint8_t _keys[MAX_STEP];
    uint8_t _octaves[MAX_STEP];
    StepSeqModel::Gate _gates[MAX_STEP];
    LimitValue<uint8_t> _scaleIndex;
};

void generateSequence(StepSeqModel *pssm)
{
    Serial.println("generateSequence\n");
    randomSeed(micros());
    byte geteSelect = random(MAX_GATE_TIMINGS);

    for (byte i = 0; i < StepSeqModel::MAX_STEP; ++i)
    {
        // タイミングマップにランダムでタイミングをorして足す
        StepSeqModel::Gate gate = gateMap[geteSelect][i] == 1 ? (StepSeqModel::Gate)random(StepSeqModel::Gate::S, StepSeqModel::Gate::Max) : StepSeqModel::Gate::_;
        pssm->setGate(i, gate);

        // 変更前のメロディーラインをランダムに残して繋がりを持たせる
        if (random(2))
        {
            continue;
        }

        // 基音(C0) + 音階はスケールに従いつつランダムで + オクターブ上下移動をランダムで(-1 or 0 ~ 2 * 12)
        // 0 ~ 24 + スケール音
        pssm->setOctave(i, 1 + (random(-1, 3)));
        pssm->setKey(i, random(MAX_SCALE_KEY));
    }
}

void generateTestToneSequence(StepSeqModel *pssm)
{
    Serial.println("generateTestToneSequence\n");
    byte geteSelect = random(MAX_GATE_TIMINGS);

    for (byte i = 0; i < StepSeqModel::MAX_STEP; ++i)
    {
        if (i < 11)
        {
            pssm->setGate(i, i&1?StepSeqModel::Gate::_ : StepSeqModel::Gate::T);
        }
        else{
            pssm->setGate(i, i&1?StepSeqModel::Gate::S : StepSeqModel::Gate::H);
        }
        pssm->setOctave(i, (i>>1)%6);
        pssm->setKey(i, 0);
     }
}
