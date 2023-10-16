/*!
 * Marksard Synthesizer One
 * MCU: Waveshare RP2040 Zero
 * Mozzi library 1.1.0 or later
 * Tiny4kOLED 2.2.2 or later
 * MIDIUSB 1.0.5 or later
 * Copyright 2023 marksard
 */

///////////////////////////////////////////////////////////////////////////////
// mozzi 1.1.0 or later

#define CONTROL_RATE 512
#include <MozziGuts.h>
#include <mozzi_midi.h>
#include <Oscil.h>
#include <tables/sin256_int8.h>
#include <ADSR.h>
#include <AudioDelayFeedback.h>
#include "../../commonlib/soundlogic/Overdrive.hpp"
#include "../../commonlib/common/RecieveMidi.hpp"
#include "../../commonlib/common/RecieveGateOct.hpp"
#include "../../commonlib/common/SyncInTrigger.hpp"
#include "../../commonlib/common/PollingTimeEvent.hpp"
#include "../../commonlib/common/SequenceGenerator.hpp"
#include "../../commonlib/common/MIDIClockTriggerEvent.hpp"
#include "ResonantFilterEx.hpp"
#include "Oscillator.hpp"
#include "EEPROMData.h"
#include "GpioSet.h"
#include "UI.h"

#define AMOUNT(value, amount, max) ((amount) == 0 ? 0 : ((value) >> ((max) - (amount))))

#define DELAY_FEEDBACK_MEM 16384

static Oscillator osc;
static Oscil<SIN256_NUM_CELLS, AUDIO_RATE> lfo01(SIN256_DATA);
static Oscil<SIN256_NUM_CELLS, AUDIO_RATE> lfo02(SIN256_DATA);
static ADSR<CONTROL_RATE, CONTROL_RATE> envFlt;
static ADSR<CONTROL_RATE, CONTROL_RATE> envAmp;
static ResonantFilterEx<LOWPASS, uint16_t> lpf01;
static AudioDelayFeedback<DELAY_FEEDBACK_MEM, ALLPASS> audioDelay;
static RecieveGateOct rgo;
static Overdrive overDrive;
// static Overdrive limitter;
static SendRecvMIDI rm(Serial2, 1);
// static SyncInTrigger sit(GATE_PIN);
static MIDIClockTriggerEvent midiTrigger(Serial2);
static PollingTimeEvent pollingTrigger;
static SequenceAutoChanger seqGen;
static byte envFltStep;
static byte envAmpStep;
static int8_t lfo01Step = 0;
static int8_t lfo02Step = 0;

static int16_t sequencerVoltage = 0;
static float voltPerTone = 4096.0 / 12.0 / 5.0;

UserConfig conf;
SynthPatch patch;

byte seqStart = 0;
byte seqChange = 1;
byte envAmpFree = 0;
byte testtone = 0;

///////////////////////////////////////////////////////////////////////////////

/// @brief ディケイタイムに応じたサスティンの時間変更
/// @param decay 
/// @param x2 掛け算したい数
/// @return 
uint16_t getSustainTime(int decay, byte x2 = 1)
{
    uint16_t sustain = decay == 255 ? 65535 : decay >= 196 ? 400 * x2 : decay >= 128 ? 200 * x2 : 0;
    return sustain;
}

/// @brief エンベロープの設定(Sustain省略版)
/// @param pEnv
/// @param attack attack time/level
/// @param decay decaya time/level
/// @param release release time/level
void envSetADR(ADSR<CONTROL_RATE, CONTROL_RATE> *pEnv, int attack, int decay, int release)
{
    // Attack Time, Decay Time, sustainain Level, Release Timeの設定
    // setADLevels(250, 250)の設定に敵わないので固定
    // pEnv->setLevels(decay, min(decay << 1, 255), decay, max(min(decay << 1, release), 1));
    // pEnv->setTimes(max(attack << 2, 1), decay, decay << 8 + decay, max(release << 1, 1));
    pEnv->setADLevels(250, 250);
    uint16_t sustain = getSustainTime(decay);
    pEnv->setTimes(attack, decay, sustain, release);
}

/// @brief GATE/CV情報受付
void recieveGateCV()
{
    if (rgo.ready())
    {
        if (rgo.isNoteOn())
        {
            envFlt.noteOn();
            envAmp.noteOn();
        }
        else if (rgo.isNoteOff())
        {
            envFlt.noteOff();
            envAmp.noteOff();
        }
    }

    int vOct = rgo.getVOct();
    osc.setFreqFromVOctCalc12bit(Oscillator::Select::OSC01, vOct, patch.osc01_octfull, patch.osc01_detune);
    int add = patch.osc02_detune + AMOUNT((int)lfo01Step, patch.lfo01_amt_osc02, 8);
    osc.setFreqFromVOctCalc12bit(Oscillator::Select::OSC02, vOct, patch.osc02_octfull, add);
}

// #define RX_LED 17
// #define TX_LED 30
/// @brief MIDI情報受付
void recieveMIDI()
{
    if (rm.ready())
    {
        if (rm.isNoteOn())
        {
            if (conf.cvOutMode) digitalWrite(GATE_PIN, HIGH);
            envFlt.noteOn();
            envAmp.noteOn();
        }
        if (rm.isNoteOff())
        {
            if (conf.cvOutMode) digitalWrite(GATE_PIN, LOW);
            envFlt.noteOff();
            envAmp.noteOff();
        }
    }

    byte lastNote = rm.getNote();
    if (conf.cvOutMode) {
        sequencerVoltage = lastNote * voltPerTone;
    }
    osc.setFreq_Q16n16(Oscillator::Select::OSC01, lastNote, patch.osc01_octfull, patch.osc01_detune);
    int add = patch.osc02_detune + AMOUNT((int)lfo01Step, patch.lfo01_amt_osc02, 8);
    osc.setFreq_Q16n16(Oscillator::Select::OSC02, lastNote, patch.osc02_octfull, add);
}

/// @brief ランダムシーケンサ
void randomSequencer()
{
    static byte ppqCount = 24;
    static byte lastNote = 0;

    if (seqChange)
    {
        seqGen.generate();
        seqChange = 0;
        ppqCount = 24;
    }

    if (!seqStart && !seqGen.isStart())
    {
        ppqCount = 24;
        digitalWrite(GATE_PIN, LOW);
        seqGen.resetSeq();
        envFlt.noteOff();
        envAmp.noteOff();
        rm.allSoundOff();
        return;
    }

    if (seqGen.ready())
    {
        // rm.sendClock();
        if (conf.sendSync)
        {
            ppqCount++;
            if (ppqCount / (24 / conf.seqPpq))
            {
                if (conf.cvOutMode) {
                    if (seqGen.isNoteOn())
                    {
                        digitalWrite(GATE_PIN, LOW);
                        digitalWrite(VOCT_PIN, LOW);
                    }
                }
                else
                {
                    digitalWrite(GATE_PIN, HIGH);
                }
                ppqCount = 0;
            }
            else
            {
                if (!conf.cvOutMode) {
                    digitalWrite(GATE_PIN, LOW);
                }
            }
        }
    }

    if (seqGen.seqReady())
    {

        lastNote = seqGen.getNote();
        if (seqGen.isNoteOn())
        {
            if (conf.cvOutMode) 
            {
                digitalWrite(GATE_PIN, HIGH);
                digitalWrite(VOCT_PIN, seqGen.getAcc() ? HIGH : LOW);
            }
            envFlt.noteOn();
            envAmp.noteOn();
            rm.noteOff();
            rm.noteOn(lastNote, seqGen.getAcc() ? 112 : 96, patch.osc01_octfull);
        }
        else if (seqGen.isNoteOff())
        {
            if (conf.cvOutMode)
            {
                digitalWrite(GATE_PIN, LOW);
                digitalWrite(VOCT_PIN, LOW);
            }
            envFlt.noteOff();
            envAmp.noteOff();
            envAmp.setSustainTime(getSustainTime(patch.envAmp_decay));
            envFlt.setSustainTime(getSustainTime(patch.envFlt_decay));
            rm.noteOff();
        }
        // ノートオン・オフの間はスライドのためサスティンを伸ばす
        else
        {
            envAmp.setSustainTime(getSustainTime(patch.envAmp_decay, 2));
            envFlt.setSustainTime(getSustainTime(patch.envFlt_decay, 2));
        }
        seqGen.next();
    }

    if (conf.cvOutMode) {
        sequencerVoltage = lastNote * voltPerTone;
    }

    osc.setFreq_Q16n16(Oscillator::Select::OSC01, lastNote, patch.osc01_octfull, patch.osc01_detune);
    int add = patch.osc02_detune + AMOUNT((int)lfo01Step, patch.lfo01_amt_osc02, 8);
    osc.setFreq_Q16n16(Oscillator::Select::OSC02, lastNote, patch.osc02_octfull, add);
}

void initSynth()
{
    initEEPROM();

    loadUserConfig(&conf);
    loadSynthPatch(&patch, conf.selectedSlot);

    startMozzi(CONTROL_RATE);

    analogReadResolution(POTS_BIT);
    rgo.init(GATE_PIN, VOCT_PIN);
    pinMode(VOCT_PIN, conf.cvOutMode == 0 ? INPUT : OUTPUT);

    // seqGen.setTrigger(&midiTrigger);
    seqGen.setTrigger(&pollingTrigger);

    initController();

    rm.setup();
}

void updateSynth()
{
    static byte lastInputOctVorMIDI = 255;

    // ユーザー操作系更新
    byte changed = updateUserIF();

    if (lastInputOctVorMIDI != conf.inputOctVorMIDI)
    {
        lastInputOctVorMIDI = conf.inputOctVorMIDI;
        pinMode(GATE_PIN, lastInputOctVorMIDI == 0 ? INPUT : OUTPUT);
    }

    // 入力端子、MIDI、シーケンサーからの入力受付、エンベロープとオシレータ周波数の更新
    if (conf.inputOctVorMIDI == 0)
        recieveGateCV();
    else if (conf.inputOctVorMIDI == 1)
        recieveMIDI();
    else
    {
        if (changed)
        {
            pinMode(VOCT_PIN, conf.cvOutMode == 0 ? INPUT : OUTPUT);
            if (conf.syncMidi)
            {
                seqGen.stop();
                seqGen.setTrigger(&midiTrigger);
            }
            else
            {
                seqGen.stop();
                seqGen.setTrigger(&pollingTrigger);
            }
            seqGen.setTestMode(testtone);
            seqGen.setBPM(conf.seqBPM, 24);
            seqGen.setEndStep(conf.seqMaxStep);
            seqGen.setChangeBar(conf.autoChangeBar);
            seqGen.autoChanger(conf.autoChange);
            seqGen.setScale(conf.seqScale);
            if (seqStart)
                seqGen.start();
            else 
                seqGen.stop();
        }
        randomSequencer();
    }

    // その他の操作による設定の更新
    if (changed)
    {
        osc.setVOctCalibration(conf.setVOctCalibration);
        osc.setSlideTime(patch.slideTime);
        osc.setWave(Oscillator::Select::OSC01, (Oscillator::Wave)patch.osc01_wave);
        osc.setWave(Oscillator::Select::OSC02, (Oscillator::Wave)patch.osc02_wave);
        osc.setLevel(Oscillator::Select::OSC01, patch.osc01_vol);
        osc.setLevel(Oscillator::Select::OSC02, patch.osc02_vol);

        // パラメータ更新
        envSetADR(&envFlt, patch.envFlt_attack, patch.envFlt_decay, patch.envFlt_release);
        envSetADR(&envAmp, patch.envAmp_attack, patch.envAmp_decay, patch.envAmp_release);
        lfo01.setFreq((float)(patch.lfo01_freq * 0.05));
        lfo02.setFreq((float)(patch.lfo01_freq * 0.12));

        audioDelay.setFeedbackLevel(patch.delay_feedback - 128);

        overDrive.setParam(patch.driveLevel, patch.driveLevel+2);
    }

    // コーラス情報更新
    uint16_t time_amt = constrain(patch.delay_time + AMOUNT(lfo02Step, patch.lfo01_amt_delay, 8), 0, 255);
    time_amt = (time_amt << 8) + time_amt; // 8bit持ち上げるだけじゃなく、足してあげないと良い感じにならんので…
    audioDelay.setDelayTimeCells(time_amt);

    // エンベロープ情報更新
    envFlt.update();
    envAmp.update();
    envFltStep = AMOUNT(envFlt.next(), patch.envFlt_amount, 8);
    envAmpStep = AMOUNT(envAmp.next(), patch.envAmp_amount, 8);

    // フィルター情報更新
    int freq = min(patch.flt_Freq + envFltStep, 255);
    freq = constrain(freq + AMOUNT((int)lfo01Step, patch.lfo01_amt_ffreq, 8), 0, 255);
    lpf01.setCutoffFreqAndResonance(freq * 255, patch.flt_Reso * 255);
}

AudioOutput_t updateAudio()
{
    // LFO更新
    lfo01Step = lfo01.next();
    lfo02Step = lfo02.next();
    // サウンド更新
    AudioOutput_t sound = osc.next();
    // ローパスフィルター(int8_t)
    sound = lpf01.nextSafe(sound);

    // アンプエンベロープをかける（int8_t -> int16_tに拡大）
    if (!envAmpFree)
        sound = sound * envAmpStep;
    else
        sound = sound << 8;

    // オーバードライブ
    sound = overDrive.next(sound);
    // 5-8はちょっと増幅しとく
    sound = sound << (patch.driveLevel > 2 ? patch.driveLevel >> 1 : 0);

    // ディレイ(入力はint8_tなので8bitシフトして、任意の大きさに戻す)
    if (patch.delay_level == 8)
    {
        // エフェクト音のみ
        sound = audioDelay.next(sound >> 8) << 6;
    }
    else if (patch.delay_level > 0)
    {
        sound = (sound + (audioDelay.next(sound >> 8) << constrain(patch.delay_level -1 , 0, 6)));
    }

    // リミッター
    // sound = limitter.next(sound);

    // if (sound >> 14 > 0) Serial.print("14");
    // else if (sound >> 13 > 0) Serial.print("13");
    // else if (sound >> 12 > 0) Serial.print("12");
    // else if (sound >> 11 > 0) Serial.print("11");
    // else if (sound >> 10 > 0) Serial.print("10");

    // オーバードライブで小さくなるので補正
    byte bit = patch.driveLevel >= 7 ? 11 : patch.driveLevel >= 5 ? 12 : patch.driveLevel >= 2 ? 13 : 14;
    if (patch.hardClip)
        bit -= 1;

    if (conf.cvOutMode) {
        AudioOutput_t readValue = (sequencerVoltage >> 1) - AUDIO_BIAS;
        return readValue;
    }
    else
    {
        // RP2040だとpwm出力11bit幅(0-2048)に調整
        return MonoOutput::fromAlmostNBit(bit, sound).clip();
    }
}

void synthHook()
{
    audioHook(); // required here
}
