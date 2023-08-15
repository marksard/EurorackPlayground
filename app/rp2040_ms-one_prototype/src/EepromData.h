/*!
 * EEPROM Data
 * 設定まわりの処理のまとめ
 * Copyright 2023 marksard
 */ 

#pragma once

#include <Arduino.h>
#include <EEPROM.h>

void initEEPROM()
{
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
    EEPROM.begin(1024);
#else
    EEPROM.begin();
#endif
}

// 設定値系
const static char *UI_VER = "ms01conf_001\0";
struct UserConfig
{
    char ver[14];
    byte selectedSlot;
    byte inputOctVorMIDI; // 0:cv 1:usbmidi 2:randomseq
    byte seqMaxStep;
    byte seqBPM;
    byte seqScale;
    byte seqPpq;
    byte sendSync;
    byte setVOctCalibration;
    byte autoChange;
    byte autoChangeBar;
};

// 操作系パラメータ現在値
const static char *PARAM_VER = "ms01param001\0";
struct SynthPatch
{
    char ver[14];
    byte osc01_wave;
    byte osc01_oct;
    byte osc01_semi;
    byte osc01_octfull;
    byte osc01_detune;
    byte osc01_vol;

    byte osc02_wave;
    byte osc02_oct;
    byte osc02_semi;
    byte osc02_octfull;
    byte osc02_detune;
    byte osc02_vol;

    byte envFlt_attack;
    byte envFlt_decay;
    // byte envFlt_sustain;
    byte envFlt_release;
    byte envFlt_amount;

    byte envAmp_attack;
    byte envAmp_decay;
    // byte envAmp_sustain;
    byte envAmp_release;
    byte envAmp_amount;

    byte lfo01_freq;
    byte lfo01_amt_delay;
    byte lfo01_amt_osc02;
    byte lfo01_amt_ffreq;

    byte flt_Freq;
    byte flt_Reso;

    byte delay_feedback;
    byte delay_time;
    byte delay_level;

    byte driveLevel;
    byte hardClip;
    byte slideTime;
};

// EEPROM格納順番
int startUserConfigAddress = 0;
int startSynthPatchAddress = sizeof(UserConfig);

///////////////////////////////////////////////////////////////////////////////
void initUserConfig(UserConfig *pUserConfig)
{
    strcpy(pUserConfig->ver, UI_VER);
    pUserConfig->selectedSlot = -1;
    pUserConfig->inputOctVorMIDI = 2;
    pUserConfig->seqMaxStep = 8;
    pUserConfig->seqBPM = 155;
    pUserConfig->seqScale = 2;
    pUserConfig->seqPpq = 2;
    pUserConfig->sendSync = 0;
    pUserConfig->setVOctCalibration = 100;
    pUserConfig->autoChange = 0;
    pUserConfig->autoChangeBar = 8;
}

void loadUserConfig(UserConfig *pUserConfig)
{
    EEPROM.get<UserConfig>(startUserConfigAddress, *pUserConfig);
    if (strcmp(pUserConfig->ver, UI_VER))
    {
        initUserConfig(pUserConfig);
    }
}

void saveUserConfig(UserConfig *pUserConfig)
{
    EEPROM.put<UserConfig>(startUserConfigAddress, *pUserConfig);
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
    EEPROM.commit();
#endif
}

void saveSelectedSlot(byte selectedSlot)
{
    UserConfig userConfig;
    loadUserConfig(&userConfig);
    userConfig.selectedSlot = selectedSlot;
    saveUserConfig(&userConfig);
}

///////////////////////////////////////////////////////////////////////////////
void initSynthPatch(SynthPatch *pPatch)
{
    strcpy(pPatch->ver, PARAM_VER);
    pPatch->osc01_wave = 3;
    pPatch->osc01_oct = 1;
    pPatch->osc01_semi = 0;
    pPatch->osc01_octfull = 41;
    pPatch->osc01_detune = 3;
    pPatch->osc01_vol = 7;

    pPatch->osc02_wave = 1;
    pPatch->osc02_oct = 2;
    pPatch->osc02_semi = 0;
    pPatch->osc02_octfull = 46;
    pPatch->osc02_detune = 2;
    pPatch->osc02_vol = 7;

    pPatch->envFlt_attack = 0;
    pPatch->envFlt_decay = 111;
    pPatch->envFlt_release = 252;
    pPatch->envFlt_amount = 3;

    pPatch->envAmp_attack = 0;
    pPatch->envAmp_decay = 22;
    pPatch->envAmp_release = 88;
    pPatch->envAmp_amount = 8;

    pPatch->lfo01_freq = 1;
    pPatch->lfo01_amt_delay = 0;
    pPatch->lfo01_amt_osc02 = 1;
    pPatch->lfo01_amt_ffreq = 5;

    pPatch->flt_Freq = 56;
    pPatch->flt_Reso = 245;

    pPatch->delay_feedback = 224;
    pPatch->delay_time = 168;
    pPatch->delay_level = 6;

    pPatch->driveLevel = 0;
    pPatch->hardClip = 1;
    pPatch->slideTime = 0;
}

void loadSynthPatch(SynthPatch *pPatch, int selectSlot)
{
    if (selectSlot == -1)
    {
        initSynthPatch(pPatch);
        return;
    }

    EEPROM.get<SynthPatch>(startSynthPatchAddress +
                                   (sizeof(SynthPatch) * selectSlot),
                               *pPatch);
    if (strcmp(pPatch->ver, PARAM_VER))
    {
        initSynthPatch(pPatch);
    }

    // Serial.print("pPatch->osc01_wave = ");
    // Serial.print(pPatch->osc01_wave);
    // Serial.println(";");
    // Serial.print("pPatch->osc01_oct = ");
    // Serial.print(pPatch->osc01_oct);
    // Serial.println(";");
    // Serial.print("pPatch->osc01_semi = ");
    // Serial.print(pPatch->osc01_semi);
    // Serial.println(";");
    // Serial.print("pPatch->osc01_octfull = ");
    // Serial.print(pPatch->osc01_octfull);
    // Serial.println(";");
    // Serial.print("pPatch->osc01_detune = ");
    // Serial.print(pPatch->osc01_detune);
    // Serial.println(";");
    // Serial.print("pPatch->osc01_vol = ");
    // Serial.print(pPatch->osc01_vol);
    // Serial.println(";");

    // Serial.print("pPatch->osc02_wave = ");
    // Serial.print(pPatch->osc02_wave);
    // Serial.println(";");
    // Serial.print("pPatch->osc02_oct = ");
    // Serial.print(pPatch->osc02_oct);
    // Serial.println(";");
    // Serial.print("pPatch->osc02_semi = ");
    // Serial.print(pPatch->osc02_semi);
    // Serial.println(";");
    // Serial.print("pPatch->osc02_octfull = ");
    // Serial.print(pPatch->osc02_octfull);
    // Serial.println(";");
    // Serial.print("pPatch->osc02_detune = ");
    // Serial.print(pPatch->osc02_detune);
    // Serial.println(";");
    // Serial.print("pPatch->osc02_vol = ");
    // Serial.print(pPatch->osc02_vol);
    // Serial.println(";");

    // Serial.print("pPatch->envFlt_attack = ");
    // Serial.print(pPatch->envFlt_attack);
    // Serial.println(";");
    // Serial.print("pPatch->envFlt_decay = ");
    // Serial.print(pPatch->envFlt_decay);
    // Serial.println(";");
    // Serial.print("pPatch->envFlt_release = ");
    // Serial.print(pPatch->envFlt_release);
    // Serial.println(";");
    // Serial.print("pPatch->envFlt_amount = ");
    // Serial.print(pPatch->envFlt_amount);
    // Serial.println(";");

    // Serial.print("pPatch->envAmp_attack = ");
    // Serial.print(pPatch->envAmp_attack);
    // Serial.println(";");
    // Serial.print("pPatch->envAmp_decay = ");
    // Serial.print(pPatch->envAmp_decay);
    // Serial.println(";");
    // Serial.print("pPatch->envAmp_release = ");
    // Serial.print(pPatch->envAmp_release);
    // Serial.println(";");
    // Serial.print("pPatch->envAmp_amount = ");
    // Serial.print(pPatch->envAmp_amount);
    // Serial.println(";");

    // Serial.print("pPatch->lfo01_freq = ");
    // Serial.print(pPatch->lfo01_freq);
    // Serial.println(";");
    // Serial.print("pPatch->lfo01_amt_delay = ");
    // Serial.print(pPatch->lfo01_amt_delay);
    // Serial.println(";");
    // Serial.print("pPatch->lfo01_amt_osc02 = ");
    // Serial.print(pPatch->lfo01_amt_osc02);
    // Serial.println(";");
    // Serial.print("pPatch->lfo01_amt_ffreq = ");
    // Serial.print(pPatch->lfo01_amt_ffreq);
    // Serial.println(";");

    // Serial.print("pPatch->flt_Freq = ");
    // Serial.print(pPatch->flt_Freq);
    // Serial.println(";");
    // Serial.print("pPatch->flt_Reso = ");
    // Serial.print(pPatch->flt_Reso);
    // Serial.println(";");

    // Serial.print("pPatch->delay_feedback = ");
    // Serial.print(pPatch->delay_feedback);
    // Serial.println(";");
    // Serial.print("pPatch->delay_time = ");
    // Serial.print(pPatch->delay_time);
    // Serial.println(";");
    // Serial.print("pPatch->delay_level = ");
    // Serial.print(pPatch->delay_level);
    // Serial.println(";");

    // Serial.print("pPatch->driveLevel = ");
    // Serial.print(pPatch->driveLevel);
    // Serial.println(";");
    // Serial.print("pPatch->slideTime = ");
    // Serial.print(pPatch->slideTime);
    // Serial.println(";");
}

void saveSynthPatch(SynthPatch *pPatch, int selectSlot)
{
    if (selectSlot == -1)
    {
        return;
    }

    EEPROM.put<SynthPatch>(startSynthPatchAddress +
                                   (sizeof(SynthPatch) * selectSlot),
                               *pPatch);
#if defined(ARDUINO_ARCH_RP2040) && !defined(ARDUINO_ARCH_MBED)
    EEPROM.commit();
#endif
}
