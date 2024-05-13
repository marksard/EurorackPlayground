/*!
 * EEPROM Data
 * 設定まわりの処理のまとめ
 * Copyright 2024 marksard
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
const static char *UI_VER = "svco_conf_000\0";
struct UserConfig
{
    char ver[15];
    int16_t voctTune;
    int16_t oscAWave;
    int16_t oscAPhaseShift;
    int16_t oscAFolding;
    int16_t oscBWave;
    int16_t oscBPhaseShift;
    int16_t oscBFolding;
    int8_t cvAssigned;
    int8_t rangeMode;
    int8_t biasMode;
};

int startUserConfigAddress = 0;
int startSynthPatchAddress = sizeof(UserConfig);

///////////////////////////////////////////////////////////////////////////////
void initUserConfig(UserConfig *pUserConfig)
{
    strcpy(pUserConfig->ver, UI_VER);
    pUserConfig->voctTune = 118;
    pUserConfig->oscAWave = 0;
    pUserConfig->oscAPhaseShift = 5;
    pUserConfig->oscAFolding = 0;
    pUserConfig->oscBWave = 0;
    pUserConfig->oscBPhaseShift = 5;
    pUserConfig->oscBFolding = 0;
    pUserConfig->cvAssigned = 0;
    pUserConfig->rangeMode = 0;
    pUserConfig->biasMode = 0;
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
