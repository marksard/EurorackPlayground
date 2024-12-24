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
const static char *UI_VER = "cvco_conf_000\0";
struct UserConfig
{
    char ver[15];
    int16_t voctTune;
    int16_t oscWave;
    int16_t oscPhaseShift;
    int16_t oscFolding;
    int8_t biasMode;
};

int startUserConfigAddress = 0;
int startSynthPatchAddress = sizeof(UserConfig);

///////////////////////////////////////////////////////////////////////////////
void initUserConfig(UserConfig *pUserConfig)
{
    strcpy(pUserConfig->ver, UI_VER);
    pUserConfig->voctTune = 118;
    pUserConfig->oscWave = 0;
    pUserConfig->oscPhaseShift = 5;
    pUserConfig->oscFolding = 0;
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
