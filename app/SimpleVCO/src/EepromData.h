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
const static char *UI_VER = "svco_conf_001\0";
struct UserConfig
{
    char ver[15];
    int16_t voctTune;
    int16_t oscA_wave;
    int16_t oscA_phaseShift;
    int16_t oscB_wave;
    int16_t oscB_phaseShift;
};

int startUserConfigAddress = 0;
int startSynthPatchAddress = sizeof(UserConfig);

///////////////////////////////////////////////////////////////////////////////
void initUserConfig(UserConfig *pUserConfig)
{
    strcpy(pUserConfig->ver, UI_VER);
    pUserConfig->voctTune = 118;
    pUserConfig->oscA_wave = 0;
    pUserConfig->oscA_phaseShift = 5;
    pUserConfig->oscB_wave = 0;
    pUserConfig->oscB_phaseShift = 5;
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
