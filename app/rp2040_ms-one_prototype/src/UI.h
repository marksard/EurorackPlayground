#pragma once

#include <MozziGuts.h>
// #include "../../commonlib/common/Tiny4kOLEDi2c.hpp"
#include <U8g2lib.h>
#include "../../commonlib/common/SmoothAnalogRead.hpp"
#include "../../commonlib/common/RotaryEncoder.hpp"
#include "../../commonlib/common/Button.hpp"
#include "../../commonlib/soundlogic/OscillatorTables12.h"
#include "GpioSet.h"
#include "EEPROMData.h"

static U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/U8X8_PIN_NONE);

static const char *_note[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
static const char *_shape[] = {"saw", "square", "triange", "sine"};
static const char *_mode[] = {"analog in", "midi in", "int. seq"};
static const char *_scale[] = {"ion:maj", "dorian", "phrygian", "lydian", "mixo", "aeo:n.min", "locrian"};

class ParamSet
{
public:
    ParamSet()
    {
    }

    void setOffset(byte offsetX)
    {
        _offsetX = offsetX;
    }

    void setNames(const char *names[])
    {
        _pTitle = names[0];
        _pValueName[0] = names[1];
        _pValueName[1] = names[2];
        _pValueName[2] = names[3];
        _pValueName[3] = names[4];
        _pValueName[4] = names[5];
    }

    void setValues(byte *values[5][4])
    {
        for (byte i = 0; i < 5; ++i)
        {
            _pValueItems[i] = values[i][0];
            _MinItems[i] = *values[i][1];
            _MaxItems[i] = *values[i][2];
            _DispMode[i] = *values[i][3];
        }
    }

    void dispNames(U8G2 *pOled, byte selected, 
        uint16_t analogValue0, uint16_t analogValue1, 
        byte btnAState, byte btnBState)
    {
        static char disp_buf[20] = {0};
        pOled->setFont(u8g2_font_6x13_tf);

        // pots and encoder
        for (byte i = 0; i < 3; ++i)
        {
            // setting label and values
            byte valueItem = (byte)(*_pValueItems[i]);
            switch (_DispMode[i])
            {
            case 0:
                strcpy(disp_buf, _pValueName[i]);
                break;
            case 1:
                sprintf(disp_buf, "%s:%03d", _pValueName[i], valueItem);
                break;
            case 2:
                sprintf(disp_buf, "%s:%s%1d", _pValueName[i], _note[valueItem % 12], valueItem / 12);
                break;
            case 3:
                sprintf(disp_buf, "%s", _shape[valueItem]);
                break;
            case 4:
                sprintf(disp_buf, "%s", _mode[valueItem]);
                break;
            case 5:
                sprintf(disp_buf, "%s", _scale[valueItem]);
                break;
            
            default:
                strcpy(disp_buf, _pValueName[i]);
                break;
            }

            // frames
            byte height = _height * i;
            if (selected)
            {
                pOled->drawFrame(_offsetX, height, _maxWidth, _frameHeight);
            }

            pOled->drawStr(_offsetX + 2, height, disp_buf);

            byte value = (byte)constrain(map(valueItem, _MinItems[i], _MaxItems[i], 1, _maxWidth - 1), 1, _maxWidth - 1);

            // value fill
            if (value > 1)
                pOled->drawBox(_offsetX + 1, height + 1, value, _frameHeight - 2);

            // pots indicator
            if (selected)
            {
                if (i < 2)
                {
                    uint16_t current = i == 0 ? analogValue0 : analogValue1;
                    value = (byte)map(current, 0, 4096, 1, _maxWidth - 2);
                    byte x = _offsetX + value;
                    byte y = height + _frameHeight;
                    pOled->drawTriangle(x, y - 1, x + 4, y + 3, x - 4, y + 3);
                    pOled->drawVLine(x, height + 1, _frameHeight - 2);
                }
            }
        }

        // Setting title and buttons
        if (selected)
        {
            pOled->setFont(u8g2_font_5x8_tf);
            strcpy(disp_buf, _pValueName[3]);
            pOled->drawStr(63 + 1, _height * 3 + 1, disp_buf);
            strcpy(disp_buf, _pValueName[4]);
            pOled->drawStr(96 + 1, _height * 3 + 1, disp_buf);

            pOled->drawFrame(63, _height * 3, 32, 12);
            pOled->drawFrame(96, _height * 3, 32, 12);

            if ((byte)(*_pValueItems[3]) || btnAState)
                pOled->drawBox(63, _height * 3, 32, 12);
            if ((byte)(*_pValueItems[4]) || btnBState)
                pOled->drawBox(96, _height * 3, 32, 12);

            pOled->setFont(u8g2_font_8x13B_tf);
            strcpy(disp_buf, _pTitle);
            pOled->drawStr(0, _height * 3, disp_buf);
        }
    }

private:
    byte _offsetX = 0;
    byte _maxWidth = 63;
    byte _height = 16;
    byte _frameHeight = 13;
    byte *_pValueItems[5];
    byte _MinItems[5];
    byte _MaxItems[5];
    byte _DispMode[5];
    const char *_pTitle;
    const char *_pValueName[5];
};





extern UserConfig conf;
extern SynthPatch patch;
extern byte seqStart;
extern byte seqChange;
extern byte envAmpFree;
extern byte testtone;

static RotaryEncoder encA;
static RotaryEncoder encB;
static SmoothAnalogRead pots[2];
static Button buttons[2];
static byte isOLEDInit = 0;
static byte userParamSave = 0;
static byte userParamLoad = 0;
static byte userConfigSave = 0;
static byte btnDisp[2] = {0, 0};

static byte nullItem = 0;
static byte max_osc = OSC_MAX - 1;
static byte max_oct = 119;
static byte max_semi = MAX_SEMI - 1;
static byte max_8bit = 255;
static byte max_scale = 6;
static byte max_amnt = 8;
static byte max_tune = 10;
static byte max_save = 8;
static byte min_zero = 0;
static byte min_one = 1;
static byte max_two = 2;
static byte max_6bit = 32;
static byte max_7bit = 64;
static byte mode0 = 0;
static byte mode1 = 1;
static byte mode2 = 2;
static byte mode3 = 3;
static byte mode4 = 4;
static byte mode5 = 5;
#define MENUMAX 12

static const char *names[MENUMAX][6] = 
{
    {"Osc 1  ", "wave", "tune", "oct ", "", ""},
    {"Osc 2  ", "wave", "tune", "oct ", "", ""},
    {"Osc Out", "osc1 vol", "osc2 vol", "drive", " hard", ""},
    {"Filter ", "freq", "reso", "amount", "", ""},
    {"Flt Env", "attack", "decay", "release", "", ""},
    {"Amp Env", "attack", "decay", "release", "", ""},
    {"LFO    ", "rate", "->freq", "->osc", "", ""},
    {"Delay  ", "feedback", "time", "level", "", ""},
    {"Option ", "", "env free", "plymode", "", " save"},
    {"Patch  ", "", "", "slot", " load", " save"},
    {"Bar Chg", "ppq", "testtone", "bar", "enable", " sync"},
    {"Gen Seq", "scale", "step", "BPM", "change", " play"},
};

static byte *values[MENUMAX][5][4] =
{
    {
        {&patch.osc01_wave, &min_zero, &max_osc, &mode3},
        {&patch.osc01_detune, &min_zero, &max_tune, &mode1},
        {&patch.osc01_octfull, &min_zero, &max_oct, &mode2}, 
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
    },
    {
        {&patch.osc02_wave, &min_zero, &max_osc, &mode3},
        {&patch.osc02_detune, &min_zero, &max_tune, &mode1},
        {&patch.osc02_octfull, &min_zero, &max_oct, &mode2}, 
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
    },
    {
        {&patch.osc01_vol, &min_zero, &max_amnt, &mode0},
        {&patch.osc02_vol, &min_zero, &max_amnt, &mode0},
        {&patch.driveLevel, &min_zero, &max_amnt, &mode0}, 
        {&patch.hardClip, &min_zero, &min_one, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
    },
    {
        {&patch.flt_Freq, &min_zero, &max_8bit, &mode0},
        {&patch.flt_Reso, &min_zero, &max_8bit, &mode0},
        {&patch.envFlt_amount, &min_zero, &max_amnt, &mode0}, 
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
    },
    {
        {&patch.envFlt_attack, &min_zero, &max_8bit, &mode0},
        {&patch.envFlt_decay, &min_zero, &max_8bit, &mode0},
        {&patch.envFlt_release, &min_zero, &max_8bit, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
    },
    {
        {&patch.envAmp_attack, &min_zero, &max_8bit, &mode0},
        {&patch.envAmp_decay, &min_zero, &max_8bit, &mode0},
        {&patch.envAmp_release, &min_zero, &max_8bit, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
    },
    {
        {&patch.lfo01_freq, &min_one, &max_8bit, &mode0},
        {&patch.lfo01_amt_ffreq, &min_zero, &max_amnt, &mode0},
        {&patch.lfo01_amt_osc02, &min_zero, &max_amnt, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
    },
    {
        {&patch.delay_feedback, &min_zero, &max_8bit, &mode0},
        {&patch.delay_time, &min_zero, &max_8bit, &mode0},
        {&patch.delay_level, &min_zero, &max_amnt, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
    },
    {
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&envAmpFree, &min_zero, &min_one, &mode0},
        {&conf.inputOctVorMIDI, &min_zero, &max_two, &mode4},
        {&userConfigSave, &min_zero, &min_one, &mode0},
        {&userConfigSave, &min_zero, &min_one, &mode0},
    },
    {
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&nullItem, &min_zero, &min_zero, &mode0},
        {&conf.selectedSlot, &min_zero, &max_save, &mode1},
        {&userParamLoad, &min_zero, &min_one, &mode0},
        {&userParamSave, &min_zero, &min_one, &mode0},
    },
    {
        {&conf.seqPpq, &min_zero, &mode4, &mode1},
        {&testtone, &min_zero, &min_one, &mode0},
        {&conf.autoChangeBar, &min_one, &max_7bit, &mode1},
        {&conf.autoChange, &min_zero, &min_one, &mode0},
        {&conf.sendSync, &min_zero, &min_one, &mode0},
    },
    {
        {&conf.seqScale, &min_zero, &max_scale, &mode5},
        {&conf.seqMaxStep, &min_zero, &max_6bit, &mode1},
        {&conf.seqBPM, &min_one, &max_8bit, &mode1},
        {&seqChange, &min_zero, &min_one, &mode0},
        {&seqStart, &min_zero, &min_one, &mode0},
    },
};

static ParamSet ps[MENUMAX];

void initDispItems()
{
    for (byte i = 0; i < MENUMAX; ++i)
    {
        ps[i].setOffset(i % 2 == 1 ? 64 : 0);
        ps[i].setNames(names[i]);
        ps[i].setValues(values[i]);
    }
}

void initOLED()
{
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.setContrast(40);
    u8g2.setFontPosTop();
    u8g2.setDrawColor(2);
    u8g2.begin();
    initDispItems();
    isOLEDInit = 1;
}

void dispOLED(int menuIndex)
{
    byte page = (menuIndex >> 1) * 2;
    u8g2.clearBuffer();
    bool selected = menuIndex % 2;
    // Serial.print(menuIndex);
    // Serial.print(",");
    // Serial.print(page);
    // Serial.print(",");
    // Serial.print(selected);
    // Serial.print(",");
    // Serial.println(menuIndex);
    ps[page].dispNames(&u8g2, !selected, pots[0].getValue(), pots[1].getValue(), btnDisp[0], btnDisp[1]);
    if (MENUMAX > page + 1){
        ps[page + 1].dispNames(&u8g2, selected, pots[0].getValue(), pots[1].getValue(), btnDisp[0], btnDisp[1]);
    }
    u8g2.sendBuffer();
}


void initController()
{
    // reconnectDigitalIn(A1);
    // reconnectDigitalIn(A0);
    encA.init(ENC0A, ENC0B);
    encB.init(ENC1A, ENC1B);
    pots[0].init(POT0);
    pots[1].init(POT1);
    buttons[0].init(SW0);
    buttons[1].init(SW1);
    buttons[0].setHoldTime(200);
    buttons[1].setHoldTime(200);
}

///////////////////////////////////////////////////////////////////////////////
/// @brief 範囲制限サイクリック版
/// @tparam su
/// @param value 検査値
/// @param min 制限範囲の最小値
/// @param max 制限範囲の最大値
/// @return 更新値
template <typename su = uint8_t>
su constrainCyclic(su value, su min, su max)
{
    if (value > max)
        return min;
    if (value < min)
        return max;
    return value;
}

static int menuIndex = 0;
static byte reqUpdateDisp = 0;
static byte reqResetDisp = 1;

/// @brief 設定情報の更新
byte updateUserIF()
{
    if (!isOLEDInit)
        return 0;
    
    static byte lastmenuIndex = menuIndex;
    static byte unlock[2] = {0, 0};
    menuIndex = constrainCyclic(menuIndex + encB.getDirection(true), 0, MENUMAX - 1);
    if (menuIndex != lastmenuIndex)
    {
        reqUpdateDisp = 1;
        for (byte i = 0; i < 2; ++i)
        {
            unlock[i] = 0;
        }
    }
    else
    {
        // pots
        for (byte i = 0; i < 2; ++i)
        {
            uint16_t readValue = pots[i].analogRead();
            byte value = (*(byte *)(values[menuIndex][i][0]));
            byte min = (*(byte *)values[menuIndex][i][1]);
            byte max = (*(byte *)values[menuIndex][i][2]);
            byte newValue = constrain(map(readValue, 0, 4095, min, max), min, max);
            if (newValue == value)
            {
                unlock[i] = 1;
            }

            if (unlock[i])
            {
                (*(byte *)(values[menuIndex][i][0])) = newValue;       
            }

            if (pots[i].hasChanged())
            {
                reqUpdateDisp = 1;
            }
        }

        // encoder
        byte value = (*(byte *)(values[menuIndex][2][0]));
        byte min = (*(byte *)values[menuIndex][2][1]);
        byte max = (*(byte *)values[menuIndex][2][2]);
        int8_t encoder = encA.getDirection();
        int8_t step = max <= 32 ? constrain(encoder, -1, 1) : encoder;
        (*(byte *)(values[menuIndex][2][0])) = constrain((int)value + step, min, max);
        if (encoder != 0)
        {
            reqUpdateDisp = 1;
        }

        // buttons
        for (byte i = 0; i < 2; ++i)
        {
            byte state = buttons[i].getState();
            byte settingIndex = i + 3;
            if (state == 2)
            {
                min = (*(byte *)values[menuIndex][settingIndex][1]);
                max = (*(byte *)values[menuIndex][settingIndex][2]);
                if (min != max)
                {
                    // toggle
                    (*(byte *)(values[menuIndex][settingIndex][0])) = 
                        (*(byte *)(values[menuIndex][settingIndex][0])) ? 0 : 1;
                    reqUpdateDisp = 1;
                    btnDisp[i] = 1;
                }
            }
            
        }
    }

    lastmenuIndex = menuIndex;

    if (userParamLoad == 1)
    {
        userParamLoad = 0;
        loadSynthPatch(&patch, conf.selectedSlot);
        saveSelectedSlot(conf.selectedSlot);
        reqResetDisp = 1;
    }
    else if (userParamSave == 1)
    {
        userParamSave = 0;
        saveSynthPatch(&patch, conf.selectedSlot);
        saveSelectedSlot(conf.selectedSlot);
        reqResetDisp = 1;
    }
    else if (userConfigSave == 1)
    {
        userConfigSave = 0;
        saveUserConfig(&conf);
        reqResetDisp = 1;
    }

    return reqResetDisp | reqUpdateDisp;
}

void updateOLED()
{
    static byte oledPowLast = 0;

    if (reqResetDisp)
    {
        reqResetDisp = 0;
        menuIndex = 0;
        dispOLED(0);
    }
    else if (reqUpdateDisp == 1)
    {
        reqUpdateDisp = 0;
        dispOLED(menuIndex);
        if (btnDisp[0] == 1 || btnDisp[1] == 1)
        {
            btnDisp[0] = 0;
            btnDisp[1] = 0;
            reqUpdateDisp = 1;
        }
    }
}
