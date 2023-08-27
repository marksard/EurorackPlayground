#pragma once

#include <Arduino.h>
#include <U8g2lib.h>
#include "GpioSet.h"

#ifdef PROTO
#define TITLE_ROW 3
#define POTS_ROW 0
#else
#define TITLE_ROW 0
#define POTS_ROW 1
#endif

static const char *_note[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
static const char *_shape[] = {"saw", "square", "triange", "sine"};
static const char *_mode[] = {"analog in", "midi in", "int. seq"};
static const char *_scale[] = {"ion:maj", "dorian", "phrygian", "lydian", "mixo", "aeo:n.min", "locrian"};

#ifdef PROTO
#define TITLE_ROW 3
#define POTS_ROW 0
#else
#define TITLE_ROW 0
#define POTS_ROW 1
#endif

class ParamGroup
{
public:
    ParamGroup()
    {
    }

    void init(U8G2 *pU8g2)
    {
        _pU8g2 = pU8g2;
        _offsetX = 0;
        _maxWidth = 63;
        _height = 16;
        _frameHeight = 13;
    }

    void setMaxWidth(byte maxWidth)
    {
        _maxWidth = maxWidth;
    }

    void setOffset(byte offsetX)
    {
        _offsetX = offsetX;
    }

    void attachNames(const char *names[])
    {
        _pTitle = names[0];
        _pValueName[0] = names[1];
        _pValueName[1] = names[2];
        _pValueName[2] = names[3];
        _pValueName[3] = names[4];
        _pValueName[4] = names[5];
    }

    void attachValues(byte *values[5][4])
    {
        for (byte i = 0; i < 5; ++i)
        {
            _pValueItems[i] = values[i][0];
            _MinItems[i] = *values[i][1];
            _MaxItems[i] = *values[i][2];
            _DispMode[i] = *values[i][3];
        }
    }

    void dispParamGroup(byte selected, 
        uint16_t analogValue0, uint16_t analogValue1, 
        byte btnAState, byte btnBState)
    {
        static char disp_buf[20] = {0};
        _pU8g2->setFont(u8g2_font_6x13_tf);

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
            byte height = _height * (i + POTS_ROW);
            if (selected)
            {
                _pU8g2->drawFrame(_offsetX, height, _maxWidth, _frameHeight);
            }

            _pU8g2->drawStr(_offsetX + 2, height, disp_buf);

            byte value = (byte)constrain(map(valueItem, _MinItems[i], _MaxItems[i], 1, _maxWidth - 1), 1, _maxWidth - 1);

            // value fill
            if (value > 1)
                _pU8g2->drawBox(_offsetX + 1, height + 1, value, _frameHeight - 2);

            // pots indicator
            if (selected)
            {
                if (i < 2)
                {
                    uint16_t current = i == 0 ? analogValue0 : analogValue1;
                    value = (byte)map(current, 0, POTS_MAX_VALUE, 1, _maxWidth - 2);
                    byte x = _offsetX + value;
                    byte y = height + _frameHeight;
                    _pU8g2->drawTriangle(x, y - 1, x + 4, y + 3, x - 4, y + 3);
                    _pU8g2->drawVLine(x, height + 1, _frameHeight - 2);
                }
            }
        }

        dispTitle(selected, btnAState, btnBState);
    }

    void dispTitle(byte selected, 
        byte btnAState, byte btnBState)
    {
        static char disp_buf[20] = {0};
        // Setting title and buttons
        if (selected)
        {
            _pU8g2->setFont(u8g2_font_5x8_tf);
            strcpy(disp_buf, _pValueName[3]);
            _pU8g2->drawStr(63 + 1, _height * TITLE_ROW + 1, disp_buf);
            strcpy(disp_buf, _pValueName[4]);
            _pU8g2->drawStr(96 + 1, _height * TITLE_ROW + 1, disp_buf);

            _pU8g2->drawFrame(63, _height * TITLE_ROW, 32, 12);
            _pU8g2->drawFrame(96, _height * TITLE_ROW, 32, 12);

            if ((byte)(*_pValueItems[3]) || btnAState)
                _pU8g2->drawBox(63, _height * TITLE_ROW, 32, 12);
            if ((byte)(*_pValueItems[4]) || btnBState)
                _pU8g2->drawBox(96, _height * TITLE_ROW, 32, 12);

            _pU8g2->setFont(u8g2_font_8x13B_tf);
            strcpy(disp_buf, _pTitle);
            _pU8g2->drawStr(0, _height * TITLE_ROW, disp_buf);
        }
    }

private:
    U8G2 *_pU8g2;
    byte _offsetX;
    byte _maxWidth;
    byte _height;
    byte _frameHeight;
    byte *_pValueItems[5];
    byte _MinItems[5];
    byte _MaxItems[5];
    byte _DispMode[5];
    const char *_pTitle;
    const char *_pValueName[5];
};
