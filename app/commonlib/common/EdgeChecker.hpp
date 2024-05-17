/*!
 * EdgeChecker class
 * Copyright 2024 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */ 

#pragma once

#include <Arduino.h>

class EdgeChecker
{
public:
    EdgeChecker() {}
    EdgeChecker(uint8_t pin)
    {
        init(pin);
    }
    
    /// @brief ピン設定
    /// @param pin
    void init(uint8_t pin)
    {
        _pin = pin;

        pinMode(pin, INPUT);

        // 空読み
        for(int i = 0; i < 8; ++i)
        {
            isEdgeHigh();
        }
    }

    /// @brief 立上がりエッジ検出
    /// @return 
    inline bool isEdgeHigh()
    {
        uint8_t value = readPin();
        uint8_t edge = value == 1 && _lastValue == 0 ? 1 : 0;
        _lastValue = value;
        return edge;
    }

    /// @brief 立下がりエッジ検出
    /// @return 
    inline bool isEdgeLow()
    {
        uint8_t value = readPin();
        uint8_t edge = value == 0 && _lastValue == 1 ? 1 : 0;
        _lastValue = value;
        return edge;
    }

protected:
    uint8_t _pin;
    uint8_t _lastValue;

    /// @brief ピン値読込
    /// @return
    virtual uint8_t readPin()
    {
        return gpio_get(_pin);
        // return digitalRead(_pin);
    }
};
