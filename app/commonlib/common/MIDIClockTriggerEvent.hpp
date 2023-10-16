/*!
 * MIDIClockTriggerEvent class
 * Copyright 2023 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
#include <MIDI.h>
#include "TriggerInterface.hpp"

class MIDIClockTriggerEvent : public TriggerInterface
{
public:
    MIDIClockTriggerEvent(MIDI_NAMESPACE::SerialMIDI<HardwareSerial> midi)
        : midiTransport(midi), midiInterface((MidiTransport &)midiTransport)
    {
    }

    void start() override
    {
        if(midiInterface.getInputChannel() == 0)
            midiInterface.begin(1);
    }

    void stop() override {}

    bool ready() override
    {
        if(!midiInterface.read())
        {
            return false;
        }

        if (midiInterface.getType() == midi::Start)
        {
            _isStart = true;
            return false;
        }

        if (midiInterface.getType() == midi::Stop)
        {
            _isStart = false;
            return false;
        }

        if (midiInterface.getType() == midi::Clock)
        {
            if (_isStart)
                return true;
            else return false;
        }

        return false;
    }

    void setMills(int millSec) override {}
    void setBPM(byte bpm, byte bpmReso) override {}

    bool isStart()
    {
        return _isStart;
    }

protected:
    using MidiTransport = MIDI_NAMESPACE::SerialMIDI<HardwareSerial>;
    using MidiInterface = MIDI_NAMESPACE::MidiInterface<MidiTransport>;

    MidiTransport midiTransport;
    MidiInterface midiInterface;

    bool _isStart = false;
};
