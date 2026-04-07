#ifndef GLOBALS_H
#define GLOBALS_H

#include "Arduino.h"
#include "software_defines.h"

namespace Haptics {
    extern volatile unsigned long lastPacketMs;
    inline volatile unsigned long lastPacketMs = 0;

    // Volatile, non-static, user-denied variables
    struct Globals {
        uint8_t ledcMotorVals[MAX_LEDC_MOTORS];
        uint16_t pcaMotorVals[MAX_I2C_MOTORS];
        uint16_t allMotorVals[MAX_MOTORS];
        // Duration bump has been active
        int64_t bumpActivateTime[MAX_MOTORS];
        // whether bump has been triggered since value was last zero.
        bool bumpSinceZero[MAX_MOTORS];
        bool updatedMotors;
        bool reinitLEDC;
        bool processOscCommand; // moves the heavy commands out of ISR time
        bool processSerCommand;
        bool beenPinged;
        String commandToProcess;
        bool messageRecieved;
        unsigned long packetCount;
    };

    inline Globals initGlobals() {
        Globals g = {};
        g.reinitLEDC = false;
        g.updatedMotors = false;
        g.processOscCommand = false;
        g.processSerCommand = false;
        g.commandToProcess = "";
        g.beenPinged = false;
        g.messageRecieved = false;
        return g;
    }

    // Define a structure to store timing data
    struct TimingData {
        uint32_t digitalWriteCycles;
        uint32_t loopCycles;
    };

    inline TimingData profiler = {0, 0};

    // Declare a global instance of Globals.
    inline Globals globals = initGlobals();
} // namespace Haptics

#endif // GLOBALS_H