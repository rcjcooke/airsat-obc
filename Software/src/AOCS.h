#ifndef AOCS_H
#define AOCS_H

#include "AOCSController.h"
#include "FusedAttitudeSensor.h"

class AOCS {
public:
    AOCS();
    bool initialize();
    
    // Core Timed Execution Path (10Hz)
    void runIteration();

    // Generalized Public Calibration Interface
    void calibrateSensors(uint32_t durationMs = 15000);

private:
    AOCSController _controller;
    FusedAttitudeSensor _attitudeSensor;
};

#endif
