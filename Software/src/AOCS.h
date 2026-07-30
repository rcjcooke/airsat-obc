#ifndef AOCS_H
#define AOCS_H

#include "AOCSController.h"
#include "FusedAttitudeSensor.h" // Include the new fusion structure

class AOCS {
public:
    AOCS();
    bool initialize();
    void runIteration();

private:
    AOCSController _controller;
    FusedAttitudeSensor _attitudeSensor; // Instantiated inside the supervisor scope
};

#endif
