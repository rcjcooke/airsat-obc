#ifndef AOCS_H
#define AOCS_H

#include "AOCSController.h"

class AOCS {
public:
    AOCS();
    bool initialize();
    
    // Core timed execution loop called by OBC main loop framework
    void runIteration();

private:
    AOCSController _controller;
};

#endif
