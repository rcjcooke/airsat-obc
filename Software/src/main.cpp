#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <string>
#include "AOCS.h"

std::atomic<bool> runApplication(true);

void signalHandler(int signalNumber) {
    if (signalNumber == SIGINT || signalNumber == SIGTERM) {
        std::cout << "\n[OBC CORE] Termination signal received. Cleaning up subsystems..." << std::endl;
        runApplication = false;
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Starting Raspberry Pi OBC Core Software Application..." << std::endl;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // 1. Command-line configuration parsing state
    bool executeCalibration = true;

    for (int i = 1; i < argc; ++i) {
        std::string argument = argv[i];
        if (argument == "--skip-cal") {
            executeCalibration = false;
        }
    }

    AOCS aocsSubsystem;

    // 2. Hand down the runtime argument condition to the initialization engine
    if (!aocsSubsystem.init(executeCalibration)) {
        std::cerr << "[CRITICAL ERROR] Fatal interface failure initialising AOCS interface." << std::endl;
        return -1;
    }

    std::cout << "OBC AOCS Subsystem Handshake Successful. Entering flight loop." << std::endl;

    // Random target attitude set for now, this will be updated by the guidance system in future iterations
    aocsSubsystem.setTargetAttitude(1.5f);

    const auto loopPeriod = std::chrono::milliseconds(100);

    while (runApplication) {
        auto startTime = std::chrono::steady_clock::now();

        aocsSubsystem.runIteration();

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

        if (elapsedTime < loopPeriod) {
            std::this_thread::sleep_for(loopPeriod - elapsedTime);
        }
    }

    std::cout << "[OBC CORE] Main loop stopped. Application terminated safely." << std::endl;
    return 0;
}
