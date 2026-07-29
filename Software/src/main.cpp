#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include "AOCS.h"

// Atomic flag to ensure safe multi-threaded access during signal handling
std::atomic<bool> runApplication(true);

// POSIX Signal Handler Function Callback
void signalHandler(int signalNumber) {
    if (signalNumber == SIGINT || signalNumber == SIGTERM) {
        std::cout << "\n[OBC CORE] Termination signal received. Cleaning up subsystems..." << std::endl;
        runApplication = false; // Gracefully breaks the main while loop
    }
}

int main() {
    std::cout << "Starting Raspberry Pi OBC Core Software Application..." << std::endl;

    // Register POSIX system signals for clean exit traps
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    AOCS aocsSubsystem;

    if (!aocsSubsystem.initialize()) {
        std::cerr << "[CRITICAL ERROR] Fatal interface failure initializing AOCS interface." << std::endl;
        return -1;
    }

    std::cout << "OBC AOCS Subsystem Handshake Successful. Entering main execution loop." << std::endl;

    // Fixed 10Hz Flight Loop Timing Structure (100 millisecond step sizes)
    const auto loopPeriod = std::chrono::milliseconds(100);

    while (runApplication) {
        auto startTime = std::chrono::steady_clock::now();

        // Run core automation iteration sequence
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
