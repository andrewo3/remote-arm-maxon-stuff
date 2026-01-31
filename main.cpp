#include "Motor.h"
#include <vector>
#include <unistd.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>
#include <cstdio>
#include <cstring>

int main(int argc, char** argv) {
    char* pDeviceName = "EPOS4";
    char* pProtocolStackName = "MAXON SERIAL V2";
    char* pInterfaceName = "USB";
    char* pPortName = "USB0";

    unsigned int error_code = 0;
    const unsigned int nodes = 2;

    void* deviceHandle = VCS_OpenDevice(
        pDeviceName,
        pProtocolStackName,
        pInterfaceName,
        pPortName,
        &error_code
    );

    printf("Device handle: %p\n", deviceHandle);

    // Create motors
    std::vector<Motor> motors;
    for (unsigned int i = 0; i < nodes; ++i) {
        motors.emplace_back(deviceHandle, i + 1);
    }

    // Enable + velocity mode
    for (auto& motor : motors) {
        motor.enable();
        motor.enableProfilePositionMode();
    }

    const char* SHM_NAME = "/motor_velocities";

    // Open shared memory
    int shmFd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shmFd == -1) {
        perror("shm_open");
        return -1;
    }

    // Map shared memory
    size_t shmSize = sizeof(int) * nodes;
    int* sharedVelocities = static_cast<int*>(
        mmap(nullptr, shmSize, PROT_READ, MAP_SHARED, shmFd, 0)
    );

    if (sharedVelocities == MAP_FAILED) {
        perror("mmap");
        close(shmFd);
        return -1;
    }

    printf("Shared memory mapped, entering control loop\n");

    // ---- CONTINUOUS LOOP ----
    while (true) {
        for (unsigned int i = 0; i < nodes; ++i) {
            printf("Motor %u position: %u\n", i + 1, sharedVelocities[i]);
            motors[i].setPosition(sharedVelocities[i],true);
        }

        usleep(16666); // ~60 Hz
    }


    // Stop + disable
    for (auto& motor : motors) {
        motor.halt();
        motor.disable();
    }

    printf("Closing device...\n");
    if (VCS_CloseAllDevices(&error_code) == 0) {
        printf("Failed to close device\n");
        return -1;
    }

    printf("Terminating\n");
    return 0;
}
