#pragma once

#include "Definitions.h"
#include <cstdio>

class Motor {
public:
    Motor(void* deviceHandle, unsigned short nodeId)
        : deviceHandle_(deviceHandle), nodeId_(nodeId) {}

    bool enable() {
        return VCS_SetEnableState(deviceHandle_, nodeId_, &errorCode_) != 0;
    }

    bool enableProfilePositionMode() {
        return VCS_ActivateProfilePositionMode(deviceHandle_, nodeId_, &errorCode_) != 0;
    }

    bool enableProfileVelocityMode() {
        return VCS_ActivateProfileVelocityMode(deviceHandle_, nodeId_, &errorCode_) != 0;
    }

    bool setVelocity(long velocity) {
        return VCS_MoveWithVelocity(deviceHandle_, nodeId_, velocity, &errorCode_) != 0;
    }

    bool setPosition(long position, bool absolute = true, bool immediately = true) {
        return VCS_MoveToPosition(
            deviceHandle_,
            nodeId_,
            position,
            absolute,
            immediately,
            &errorCode_
        ) != 0;
    }

    bool halt() {
        return VCS_HaltVelocityMovement(deviceHandle_, nodeId_, &errorCode_) != 0;
    }

    bool disable() {
        return VCS_SetDisableState(deviceHandle_, nodeId_, &errorCode_) != 0;
    }

    unsigned int lastError() const {
        return errorCode_;
    }

    unsigned short nodeId() const {
        return nodeId_;
    }

private:
    void* deviceHandle_;
    unsigned short nodeId_;
    unsigned int errorCode_ = 0;
};
