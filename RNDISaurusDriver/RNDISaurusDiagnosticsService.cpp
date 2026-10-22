//
//  RNDISaurusDiagnosticsService.cpp
//  RNDISaurus — USB tethering driver for RNDIS devices on macOS
//
//  Copyright (C) 2026 N4H4
//  SPDX-License-Identifier: MPL-2.0
//
//  This Source Code Form is subject to the terms of the Mozilla Public License,
//  v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
//  one at https://mozilla.org/MPL/2.0/.
//
//  NewUserClient follows Apple's DriverKitUserClientSample (Copyright © 2025 Apple
//  Inc., MIT; notice in NOTICE.md).
//

// The diagnostics child service: created by the driver, it exists to be opened by the
// host app and to create RNDISaurusDiagnosticsUserClient instances that read the
// driver's state. See RNDISaurusDiagnosticsService.iig for why the driver itself
// cannot be opened.

#include <DriverKit/IOLib.h>
#include <DriverKit/IODispatchQueue.h>

#include "RNDISaurusDiagnosticsService.h"
#include "RNDISaurusDiagnosticsUserClient.h"
#include "RNDISaurusDriverPrivate.hpp"

struct RNDISaurusDiagnosticsService_IVars {
    RNDISaurusDriver * driver;      // retained until free
    IODispatchQueue *  driverQueue; // the driver's default queue, retained until free
};

// MARK: - Lifecycle

bool
RNDISaurusDiagnosticsService::init()
{
    if (!super::init()) {
        return false;
    }
    ivars = IONewZero(RNDISaurusDiagnosticsService_IVars, 1);
    return ivars != nullptr;
}

void
RNDISaurusDiagnosticsService::free()
{
    if (ivars != nullptr) {
        OSSafeReleaseNULL(ivars->driverQueue);
        OSSafeReleaseNULL(ivars->driver);
    }
    IOSafeDeleteNULL(ivars, RNDISaurusDiagnosticsService_IVars, 1);
    super::free();
}

kern_return_t
IMPL(RNDISaurusDiagnosticsService, Start)
{
    kern_return_t ret = Start(provider, SUPERDISPATCH);
    RNDLOG("diagnostics service %{public}p: Start (provider %{public}p): 0x%08x", this, provider, ret);
    return ret;
}

kern_return_t
IMPL(RNDISaurusDiagnosticsService, Stop)
{
    kern_return_t ret = Stop(provider, SUPERDISPATCH);
    RNDLOG("diagnostics service %{public}p: Stop: 0x%08x", this, ret);
    return ret;
}

// MARK: - Driver access

// Called once, before the service is registered, with non-null values.
void
RNDISaurusDiagnosticsService::SetDriver(RNDISaurusDriver * driver, IODispatchQueue * driverQueue)
{
    driver->retain();
    driverQueue->retain();
    ivars->driver      = driver;
    ivars->driverQueue = driverQueue;
}

kern_return_t
RNDISaurusDiagnosticsService::CopyDriver(RNDISaurusDriver ** driver, IODispatchQueue ** driverQueue)
{
    *driver      = nullptr;
    *driverQueue = nullptr;
    if (ivars->driver == nullptr || ivars->driverQueue == nullptr) {
        return kIOReturnNotReady;
    }
    ivars->driver->retain();
    ivars->driverQueue->retain();
    *driver      = ivars->driver;
    *driverQueue = ivars->driverQueue;
    return kIOReturnSuccess;
}

// MARK: - User client

// Creates the user client from this service's UserClientProperties dictionary (IOClass
// IOUserUserClient, IOUserClass RNDISaurusDiagnosticsUserClient). Create returns the
// object retained; that reference is handed to the caller.
kern_return_t
IMPL(RNDISaurusDiagnosticsService, NewUserClient)
{
    if (type != kRNDISaurusDiagnosticsUserClientType) {
        RNDLOG("diagnostics service: NewUserClient with unsupported type %u", type);
        return kIOReturnBadArgument;
    }
    IOService * client = nullptr;
    kern_return_t ret = Create(this, "UserClientProperties", &client);
    if (ret != kIOReturnSuccess || client == nullptr) {
        RNDLOG("diagnostics service: Create(UserClientProperties) failed: 0x%08x", ret);
        return ret != kIOReturnSuccess ? ret : kIOReturnNoMemory;
    }
    *userClient = OSDynamicCast(RNDISaurusDiagnosticsUserClient, client);
    if (*userClient == nullptr) {
        RNDLOG("diagnostics service: the created object is not a RNDISaurusDiagnosticsUserClient");
        client->release();
        return kIOReturnError;
    }
    RNDLOG("diagnostics service: user client %{public}p created", *userClient);
    return kIOReturnSuccess;
}
