//
//  RNDISaurusDiagnosticsUserClient.cpp
//  RNDISaurus — USB tethering driver for RNDIS devices on macOS
//
//  Copyright (C) 2026 N4H4
//  SPDX-License-Identifier: MPL-2.0
//
//  This Source Code Form is subject to the terms of the Mozilla Public License,
//  v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
//  one at https://mozilla.org/MPL/2.0/.
//
//  The external-method dispatch follows Apple's DriverKitUserClientSample (Copyright
//  © 2025 Apple Inc., MIT; notice in NOTICE.md).
//

// The diagnostics user client: one external method that returns an RNDISaurusSnapshot.
// Its provider is the RNDISaurusDiagnosticsService child that created it; the driver
// and the driver's queue come from there.
//
// IOUserClient::ExternalMethod is declared to run on the queue named
// kIOUserClientQueueNameExternalMethod. Start binds that name to the driver's default
// queue, so the snapshot is taken between the driver's own callbacks and no lock is
// needed. Should the binding ever not take effect, the method notices (OnQueue) and
// hops onto the driver's queue synchronously instead.
//
// Stop runs on this object's own default queue while the snapshot method runs on the
// driver's, so Stop releases nothing: the driver and its queue stay retained until free.

#include <string.h>

#include <DriverKit/IOLib.h>
#include <DriverKit/IODispatchQueue.h>
#include <DriverKit/OSData.h>

#include "RNDISaurusDiagnosticsService.h"
#include "RNDISaurusDiagnosticsUserClient.h"
#include "RNDISaurusDriverPrivate.hpp"

struct RNDISaurusDiagnosticsUserClient_IVars {
    RNDISaurusDriver * driver;         // the provider, retained until free
    IODispatchQueue *  driverQueue;    // the driver's default queue, retained until free
    bool               fallbackLogged; // the off-queue fallback has been reported once
};

// One entry per RNDISaurusDiagnosticsSelector. The superclass rejects calls whose
// argument counts and sizes differ from these before the function runs.
static const IOUserClientMethodDispatch kMethods[kRNDISaurusDiagnosticsSelectorCount] = {
    // kRNDISaurusDiagnosticsSelectorSnapshot: no completion, no scalars, no input
    // structure, one RNDISaurusSnapshot out.
    {
        reinterpret_cast<IOUserClientMethodFunction>(&RNDISaurusDiagnosticsUserClient::StaticSnapshot),
        false,
        0,
        0,
        0,
        sizeof(RNDISaurusSnapshot),
    },
};

// MARK: - Lifecycle

bool
RNDISaurusDiagnosticsUserClient::init()
{
    if (!super::init()) {
        return false;
    }
    ivars = IONewZero(RNDISaurusDiagnosticsUserClient_IVars, 1);
    return ivars != nullptr;
}

void
RNDISaurusDiagnosticsUserClient::free()
{
    if (ivars != nullptr) {
        OSSafeReleaseNULL(ivars->driverQueue);
        OSSafeReleaseNULL(ivars->driver);
    }
    IOSafeDeleteNULL(ivars, RNDISaurusDiagnosticsUserClient_IVars, 1);
    super::free();
}

kern_return_t
IMPL(RNDISaurusDiagnosticsUserClient, Start)
{
    kern_return_t ret = Start(provider, SUPERDISPATCH);
    if (ret != kIOReturnSuccess) {
        RNDLOG("user client: super Start failed: 0x%08x", ret);
        return ret;
    }

    RNDISaurusDiagnosticsService * service = OSDynamicCast(RNDISaurusDiagnosticsService, provider);
    if (service == nullptr) {
        RNDLOG("user client: provider %{public}p is not a RNDISaurusDiagnosticsService", provider);
        ret = kIOReturnBadArgument;
    } else {
        ret = service->CopyDriver(&ivars->driver, &ivars->driverQueue);
        if (ret != kIOReturnSuccess) {
            RNDLOG("user client: the diagnostics service has no driver yet: 0x%08x", ret);
        } else {
            ret = SetDispatchQueue(kIOUserClientQueueNameExternalMethod, ivars->driverQueue);
            if (ret != kIOReturnSuccess) {
                RNDLOG("user client: binding ExternalMethod to the driver's queue failed: 0x%08x", ret);
            }
        }
    }

    if (ret != kIOReturnSuccess) {
        // Same as the driver's failed Start: stop the superclass so the kernel side is
        // not left started. The retained objects go with free.
        kern_return_t stopRet = Stop(provider, SUPERDISPATCH);
        if (stopRet != kIOReturnSuccess) {
            RNDLOG("user client: super Stop after the failed Start returned 0x%08x", stopRet);
        }
        return ret;
    }
    RNDLOG("user client %{public}p started for driver %{public}p; snapshot method runs on the driver's queue",
           this, ivars->driver);
    return kIOReturnSuccess;
}

kern_return_t
IMPL(RNDISaurusDiagnosticsUserClient, Stop)
{
    RNDLOG("user client %{public}p stopping", this);
    kern_return_t ret = Stop(provider, SUPERDISPATCH);
    if (ret != kIOReturnSuccess) {
        RNDLOG("user client: super Stop failed: 0x%08x", ret);
    }
    return ret;
}

// MARK: - External methods

kern_return_t
RNDISaurusDiagnosticsUserClient::ExternalMethod(uint64_t selector, IOUserClientMethodArguments * arguments,
                                                const IOUserClientMethodDispatch * dispatch,
                                                OSObject * target, void * reference)
{
    if (selector >= kRNDISaurusDiagnosticsSelectorCount) {
        return kIOReturnBadArgument;
    }
    dispatch = &kMethods[selector];
    if (target == nullptr) {
        target = this;
    }
    return super::ExternalMethod(selector, arguments, dispatch, target, reference);
}

kern_return_t
RNDISaurusDiagnosticsUserClient::StaticSnapshot(OSObject * target, void * reference,
                                                IOUserClientMethodArguments * arguments)
{
    (void)reference;
    RNDISaurusDiagnosticsUserClient * self = OSDynamicCast(RNDISaurusDiagnosticsUserClient, target);
    if (self == nullptr) {
        return kIOReturnBadArgument;
    }
    return self->Snapshot(arguments);
}

// Argument sizes were checked against kMethods by the superclass. The snapshot is taken
// on the driver's queue (see the file comment) and handed back as in-band structure
// output; the framework consumes the OSData.
kern_return_t
RNDISaurusDiagnosticsUserClient::Snapshot(IOUserClientMethodArguments * arguments)
{
    if (ivars->driver == nullptr || ivars->driverQueue == nullptr) {
        return kIOReturnNotAttached;
    }
    const RNDISaurusDriver_IVars * driverIvars = ivars->driver->ivars;
    RNDISaurusSnapshot snapshot;
    if (ivars->driverQueue->OnQueue()) {
        FillSnapshot(driverIvars, &snapshot);
    } else {
        if (!ivars->fallbackLogged) {
            ivars->fallbackLogged = true;
            RNDLOG("user client: snapshot method is not on the driver's queue; dispatching synchronously");
        }
        RNDISaurusSnapshot * out = &snapshot;
        ivars->driverQueue->DispatchSync(^{
            FillSnapshot(driverIvars, out);
        });
    }
    arguments->structureOutput = OSData::withBytes(&snapshot, sizeof(snapshot));
    return arguments->structureOutput != nullptr ? kIOReturnSuccess : kIOReturnNoMemory;
}
