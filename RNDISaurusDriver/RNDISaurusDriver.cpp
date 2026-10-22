//
//  RNDISaurusDriver.cpp
//  RNDISaurus — USB tethering driver for RNDIS devices on macOS
//
//  Copyright (C) 2026 N4H4
//  SPDX-License-Identifier: MPL-2.0
//
//  This Source Code Form is subject to the terms of the Mozilla Public License,
//  v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
//  one at https://mozilla.org/MPL/2.0/.
//
//  Behaviour informed by jwise/HoRNDIS and prostec-labs/TetherKit. Creating the
//  diagnostics child service follows Apple's DriverKitUserClientSample, and the
//  Start, Stop and power-state handling Apple's NetworkingDriverKitSample (both
//  Copyright © 2025 Apple Inc., MIT; notice in NOTICE.md).
//

// Lifecycle, USB plumbing and the diagnostics hooks (error record, snapshot, the
// diagnostics child service). The RNDIS control plane (commands over the default
// control endpoint, bring-up retries, interrupt endpoint) lives in
// RNDISaurusControlPlane.cpp, the Ethernet interface and the bulk data transfers in
// RNDISaurusDataPath.cpp; the instance variables and the logging macro they share are
// in RNDISaurusDriverPrivate.hpp.

#include <string.h>

#include <DriverKit/IOKitKeys.h>
#include <DriverKit/OSNumber.h>
#include <DriverKit/OSString.h>

#include "RNDISaurusDiagnosticsService.h"
#include "RNDISaurusDriverPrivate.hpp"

// MARK: - Constants

// CDC-Data interface class; the RNDIS data interface always has this class (USB CDC
// 1.2, table 6). Not defined by USBDriverKit.
static const uint8_t kUSBInterfaceClassCDCData = 10;

// The diagnostics header documents the SDK values the snapshot carries verbatim; these
// keep it honest.
#define PIN_CONTRACT_VALUE(contract, sdk)                                                   \
    static_assert(static_cast<uint32_t>(contract) == static_cast<uint32_t>(sdk),           \
                  #contract " must equal " #sdk)
PIN_CONTRACT_VALUE(kRNDISaurusSpeedNone, kIOUSBHostConnectionSpeedNone);
PIN_CONTRACT_VALUE(kRNDISaurusSpeedFull, kIOUSBHostConnectionSpeedFull);
PIN_CONTRACT_VALUE(kRNDISaurusSpeedLow, kIOUSBHostConnectionSpeedLow);
PIN_CONTRACT_VALUE(kRNDISaurusSpeedHigh, kIOUSBHostConnectionSpeedHigh);
PIN_CONTRACT_VALUE(kRNDISaurusSpeedSuper, kIOUSBHostConnectionSpeedSuper);
PIN_CONTRACT_VALUE(kRNDISaurusSpeedSuperPlus, kIOUSBHostConnectionSpeedSuperPlus);
PIN_CONTRACT_VALUE(kRNDISaurusSpeedSuperPlusBy2, kIOUSBHostConnectionSpeedSuperPlusBy2);
PIN_CONTRACT_VALUE(kRNDISaurusMediaEthernetAuto, kIOUserNetworkMediaEthernetAuto);
PIN_CONTRACT_VALUE(kRNDISaurusMediaEthernet100BaseTX, kIOUserNetworkMediaEthernet100BaseTX);
PIN_CONTRACT_VALUE(kRNDISaurusMediaEthernet1000BaseT, kIOUserNetworkMediaEthernet1000BaseT);
#undef PIN_CONTRACT_VALUE

// MARK: - Registry properties

// Reads a numeric registry property from the provider or one of its parents in the
// service plane (idVendor/idProduct live on the device nub above the interface).
// Returns kRNDISaurusUnknownID when no OSNumber is found.
static uint32_t
ReadNumberProperty(IOService * provider, const char * key)
{
    OSContainer * container = nullptr;
    kern_return_t ret = provider->SearchProperty(key, kIOServicePlane,
                                                 kIOServiceSearchPropertyParents, &container);
    uint32_t value = kRNDISaurusUnknownID;
    if (ret == kIOReturnSuccess && container != nullptr) {
        OSNumber * number = OSDynamicCast(OSNumber, container);
        if (number != nullptr) {
            value = number->unsigned32BitValue();
        }
    }
    OSSafeReleaseNULL(container);
    return value;
}

// Reads a string registry property the same way into `out` (always NUL-terminated,
// truncated to `capacity` - 1 characters, empty when no OSString is found).
static void
ReadStringProperty(IOService * provider, const char * key, char * out, size_t capacity)
{
    out[0] = '\0';
    OSContainer * container = nullptr;
    kern_return_t ret = provider->SearchProperty(key, kIOServicePlane,
                                                 kIOServiceSearchPropertyParents, &container);
    if (ret == kIOReturnSuccess && container != nullptr) {
        OSString * string = OSDynamicCast(OSString, container);
        const char * source = string != nullptr ? string->getCStringNoCopy() : nullptr;
        if (source != nullptr) {
            size_t i = 0;
            for (; i + 1 < capacity && source[i] != '\0'; i++) {
                out[i] = source[i];
            }
            out[i] = '\0';
        }
    }
    OSSafeReleaseNULL(container);
}

// Records which device matched (VID:PID and product string, for the diagnostics
// snapshot) and logs the personality: the interface class triple and the interface
// number (the RNDIS control interface). The property names are the matching keys
// published on IOUSBHostInterface nubs (IOUSBHostFamilyDefinitions.h).
static void
RecordMatchedInterface(RNDISaurusDriver_IVars * ivars, IOService * provider)
{
    uint32_t interfaceClass    = ReadNumberProperty(provider, kUSBHostMatchingPropertyInterfaceClass);
    uint32_t interfaceSubClass = ReadNumberProperty(provider, kUSBHostMatchingPropertyInterfaceSubClass);
    uint32_t interfaceProtocol = ReadNumberProperty(provider, kUSBHostMatchingPropertyInterfaceProtocol);
    uint32_t interfaceNumber   = ReadNumberProperty(provider, kUSBHostMatchingPropertyInterfaceNumber);
    ivars->vendorID  = ReadNumberProperty(provider, kUSBHostMatchingPropertyVendorID);
    ivars->productID = ReadNumberProperty(provider, kUSBHostMatchingPropertyProductID);
    ReadStringProperty(provider, kUSBHostDevicePropertyProductString,
                       ivars->productName, sizeof(ivars->productName));

    if (interfaceClass == kRNDISaurusUnknownID) {
        uint64_t registryID = 0;
        provider->GetRegistryEntryID(&registryID);
        RNDLOG("Start: matched, but provider properties are unreadable (registry ID 0x%llx)",
               registryID);
    } else {
        RNDLOG("Start: matched %u/%u/%u on interface %u of %04x:%04x \"%{public}s\"",
               interfaceClass, interfaceSubClass, interfaceProtocol,
               interfaceNumber, ivars->vendorID, ivars->productID, ivars->productName);
    }
}

// MARK: - Diagnostics

void
RecordError(RNDISaurusDriver_IVars * ivars, uint32_t stage, kern_return_t status)
{
    ivars->lastErrorStage   = stage;
    ivars->lastError        = status;
    ivars->lastErrorAttempt = ivars->bringUpAttempts;
    ivars->errorEvents++;
}

void
FillSnapshot(const RNDISaurusDriver_IVars * ivars, RNDISaurusSnapshot * snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->size    = sizeof(*snapshot);
    snapshot->version = kRNDISaurusDiagnosticsVersion;

    snapshot->linkUpAtNs = ivars->linkUpAtNs;

    snapshot->vendorID               = ivars->vendorID;
    snapshot->productID              = ivars->productID;
    snapshot->deviceSpeed            = ivars->deviceSpeed;
    snapshot->controlInterfaceNumber = ivars->controlInterfaceNumber;
    snapshot->dataInterfaceNumber    = ivars->dataInterfaceNumber;
    memcpy(snapshot->macAddress, ivars->macAddress, sizeof(snapshot->macAddress));

    snapshot->bringUpState          = ivars->bringUpState;
    snapshot->bringUpAttempts       = ivars->bringUpAttempts;
    snapshot->resets                = ivars->resets;
    snapshot->nextRetryDelayMs      = ivars->nextRetryDelayMs;
    snapshot->deviceMaxTransferSize = ivars->deviceMaxTransferSize;
    snapshot->maxPacketsPerTransfer = ivars->maxPacketsPerTransfer;
    snapshot->packetAlignmentFactor = ivars->packetAlignmentFactor;
    snapshot->activeMediaType       = ivars->activeMediaType;

    snapshot->rndisInitialized   = ivars->rndisInitialized;
    snapshot->filterSet          = ivars->filterSet;
    snapshot->interfaceRegistered = ivars->interfaceRegistered;
    snapshot->interfaceEnabled   = ivars->interfaceEnabled;
    snapshot->mediaConnected     = ivars->mediaConnected;
    snapshot->linkReportedActive = ivars->linkReportedActive;
    snapshot->rxParked           = ivars->rxRecovery.parked;
    snapshot->txParked           = ivars->txRecovery.parked;
    snapshot->interruptParked    = ivars->interruptParked;

    snapshot->lastErrorStage   = ivars->lastErrorStage;
    snapshot->lastErrorAttempt = ivars->lastErrorAttempt;
    snapshot->lastError        = ivars->lastError;
    snapshot->errorEvents      = ivars->errorEvents;

    snapshot->rxPackets        = ivars->rxPackets;
    snapshot->rxBytes          = ivars->rxBytes;
    snapshot->rxDropsNoBuffer  = ivars->rxDropsNoBuffer;
    snapshot->rxDropsMalformed = ivars->rxDropsMalformed;
    snapshot->rxDropsOversize  = ivars->rxDropsOversize;
    snapshot->rxTransferErrors = ivars->rxTransferErrors;
    snapshot->rxRecoveries     = ivars->rxRecovery.recoveries;
    snapshot->rxTransfers      = ivars->rxTransfers;
    snapshot->rxBatchedTransfers = ivars->rxBatchedTransfers;

    snapshot->txPackets      = ivars->txPackets;
    snapshot->txBytes        = ivars->txBytes;
    snapshot->txDropsOversize = ivars->txDropsOversize;
    snapshot->txDropsAborted = ivars->txDropsAborted;
    snapshot->txDropsParked  = ivars->txDropsParked;
    snapshot->txErrors       = ivars->txErrors;
    snapshot->txRecoveries   = ivars->txRecovery.recoveries;
    snapshot->txPads         = ivars->txPads;

    snapshot->notifications        = ivars->notifications;
    snapshot->unknownNotifications = ivars->unknownNotifications;
    snapshot->drainedMessages      = ivars->drainedMessages;
    snapshot->droppedMessages      = ivars->droppedMessages;
    snapshot->indicateStatusCount  = ivars->indicateStatusCount;
    snapshot->keepalivesAnswered   = ivars->keepalivesAnswered;
    snapshot->interruptErrors      = ivars->interruptErrors;

    memcpy(snapshot->productName, ivars->productName, sizeof(snapshot->productName));
    snapshot->productName[sizeof(snapshot->productName) - 1] = '\0';
}

// Creates and publishes the child service the host app opens for diagnostics (see
// RNDISaurusDiagnosticsService.iig for why the driver itself cannot be opened). The
// child gets the driver and its queue before it is published, so a user client can
// never see it half set up. Failure is logged and tolerated: the driver works without
// diagnostics.
static void
CreateDiagnosticsService(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    IOService * created = nullptr;
    kern_return_t ret = self->Create(self, "DiagnosticsServiceProperties", &created);
    if (ret != kIOReturnSuccess || created == nullptr) {
        RNDLOG("diagnostics service: Create failed: 0x%08x; continuing without diagnostics", ret);
        return;
    }
    RNDISaurusDiagnosticsService * service = OSDynamicCast(RNDISaurusDiagnosticsService, created);
    if (service == nullptr) {
        RNDLOG("diagnostics service: the created object is not a RNDISaurusDiagnosticsService");
        created->release();
        return;
    }
    service->SetDriver(self, ivars->defaultQueue);
    ret = service->RegisterService();
    if (ret != kIOReturnSuccess) {
        RNDLOG("diagnostics service: RegisterService failed: 0x%08x", ret);
    }
    ivars->diagnosticsService = service; // Create returned it retained
    RNDLOG("diagnostics service %{public}p published", service);
}

// Terminates the child service (its user clients go with it) and drops the reference.
// On unplug the kernel has already terminated the child along with its provider, and
// the child's Stop has run before ours; the call then reports kIOReturnIPCError, which
// is expected. It matters on the other teardown path, a failed Start.
static void
TerminateDiagnosticsService(RNDISaurusDriver_IVars * ivars)
{
    if (ivars->diagnosticsService == nullptr) {
        return;
    }
    kern_return_t ret = ivars->diagnosticsService->Terminate(0);
    if (ret != kIOReturnSuccess && ret != kIOReturnIPCError) {
        RNDLOG("diagnostics service: Terminate returned 0x%08x", ret);
    }
    OSSafeReleaseNULL(ivars->diagnosticsService);
}

// MARK: - USB descriptors and interfaces

static const char *
SpeedName(uint8_t speed)
{
    switch (speed) {
        case kIOUSBHostConnectionSpeedLow:         return "low";
        case kIOUSBHostConnectionSpeedFull:        return "full";
        case kIOUSBHostConnectionSpeedHigh:        return "high";
        case kIOUSBHostConnectionSpeedSuper:       return "super";
        case kIOUSBHostConnectionSpeedSuperPlus:   return "super+";
        case kIOUSBHostConnectionSpeedSuperPlusBy2: return "super+ x2";
        default:                                   return "unknown";
    }
}

// First endpoint of `interface` with the given transfer type and direction, or NULL.
static const IOUSBEndpointDescriptor *
FindEndpoint(const IOUSBConfigurationDescriptor * configuration,
             const IOUSBInterfaceDescriptor * interface, uint8_t type, uint8_t direction)
{
    const IOUSBEndpointDescriptor * endpoint = nullptr;
    while ((endpoint = IOUSBGetNextEndpointDescriptor(
                configuration, interface,
                reinterpret_cast<const IOUSBDescriptorHeader *>(endpoint))) != nullptr) {
        if (IOUSBGetEndpointType(endpoint) == type && IOUSBGetEndpointDirection(endpoint) == direction) {
            return endpoint;
        }
    }
    return nullptr;
}

// Opens `interface` for exclusive use by this driver; `name` identifies it in the log.
static kern_return_t
OpenInterface(RNDISaurusDriver * self, IOUSBHostInterface * interface, const char * name)
{
    kern_return_t ret = interface->Open(self, 0, nullptr);
    if (ret != kIOReturnSuccess) {
        RNDLOG("Open on %{public}s interface failed: 0x%08x%{public}s", name, ret,
               ret == kIOReturnExclusiveAccess ? " (another client holds it; check ioreg)" : "");
    }
    return ret;
}

// Closes and releases an interface opened with OpenInterface; a no-op when *interface
// is NULL. On unplug the close reports an error because the device is already gone.
static void
CloseAndRelease(RNDISaurusDriver * self, IOUSBHostInterface ** interface, const char * name)
{
    if (*interface == nullptr) {
        return;
    }
    kern_return_t ret = (*interface)->Close(self, 0);
    if (ret != kIOReturnSuccess) {
        RNDLOG("Close on %{public}s interface failed: 0x%08x", name, ret);
    }
    OSSafeReleaseNULL(*interface);
}

// Copies the pipe for `endpoint` into *pipe and records its max packet size at the
// device's connection speed. *pipe is NULL on failure.
static kern_return_t
CopyPipeForEndpoint(IOUSBHostInterface * interface, uint8_t speed,
                    const IOUSBEndpointDescriptor * endpoint, const char * name,
                    IOUSBHostPipe ** pipe, uint16_t * maxPacketSize)
{
    const uint8_t address = IOUSBGetEndpointAddress(endpoint);
    kern_return_t ret = interface->CopyPipe(address, pipe);
    if (ret != kIOReturnSuccess) {
        RNDLOG("CopyPipe for %{public}s endpoint 0x%02x failed: 0x%08x", name, address, ret);
        *pipe = nullptr;
        return ret;
    }
    *maxPacketSize = IOUSBGetEndpointMaxPacketSize(speed, endpoint);
    RNDLOG("%{public}s endpoint 0x%02x: %u-byte packets, interval %u",
           name, address, *maxPacketSize, endpoint->bInterval);
    return kIOReturnSuccess;
}

// The descriptor of the RNDIS data interface, chosen from the configuration descriptor
// alone: the CDC-Data interface numbered control + 1, alternate setting 0. That
// numbering is what RNDIS devices follow in practice; the CDC Union descriptor that
// should name the data interface is wrong on some Android gadgets, so it is ignored.
// Every interface in the configuration is logged on the way. Returns NULL when there
// is no such interface; the result points into `configuration`.
static const IOUSBInterfaceDescriptor *
DataInterfaceDescriptor(const IOUSBConfigurationDescriptor * configuration, uint8_t controlNumber)
{
    const IOUSBInterfaceDescriptor * chosen = nullptr;
    for (const IOUSBInterfaceDescriptor * descriptor = IOUSBGetNextInterfaceDescriptor(configuration, nullptr);
         descriptor != nullptr;
         descriptor = IOUSBGetNextInterfaceDescriptor(
             configuration, reinterpret_cast<const IOUSBDescriptorHeader *>(descriptor))) {
        RNDLOG("configuration interface %u alt %u: class %u/%u/%u, %u endpoint(s)",
               descriptor->bInterfaceNumber, descriptor->bAlternateSetting, descriptor->bInterfaceClass,
               descriptor->bInterfaceSubClass, descriptor->bInterfaceProtocol, descriptor->bNumEndpoints);
        if (chosen == nullptr
            && descriptor->bInterfaceNumber == static_cast<uint8_t>(controlNumber + 1)
            && descriptor->bAlternateSetting == 0
            && descriptor->bInterfaceClass == kUSBInterfaceClassCDCData) {
            chosen = descriptor;
        }
    }
    return chosen;
}

// The interface object the device publishes for interface `number`, retained, or NULL.
// A device hands out its interfaces only through an iterator; every reference it
// returns is retained, and all but the match are released at once.
static IOUSBHostInterface *
CopyInterfaceNumbered(IOUSBHostDevice * device, const IOUSBConfigurationDescriptor * configuration,
                      uint8_t number)
{
    uintptr_t iterator = 0;
    kern_return_t ret = device->CreateInterfaceIterator(&iterator);
    if (ret != kIOReturnSuccess) {
        RNDLOG("CreateInterfaceIterator failed: 0x%08x", ret);
        return nullptr;
    }
    IOUSBHostInterface * match = nullptr;
    while (match == nullptr) {
        IOUSBHostInterface * interface = nullptr;
        if (device->CopyInterface(iterator, &interface) != kIOReturnSuccess || interface == nullptr) {
            break;
        }
        const IOUSBInterfaceDescriptor * descriptor = interface->GetInterfaceDescriptor(configuration);
        if (descriptor != nullptr && descriptor->bInterfaceNumber == number) {
            match = interface;
        } else {
            OSSafeReleaseNULL(interface);
        }
    }
    device->DestroyInterfaceIterator(iterator);
    return match;
}

// Reads the interface descriptors, finds and opens the data interface, and copies the
// bulk IN/OUT pipes (data interface) and the interrupt IN pipe (control interface).
// `configuration` must stay alive for the whole call: every descriptor pointer used
// here points into it.
static kern_return_t
OpenInterfacesAndPipes(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars,
                       const IOUSBConfigurationDescriptor * configuration)
{
    const IOUSBInterfaceDescriptor * controlDescriptor =
        ivars->controlInterface->GetInterfaceDescriptor(configuration);
    if (controlDescriptor == nullptr) {
        RNDLOG("control interface has no descriptor in the current configuration");
        return kIOReturnNotFound;
    }
    ivars->controlInterfaceNumber = controlDescriptor->bInterfaceNumber;

    // The parent device reports the connection speed, which the pipe lookups below need
    // (and the snapshot shows even when bring-up stops here), and publishes the data
    // interface.
    IOUSBHostDevice * device = nullptr;
    kern_return_t ret = ivars->controlInterface->CopyDevice(&device);
    if (ret != kIOReturnSuccess || device == nullptr) {
        RNDLOG("CopyDevice failed: 0x%08x", ret);
        return ret != kIOReturnSuccess ? ret : kIOReturnNotFound;
    }
    device->GetSpeed(&ivars->deviceSpeed);

    const IOUSBInterfaceDescriptor * dataDescriptor =
        DataInterfaceDescriptor(configuration, ivars->controlInterfaceNumber);
    if (dataDescriptor == nullptr) {
        RNDLOG("no CDC-Data interface numbered %u next to control interface %u",
               static_cast<uint8_t>(ivars->controlInterfaceNumber + 1), ivars->controlInterfaceNumber);
        OSSafeReleaseNULL(device);
        return kIOReturnNotFound;
    }
    IOUSBHostInterface * dataInterface =
        CopyInterfaceNumbered(device, configuration, dataDescriptor->bInterfaceNumber);
    OSSafeReleaseNULL(device);
    if (dataInterface == nullptr) {
        RNDLOG("data interface %u is described but not published", dataDescriptor->bInterfaceNumber);
        return kIOReturnNotFound;
    }
    ret = OpenInterface(self, dataInterface, "data");
    if (ret != kIOReturnSuccess) {
        OSSafeReleaseNULL(dataInterface);
        return ret;
    }
    ivars->dataInterface       = dataInterface;
    ivars->dataInterfaceNumber = dataDescriptor->bInterfaceNumber;
    RNDLOG("data interface %u opened next to control interface %u (%{public}s speed)",
           ivars->dataInterfaceNumber, ivars->controlInterfaceNumber, SpeedName(ivars->deviceSpeed));

    const IOUSBEndpointDescriptor * bulkIn =
        FindEndpoint(configuration, dataDescriptor, kIOUSBEndpointTypeBulk, kIOUSBEndpointDirectionIn);
    const IOUSBEndpointDescriptor * bulkOut =
        FindEndpoint(configuration, dataDescriptor, kIOUSBEndpointTypeBulk, kIOUSBEndpointDirectionOut);
    if (bulkIn == nullptr || bulkOut == nullptr) {
        RNDLOG("data interface %u lacks a bulk %{public}s endpoint", ivars->dataInterfaceNumber,
               bulkIn == nullptr ? "IN" : "OUT");
        return kIOReturnNotFound;
    }
    ret = CopyPipeForEndpoint(ivars->dataInterface, ivars->deviceSpeed, bulkIn, "bulk IN",
                              &ivars->bulkInPipe, &ivars->bulkInMaxPacketSize);
    if (ret != kIOReturnSuccess) {
        return ret;
    }
    ret = CopyPipeForEndpoint(ivars->dataInterface, ivars->deviceSpeed, bulkOut, "bulk OUT",
                              &ivars->bulkOutPipe, &ivars->bulkOutMaxPacketSize);
    if (ret != kIOReturnSuccess) {
        return ret;
    }

    // The interrupt IN endpoint carries RESPONSE_AVAILABLE notifications. Its absence,
    // or a failure to copy its pipe, is tolerated: responses are polled for anyway.
    const IOUSBEndpointDescriptor * interruptEndpoint =
        FindEndpoint(configuration, controlDescriptor, kIOUSBEndpointTypeInterrupt, kIOUSBEndpointDirectionIn);
    if (interruptEndpoint == nullptr) {
        RNDLOG("control interface has no interrupt IN endpoint; continuing without notifications");
    } else if (CopyPipeForEndpoint(ivars->controlInterface, ivars->deviceSpeed, interruptEndpoint, "interrupt IN",
                                   &ivars->interruptPipe, &ivars->interruptMaxPacketSize) != kIOReturnSuccess) {
        RNDLOG("continuing without notifications");
    }
    return kIOReturnSuccess;
}

kern_return_t
CreateMappedBuffer(uint64_t direction, uint32_t capacity, const char * name,
                   IOBufferMemoryDescriptor ** buffer, uint8_t ** bytes)
{
    *bytes = nullptr;
    kern_return_t ret = IOBufferMemoryDescriptor::Create(direction, capacity, 0, buffer);
    if (ret != kIOReturnSuccess || *buffer == nullptr) {
        RNDLOG("%{public}s: IOBufferMemoryDescriptor::Create(%u) failed: 0x%08x", name, capacity, ret);
        return ret != kIOReturnSuccess ? ret : kIOReturnNoMemory;
    }
    // A fresh buffer descriptor has zero length until told otherwise; make the whole
    // capacity available so the length passed to each transfer alone governs it.
    ret = (*buffer)->SetLength(capacity);
    if (ret != kIOReturnSuccess) {
        RNDLOG("%{public}s: SetLength(%u) failed: 0x%08x", name, capacity, ret);
        return ret;
    }
    IOAddressSegment range = {};
    ret = (*buffer)->GetAddressRange(&range);
    if (ret != kIOReturnSuccess || range.address == 0 || range.length < capacity) {
        RNDLOG("%{public}s: GetAddressRange failed: 0x%08x (address 0x%llx, length %llu)",
               name, ret, range.address, range.length);
        return ret != kIOReturnSuccess ? ret : kIOReturnNoMemory;
    }
    *bytes = reinterpret_cast<uint8_t *>(range.address);
    return kIOReturnSuccess;
}

// Opens the control interface (the provider), then everything that hangs off it, and
// allocates the EP0 command buffer.
static kern_return_t
OpenUSB(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars, IOService * provider)
{
    IOUSBHostInterface * control = OSDynamicCast(IOUSBHostInterface, provider);
    if (control == nullptr) {
        RNDLOG("provider %{public}p is not an IOUSBHostInterface", provider);
        return kIOReturnBadArgument;
    }
    kern_return_t ret = OpenInterface(self, control, "control");
    if (ret != kIOReturnSuccess) {
        return ret;
    }
    control->retain();
    ivars->controlInterface = control;

    const IOUSBConfigurationDescriptor * configuration = control->CopyConfigurationDescriptor();
    if (configuration == nullptr) {
        RNDLOG("CopyConfigurationDescriptor returned NULL");
        return kIOReturnNotFound;
    }
    ret = OpenInterfacesAndPipes(self, ivars, configuration);
    IOUSBHostFreeDescriptor(configuration);
    if (ret != kIOReturnSuccess) {
        return ret;
    }

    return CreateMappedBuffer(kIOMemoryDirectionOutIn, static_cast<uint32_t>(RNDIS::kControlBufferSize),
                              "command buffer", &ivars->commandBuffer, &ivars->commandBytes);
}

// Undoes OpenUSB: releases the pipes, closes and releases both interfaces, releases
// the command buffer. Safe on a partially opened state and more than once. On unplug
// the interfaces are already gone and the closes report errors; that is expected.
static void
CloseUSB(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    OSSafeReleaseNULL(ivars->bulkInPipe);
    OSSafeReleaseNULL(ivars->bulkOutPipe);
    OSSafeReleaseNULL(ivars->interruptPipe);

    CloseAndRelease(self, &ivars->dataInterface, "data");
    CloseAndRelease(self, &ivars->controlInterface, "control");

    ivars->commandBytes = nullptr;
    OSSafeReleaseNULL(ivars->commandBuffer);
}

// MARK: - Lifecycle

// Everything Start set up, in reverse: the bring-up timer and interrupt reader, the
// Ethernet interface and bulk transfers, the RNDIS farewell (which needs the control
// interface, hence before the USB close), the USB interfaces, and the queue.
static void
TeardownAll(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    TerminateDiagnosticsService(ivars);
    TeardownControlPlane(ivars);
    TeardownNetworking(ivars);
    RNDISFarewell(self, ivars);
    CloseUSB(self, ivars);
    OSSafeReleaseNULL(ivars->defaultQueue);
}

// Everything Start does after the superclass Start: the USB interfaces and pipes, the
// queue the rest of the driver runs on, the diagnostics child service (published now,
// so that the host app can watch a bring-up that keeps failing), and the scheduling of
// the RNDIS bring-up. The bring-up itself runs from the timer once Start has returned,
// and keeps retrying if the device does not cooperate; the Ethernet interface is
// registered, and this service published, by the attempt that succeeds.
static kern_return_t
SetUp(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars, IOService * provider)
{
    kern_return_t ret = OpenUSB(self, ivars, provider);
    if (ret != kIOReturnSuccess) {
        RNDLOG("Start: USB setup failed: 0x%08x", ret);
        RecordError(ivars, kRNDISaurusErrorStageUSBSetup, ret);
        return ret;
    }
    ret = self->CopyDispatchQueue("Default", &ivars->defaultQueue);
    if (ret != kIOReturnSuccess || ivars->defaultQueue == nullptr) {
        RNDLOG("Start: CopyDispatchQueue(Default) failed: 0x%08x", ret);
        return ret != kIOReturnSuccess ? ret : kIOReturnNotFound;
    }
    CreateDiagnosticsService(self, ivars);
    ret = StartBringUp(self, ivars);
    if (ret != kIOReturnSuccess) {
        RNDLOG("Start: scheduling the RNDIS bring-up failed: 0x%08x", ret);
        return ret;
    }
    return kIOReturnSuccess;
}

bool
RNDISaurusDriver::init()
{
    if (!super::init()) {
        return false;
    }
    ivars = IONewZero(RNDISaurusDriver_IVars, 1);
    if (ivars == nullptr) {
        return false;
    }
    ivars->nextRequestID = 1;
    return true;
}

void
RNDISaurusDriver::free()
{
    IOSafeDeleteNULL(ivars, RNDISaurusDriver_IVars, 1);
    super::free();
}

kern_return_t
IMPL(RNDISaurusDriver, Start)
{
    kern_return_t ret = Start(provider, SUPERDISPATCH);
    if (ret != kIOReturnSuccess) {
        RNDLOG("Start: super Start failed: 0x%08x", ret);
        return ret;
    }

    RNDLOG("Start: this=%{public}p provider=%{public}p", this, provider);
    RecordMatchedInterface(ivars, provider);

    ret = SetUp(this, ivars, provider);
    if (ret != kIOReturnSuccess) {
        // Undo everything and stop the superclass too, as Apple's
        // NetworkingDriverKitSample does on this path, so that the kernel side is not
        // left started and the dext process can exit.
        TeardownAll(this, ivars);
        kern_return_t stopRet = Stop(provider, SUPERDISPATCH);
        if (stopRet != kIOReturnSuccess) {
            RNDLOG("Start: super Stop after the failed Start returned 0x%08x", stopRet);
        }
        return ret;
    }
    RNDLOG("Start: USB ready, RNDIS bring-up scheduled");
    return kIOReturnSuccess;
}

kern_return_t
IMPL(RNDISaurusDriver, Stop)
{
    RNDLOG("Stop: this=%{public}p provider=%{public}p", this, provider);

    TeardownAll(this, ivars);

    kern_return_t ret = Stop(provider, SUPERDISPATCH);
    if (ret != kIOReturnSuccess) {
        RNDLOG("Stop: super Stop failed: 0x%08x", ret);
    }
    return ret;
}

kern_return_t
IMPL(RNDISaurusDriver, SetPowerState)
{
    const char * name = "other";
    switch (powerFlags) {
        case kIOServicePowerCapabilityOff: name = "off (system sleeping)"; break;
        case kIOServicePowerCapabilityOn:  name = "on";                    break;
        case kIOServicePowerCapabilityLow: name = "low";                   break;
        default:                                                           break;
    }
    RNDLOG("SetPowerState(0x%08x): %{public}s", powerFlags, name);
    return SetPowerState(powerFlags, SUPERDISPATCH);
}
