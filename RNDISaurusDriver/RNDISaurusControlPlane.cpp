//
//  RNDISaurusControlPlane.cpp
//  RNDISaurus — USB tethering driver for RNDIS devices on macOS
//
//  Copyright (C) 2026 N4H4
//  SPDX-License-Identifier: MPL-2.0
//
//  This Source Code Form is subject to the terms of the Mozilla Public License,
//  v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
//  one at https://mozilla.org/MPL/2.0/.
//
//  Implemented from Microsoft's [MS-RNDIS] and "Remote NDIS to USB Mapping".
//  Behaviour informed by the Linux RNDIS host driver and jwise/HoRNDIS (the command
//  exchange over the control endpoint) and by JellyBrick/ReRNDIS and
//  XiaoMiku01/TetherKit (RESET before retrying a failed bring-up). The bring-up timer
//  follows Apple's NetworkingDriverKitSample (Copyright © 2025 Apple Inc., MIT;
//  notice in NOTICE.md).
//

// The RNDIS control plane: encapsulated commands over the default control endpoint,
// the bring-up state machine that runs them (INITIALIZE → permanent MAC address →
// packet filter) and retries them after a RESET when the device does not cooperate,
// and the interrupt endpoint reader that picks up device-initiated messages between
// commands.
//
// Everything here runs on the driver's default dispatch queue. A command waits for
// its completion synchronously, so the timer and interrupt callbacks queue up behind
// it and never overlap it; that is why no state is locked. The cost is that a command
// holds the queue the data plane runs on while it waits, which is why every command
// has a fixed time budget and drains use a much shorter timeout.

#include <string.h>
#include <time.h>

#include "RNDISaurusDriverPrivate.hpp"

// MARK: - Constants

// CDC class requests carrying RNDIS messages over the default control endpoint
// (Remote NDIS to USB Mapping). Both are interface-recipient class requests with
// wValue 0 and wIndex = the control interface number.
static const uint8_t kRequestSendEncapsulatedCommand = 0x00;
static const uint8_t kRequestGetEncapsulatedResponse = 0x01;
static const uint8_t kRequestTypeClassInterfaceOut =
    kIOUSBDeviceRequestDirectionOut | kIOUSBDeviceRequestTypeClass | kIOUSBDeviceRequestRecipientInterface;
static const uint8_t kRequestTypeClassInterfaceIn =
    kIOUSBDeviceRequestDirectionIn | kIOUSBDeviceRequestTypeClass | kIOUSBDeviceRequestRecipientInterface;

// The longest a command may take, from its SEND to its completion; it also caps any
// single control request. The Unisoc phone sometimes leaves a QUERY unanswered past
// it; bring-up then counts the attempt as failed (RESET, then retry).
static const uint32_t kCommandTimeoutMs = 5000;

// Completion timeout of a GET issued to drain the response queue after a notification,
// and the most GETs one drain issues. A drain only runs when the device has announced
// more messages than have been read, so its GET should find one at once; some devices
// (the Unisoc phone) hold an empty GET until it times out instead of stalling it, so
// the timeout is short to bound how long a drain can occupy the queue the data plane
// runs on.
static const uint32_t kDrainTimeoutMs = 500;
static const uint32_t kMaxDrainGets   = 4;

// Pause before asking again when a GET found the response queue empty: short at first,
// so a device that queues its completion within a few milliseconds is answered almost
// at once, then doubling up to the cap, so a slow one is asked at most ten times a
// second.
static const uint32_t kFirstNotReadyWaitMs = 5;
static const uint32_t kMaxNotReadyWaitMs   = 100;

// GETs one command may issue. The time budget already bounds the waits; this bounds a
// device that keeps putting other messages ahead of the completion.
static const uint32_t kMaxGetsPerCommand = 64;

// Bring-up retries: the delay doubles from the first value up to the cap, and the
// attempts continue for as long as the device stays attached. Every attempt up to
// kLoggedAttempts is logged, then one in kAttemptLogPeriod.
static const uint32_t kFirstRetryDelayMs        = 1000;
static const uint32_t kMaxRetryDelayMs          = 30000;
static const uint32_t kLoggedAttempts           = 6;
static const uint32_t kAttemptLogPeriod         = 10;
static const uint64_t kNanosecondsPerMillisecond = 1000000ull;

// Consecutive interrupt transfer errors tolerated before the reader is parked.
// Without it the driver still works: responses are polled, only device-initiated
// messages between commands go unread.
static const uint32_t kMaxInterruptErrors = 3;

// MARK: - Forward declarations

static kern_return_t UpdateMediaState(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars, uint32_t received);
static void          ArmInterrupt(RNDISaurusDriver_IVars * ivars);

// MARK: - EP0 transport

// True while the control interface is open and the command buffer mapped, i.e. until
// the USB close in teardown. Both are set and cleared together by the USB plumbing.
static bool
ControlEndpointUsable(const RNDISaurusDriver_IVars * ivars)
{
    return ivars->controlInterface != nullptr && ivars->commandBytes != nullptr;
}

// The device queues each response before it announces it with RESPONSE_AVAILABLE, and
// every GET that returned a message consumed one, whoever issued it. When the device
// drops its queue (RESET) or a GET failed, whatever was announced is not coming any
// more and the ledger is settled so that no drain asks for it.
static void
ResyncResponseLedger(RNDISaurusDriver_IVars * ivars)
{
    ivars->responsesConsumed = ivars->responsesAnnounced;
}

static uint32_t
NextRequestID(RNDISaurusDriver_IVars * ivars)
{
    uint32_t id = ivars->nextRequestID++;
    if (id == 0) {
        id = ivars->nextRequestID++;
    }
    return id;
}

// SEND_ENCAPSULATED_COMMAND with the first `length` bytes of the command buffer.
static kern_return_t
SendEncapsulatedCommand(RNDISaurusDriver_IVars * ivars, size_t length)
{
    uint16_t sent = 0;
    kern_return_t ret = ivars->controlInterface->DeviceRequest(
        kRequestTypeClassInterfaceOut, kRequestSendEncapsulatedCommand, 0,
        ivars->controlInterfaceNumber, static_cast<uint16_t>(length),
        ivars->commandBuffer, &sent, kCommandTimeoutMs);
    if (ret != kIOReturnSuccess) {
        RNDLOG("SEND_ENCAPSULATED_COMMAND (%u bytes) failed: 0x%08x", static_cast<unsigned>(length), ret);
        return ret;
    }
    if (sent != length) {
        RNDLOG("SEND_ENCAPSULATED_COMMAND sent %u of %u bytes", sent, static_cast<unsigned>(length));
        return kIOReturnUnderrun;
    }
    return kIOReturnSuccess;
}

// GET_ENCAPSULATED_RESPONSE into the command buffer; *received is the byte count.
static kern_return_t
GetEncapsulatedResponse(RNDISaurusDriver_IVars * ivars, uint32_t timeoutMs, uint16_t * received)
{
    // Never let a previous reply masquerade as a fresh header.
    memset(ivars->commandBytes, 0, RNDIS::kHeaderSize);
    *received = 0;
    return ivars->controlInterface->DeviceRequest(
        kRequestTypeClassInterfaceIn, kRequestGetEncapsulatedResponse, 0,
        ivars->controlInterfaceNumber, static_cast<uint16_t>(RNDIS::kControlBufferSize),
        ivars->commandBuffer, received, timeoutMs);
}

// One GET_ENCAPSULATED_RESPONSE, classified against the completion being waited for
// (type without the completion flag and RequestID; 0 when none is). A device with
// nothing queued either answers with fewer than 8 bytes, which the protocol layer
// reports as NotReady, or stalls the control endpoint (Linux gadgets), which is mapped
// to NotReady here; both leave the return value at success. Any other transport error
// is returned as is: the device is gone or wedged. Every message that did come out
// counts as consumed from the device's response queue.
static kern_return_t
ReceiveResponse(RNDISaurusDriver_IVars * ivars, uint32_t timeoutMs, uint32_t expectedMessageType,
                uint32_t expectedRequestID, RNDIS::Header * header, uint16_t * received,
                RNDIS::ResponseKind * kind)
{
    kern_return_t ret = GetEncapsulatedResponse(ivars, timeoutMs, received);
    if (ret == kUSBHostReturnPipeStalled) {
        *kind = RNDIS::ResponseKind::NotReady;
        return kIOReturnSuccess;
    }
    if (ret != kIOReturnSuccess) {
        return ret;
    }
    *kind = RNDIS::ClassifyResponse(ivars->commandBytes, *received, expectedMessageType,
                                    expectedRequestID, header);
    if (*kind != RNDIS::ResponseKind::NotReady) {
        ivars->responsesConsumed++;
    }
    return kIOReturnSuccess;
}

// MARK: - Device-initiated and stale messages

// Deals with a message that came out of the device's response queue but is not the
// completion a command is waiting for; the command loop and the notification drain
// both pass such messages here. An INDICATE_STATUS updates the media state. A KEEPALIVE
// from the device gets its KEEPALIVE_CMPLT, written over the command buffer: a command
// in flight no longer needs it, since its message already went out. Anything else (a
// late completion, an unknown type, a malformed message) is counted and dropped.
// Returns an error only when the KEEPALIVE_CMPLT could not be sent.
static kern_return_t
ServiceDeviceMessage(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars,
                     RNDIS::ResponseKind kind, const RNDIS::Header * header, uint32_t received)
{
    switch (kind) {
        case RNDIS::ResponseKind::IndicateStatus:
            return UpdateMediaState(self, ivars, received);

        case RNDIS::ResponseKind::Keepalive: {
            const size_t replyLength = RNDIS::BuildKeepaliveComplete(
                ivars->commandBytes, RNDIS::kControlBufferSize, header->requestID);
            kern_return_t ret = SendEncapsulatedCommand(ivars, replyLength);
            RNDLOG_COUNTED(ivars->keepalivesAnswered, "answered device KEEPALIVE id %u: 0x%08x",
                           header->requestID, ret);
            return ret;
        }

        case RNDIS::ResponseKind::CompletionOtherRequest:
        case RNDIS::ResponseKind::Unexpected:
            RNDLOG_COUNTED(ivars->droppedMessages, "dropping %{public}s (type 0x%08x, id %u, %u bytes)",
                           RNDIS::MessageTypeName(header->messageType), header->messageType,
                           header->requestID, received);
            return kIOReturnSuccess;

        case RNDIS::ResponseKind::Malformed:
            RNDLOG_COUNTED(ivars->droppedMessages, "dropping malformed response (%u bytes)", received);
            return kIOReturnSuccess;

        case RNDIS::ResponseKind::NotReady:
        case RNDIS::ResponseKind::Completion:
            return kIOReturnSuccess; // not this function's business
    }
    return kIOReturnSuccess;
}

// MARK: - Commands

static uint64_t
UptimeMs()
{
    return clock_gettime_nsec_np(CLOCK_UPTIME_RAW) / kNanosecondsPerMillisecond;
}

// Sends the message that occupies the first `messageLength` bytes of the command buffer
// and, when its type is answered at all, reads GET_ENCAPSULATED_RESPONSE until the
// matching completion comes out. The completion is left in the command buffer and its
// size stored in *responseLength (may be NULL).
//
// The whole exchange gets kCommandTimeoutMs, counted from just before the SEND, and
// each GET may wait only for what is left of it, so a command occupies the queue for
// that long at most, whatever the device does. What a GET brings back decides the next
// step:
// - the awaited completion ends the command;
// - fewer than 8 bytes (what the spec prescribes) or a stalled control endpoint (what
//   Linux gadgets do instead) means nothing is queued yet: pause and ask again;
// - any other message was queued ahead of ours, since the device keeps one FIFO for
//   completions and device-initiated messages alike: ServiceDeviceMessage takes it
//   and the next GET follows at once, because the completion may be right behind it.
// Any other transport error means the device is gone or wedged and ends the command.
static kern_return_t
RNDISCommand(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars, size_t messageLength,
             uint32_t * responseLength)
{
    // Type and RequestID are captured before sending: every GET and any KEEPALIVE_CMPLT
    // reuse the command buffer.
    RNDIS::Header request = {};
    if (!RNDIS::ParseHeader(ivars->commandBytes, messageLength, &request)) {
        RNDLOG("refusing to send a malformed %u-byte message", static_cast<unsigned>(messageLength));
        return kIOReturnBadArgument;
    }
    const uint32_t messageType = request.messageType;
    const uint32_t requestID   = request.requestID;
    const char * completionName = RNDIS::MessageTypeName(messageType | RNDIS::kMessageCompletionFlag);
    const uint64_t startMs    = UptimeMs();
    const uint64_t deadlineMs = startMs + kCommandTimeoutMs;

    kern_return_t ret = SendEncapsulatedCommand(ivars, messageLength);
    if (ret != kIOReturnSuccess) {
        return ret;
    }
    if (!RNDIS::ExpectsCompletion(messageType)) {
        return kIOReturnSuccess;
    }

    uint32_t gets     = 0;
    uint32_t notReady = 0;
    uint32_t pauseMs  = kFirstNotReadyWaitMs;
    for (uint64_t nowMs = UptimeMs(); nowMs < deadlineMs && gets < kMaxGetsPerCommand; nowMs = UptimeMs()) {
        gets++;
        uint16_t received = 0;
        RNDIS::Header header = {};
        RNDIS::ResponseKind kind = RNDIS::ResponseKind::NotReady;
        // At least 1 ms here (the loop condition); a timeout of 0 would mean none at all.
        const uint32_t timeoutMs = static_cast<uint32_t>(deadlineMs - nowMs);
        ret = ReceiveResponse(ivars, timeoutMs, messageType, requestID, &header, &received, &kind);
        if (ret != kIOReturnSuccess) {
            RNDLOG("GET_ENCAPSULATED_RESPONSE failed: 0x%08x while waiting for %{public}s id %u (%llu ms)",
                   ret, completionName, requestID, UptimeMs() - startMs);
            return ret;
        }

        if (kind == RNDIS::ResponseKind::Completion) {
            const uint64_t elapsedMs = UptimeMs() - startMs;
            if (messageType == RNDIS::kMessageTypeReset) {
                // RESET_CMPLT has no RequestID and its fields differ; the caller logs them.
                RNDLOG("RESET_CMPLT: %u bytes after %llu ms (%u GET(s), %u not ready)",
                       received, elapsedMs, gets, notReady);
            } else {
                RNDLOG("%{public}s id %u: %u bytes, status 0x%08x after %llu ms (%u GET(s), %u not ready)",
                       completionName, requestID, received, header.status, elapsedMs, gets, notReady);
            }
            if (responseLength != nullptr) {
                *responseLength = received;
            }
            return kIOReturnSuccess;
        }

        if (kind == RNDIS::ResponseKind::NotReady) {
            notReady++;
            if (UptimeMs() + pauseMs >= deadlineMs) {
                break; // no time left for another GET after the pause
            }
            IOSleep(pauseMs);
            pauseMs = pauseMs * 2 < kMaxNotReadyWaitMs ? pauseMs * 2 : kMaxNotReadyWaitMs;
            continue;
        }

        ret = ServiceDeviceMessage(self, ivars, kind, &header, received);
        if (ret != kIOReturnSuccess) {
            return ret;
        }
    }

    RNDLOG("timed out waiting for %{public}s id %u after %llu ms (%u GET(s), %u not ready)",
           completionName, requestID, UptimeMs() - startMs, gets, notReady);
    return kIOReturnTimeout;
}

static kern_return_t
RNDISInitialize(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    const uint32_t requestID = NextRequestID(ivars);
    const size_t length = RNDIS::BuildInitialize(ivars->commandBytes, RNDIS::kControlBufferSize,
                                                 requestID, RNDIS::kHostMaxTransferSize);
    uint32_t responseLength = 0;
    kern_return_t ret = RNDISCommand(self, ivars, length, &responseLength);
    if (ret != kIOReturnSuccess) {
        return ret;
    }

    RNDIS::InitializeResult result = {};
    if (!RNDIS::ParseInitializeComplete(ivars->commandBytes, responseLength, &result)) {
        RNDLOG("INITIALIZE_CMPLT is malformed (%u bytes)", responseLength);
        return kIOReturnIOError;
    }
    if (result.status != RNDIS::kStatusSuccess) {
        RNDLOG("INITIALIZE failed with status 0x%08x", result.status);
        return kIOReturnIOError;
    }
    ivars->rndisInitialized      = true;
    ivars->deviceMaxTransferSize = result.maxTransferSize;
    ivars->maxPacketsPerTransfer = result.maxPacketsPerTransfer;
    ivars->packetAlignmentFactor = result.packetAlignmentFactor;
    RNDLOG("INITIALIZE_CMPLT: RNDIS %u.%u, device flags 0x%08x, medium %u, "
           "max %u packet(s) per transfer, device MaxTransferSize %u, alignment factor %u "
           "(host offered %u)",
           result.majorVersion, result.minorVersion, result.deviceFlags, result.medium,
           result.maxPacketsPerTransfer, result.maxTransferSize, result.packetAlignmentFactor,
           RNDIS::kHostMaxTransferSize);
    return kIOReturnSuccess;
}

static kern_return_t
RNDISQueryPermanentAddress(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    const uint32_t requestID = NextRequestID(ivars);
    const size_t length = RNDIS::BuildQuery(ivars->commandBytes, RNDIS::kControlBufferSize, requestID,
                                            RNDIS::kOID8023PermanentAddress,
                                            RNDIS::kFixedOIDPlaceholderBytes);
    uint32_t responseLength = 0;
    kern_return_t ret = RNDISCommand(self, ivars, length, &responseLength);
    if (ret != kIOReturnSuccess) {
        return ret;
    }

    uint32_t status = 0;
    const uint8_t * information = nullptr;
    uint32_t informationLength = 0;
    if (!RNDIS::ParseQueryComplete(ivars->commandBytes, responseLength, &status, &information, &informationLength)) {
        RNDLOG("QUERY_CMPLT for the permanent address is malformed (%u bytes)", responseLength);
        return kIOReturnIOError;
    }
    if (status != RNDIS::kStatusSuccess) {
        RNDLOG("QUERY OID_802_3_PERMANENT_ADDRESS failed with status 0x%08x", status);
        return kIOReturnIOError;
    }
    if (informationLength != sizeof(ivars->macAddress)) {
        RNDLOG("permanent address has %u bytes, expected %u", informationLength,
               static_cast<unsigned>(sizeof(ivars->macAddress)));
        return kIOReturnIOError;
    }
    memcpy(ivars->macAddress, information, sizeof(ivars->macAddress));
    RNDLOG("permanent MAC address %02x:%02x:%02x:%02x:%02x:%02x",
           ivars->macAddress[0], ivars->macAddress[1], ivars->macAddress[2],
           ivars->macAddress[3], ivars->macAddress[4], ivars->macAddress[5]);
    return kIOReturnSuccess;
}

// A non-zero filter moves the device into its data-flowing state; zero pauses it.
static kern_return_t
RNDISSetPacketFilter(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars, uint32_t filter)
{
    const uint32_t requestID = NextRequestID(ivars);
    const size_t length = RNDIS::BuildSetPacketFilter(ivars->commandBytes, RNDIS::kControlBufferSize,
                                                      requestID, filter);
    uint32_t responseLength = 0;
    kern_return_t ret = RNDISCommand(self, ivars, length, &responseLength);
    if (ret != kIOReturnSuccess) {
        return ret;
    }

    uint32_t status = 0;
    if (!RNDIS::ParseSetComplete(ivars->commandBytes, responseLength, &status)) {
        RNDLOG("SET_CMPLT for the packet filter is malformed (%u bytes)", responseLength);
        return kIOReturnIOError;
    }
    if (status != RNDIS::kStatusSuccess) {
        RNDLOG("SET OID_GEN_CURRENT_PACKET_FILTER 0x%08x failed with status 0x%08x", filter, status);
        return kIOReturnIOError;
    }
    ivars->filterSet = (filter != 0);
    RNDLOG("packet filter 0x%08x set: %{public}s", filter,
           filter != 0 ? "device is in the data-initialized state" : "data flow paused");
    return kIOReturnSuccess;
}

// RESET is the RNDIS soft reset. Sent before every retried bring-up, it is what gets a
// device that stopped answering commands to respond again. The completion
// is waited for, but its absence is not an error: the following INITIALIZE decides
// whether the device is alive. Linux gadgets always report
// AddressingReset = 1 and drain their response queue, so the packet filter has to be
// SET again afterwards — every bring-up attempt does.
static void
RNDISReset(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    ivars->resets++;
    const size_t length = RNDIS::BuildReset(ivars->commandBytes, RNDIS::kControlBufferSize);
    uint32_t responseLength = 0;
    kern_return_t ret = RNDISCommand(self, ivars, length, &responseLength);
    if (ret != kIOReturnSuccess) {
        RNDLOG("RESET got no completion (0x%08x); continuing with INITIALIZE anyway", ret);
    } else {
        uint32_t status = 0;
        uint32_t addressingReset = 0;
        if (RNDIS::ParseResetComplete(ivars->commandBytes, responseLength, &status, &addressingReset)) {
            RNDLOG("RESET_CMPLT: status 0x%08x, AddressingReset %u", status, addressingReset);
        } else {
            RNDLOG("RESET_CMPLT is malformed (%u bytes); continuing with INITIALIZE anyway", responseLength);
        }
    }
    // Whatever the device said, it no longer holds the state the earlier steps set up,
    // and its response queue is empty.
    ivars->rndisInitialized = false;
    ivars->filterSet        = false;
    ResyncResponseLedger(ivars);
}

// HALT is the one message with no completion: once sent, the device drops all RNDIS
// state without answering, so there is nothing to wait for.
static void
RNDISHalt(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    const uint32_t requestID = NextRequestID(ivars);
    const size_t length = RNDIS::BuildHalt(ivars->commandBytes, RNDIS::kControlBufferSize, requestID);
    kern_return_t ret = RNDISCommand(self, ivars, length, nullptr);
    RNDLOG("HALT sent: 0x%08x", ret);
    ivars->rndisInitialized = false;
}

// INITIALIZE, then the permanent MAC address, then a packet filter that starts data flow.
static kern_return_t
BringUpRNDIS(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    kern_return_t ret = RNDISInitialize(self, ivars);
    if (ret != kIOReturnSuccess) {
        RecordError(ivars, kRNDISaurusErrorStageInitialize, ret);
        return ret;
    }
    ret = RNDISQueryPermanentAddress(self, ivars);
    if (ret != kIOReturnSuccess) {
        RecordError(ivars, kRNDISaurusErrorStageQueryMAC, ret);
        return ret;
    }
    ret = RNDISSetPacketFilter(self, ivars, RNDIS::kDefaultPacketFilter);
    if (ret != kIOReturnSuccess) {
        RecordError(ivars, kRNDISaurusErrorStageSetFilter, ret);
    }
    return ret;
}

void
RNDISFarewell(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    if (ControlEndpointUsable(ivars)) {
        if (ivars->filterSet) {
            kern_return_t ret = RNDISSetPacketFilter(self, ivars, 0);
            if (ret != kIOReturnSuccess) {
                RNDLOG("clearing the packet filter failed: 0x%08x", ret);
            }
        }
        if (ivars->rndisInitialized) {
            RNDISHalt(self, ivars);
        }
    }
    ivars->filterSet        = false;
    ivars->rndisInitialized = false;
}

// MARK: - Media state

// Reads an INDICATE_STATUS out of the command buffer and, for the media codes, updates
// the link reported to the stack. Other codes are only logged.
static kern_return_t
UpdateMediaState(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars, uint32_t received)
{
    uint32_t status = 0;
    if (!RNDIS::ParseIndicateStatus(ivars->commandBytes, received, &status)) {
        RNDLOG_COUNTED(ivars->droppedMessages, "dropping malformed INDICATE_STATUS (%u bytes)", received);
        return kIOReturnSuccess;
    }
    ivars->indicateStatusCount++;
    switch (status) {
        case RNDIS::kStatusMediaConnect:
            ivars->mediaConnected = true;
            break;
        case RNDIS::kStatusMediaDisconnect:
            ivars->mediaConnected = false;
            break;
        default:
            RNDLOG("INDICATE_STATUS 0x%08x (%{public}s) ignored", status, RNDIS::StatusName(status));
            return kIOReturnSuccess;
    }
    RNDLOG("INDICATE_STATUS: %{public}s", RNDIS::StatusName(status));
    UpdateLinkStatus(self, ivars);
    return kIOReturnSuccess;
}

// MARK: - Interrupt endpoint

// The read buffer is sized from the endpoint descriptor (mainline Linux gadgets declare
// 8 bytes, some vendor kernels 512), never below the notification size.
static uint32_t
InterruptBufferSize(const RNDISaurusDriver_IVars * ivars)
{
    const uint32_t minimum = static_cast<uint32_t>(RNDIS::kResponseAvailableSize);
    return ivars->interruptMaxPacketSize > minimum ? ivars->interruptMaxPacketSize : minimum;
}

static void
ArmInterrupt(RNDISaurusDriver_IVars * ivars)
{
    TransferSlot * slot = &ivars->interruptSlot;
    if (slot->inFlight || slot->buffer == nullptr || ivars->interruptPipe == nullptr) {
        return;
    }
    kern_return_t ret = ivars->interruptPipe->AsyncIO(slot->buffer, InterruptBufferSize(ivars),
                                                      slot->action, 0);
    if (ret != kIOReturnSuccess) {
        if (!IsDeviceLeavingStatus(ret)) {
            RNDLOG_COUNTED(ivars->interruptErrors, "interrupt IN: AsyncIO failed: 0x%08x", ret);
            RecordError(ivars, kRNDISaurusErrorStageInterruptIn, ret);
        }
        return;
    }
    slot->inFlight = true;
}

// Creates the interrupt reader's slot and starts the first read. Failure is logged
// and tolerated: responses are polled for anyway.
static void
SetUpInterrupt(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    if (ivars->interruptPipe == nullptr) {
        return; // already logged when the pipes were located
    }
    const uint32_t capacity = InterruptBufferSize(ivars);
    kern_return_t ret = PrepareSlot(self, &ivars->interruptSlot, 0, SlotKind::Interrupt, capacity);
    if (ret != kIOReturnSuccess) {
        ReleaseSlot(&ivars->interruptSlot);
        RNDLOG("continuing without notifications");
        return;
    }
    ArmInterrupt(ivars);
    RNDLOG("interrupt IN reader %{public}s (%u-byte buffer)",
           ivars->interruptSlot.inFlight ? "armed" : "not armed", capacity);
}

// Reads the messages the device has announced with RESPONSE_AVAILABLE but no GET has
// taken yet (announced minus consumed), one per GET, and passes each to
// ServiceDeviceMessage. Stops when the count is settled, at kMaxDrainGets, at an empty
// answer, or at the first transport error. A completion cannot be waited for here (no
// command is in flight), so every completion that comes out is a stale one and is
// dropped.
//
// The count matters because the device queues each response before it notifies, and
// the notification's completion runs on this queue only after the command waiting for
// that response has already read it: a GET for it would find the queue empty, which
// some devices answer only with a timeout (500 ms of blocked queue each).
static void
DrainResponses(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    if (!ControlEndpointUsable(ivars)) {
        return;
    }
    for (uint32_t gets = 0;
         gets < kMaxDrainGets && ivars->responsesConsumed < ivars->responsesAnnounced;
         gets++) {
        uint16_t received = 0;
        RNDIS::Header header = {};
        RNDIS::ResponseKind kind = RNDIS::ResponseKind::NotReady;
        kern_return_t ret = ReceiveResponse(ivars, kDrainTimeoutMs, 0, 0, &header, &received, &kind);
        if (ret != kIOReturnSuccess) {
            if (!IsDeviceGoneStatus(ret)) {
                RNDLOG_COUNTED(ivars->interruptErrors,
                               "drain after notification: GET_ENCAPSULATED_RESPONSE failed: 0x%08x", ret);
            }
            ResyncResponseLedger(ivars);
            return;
        }
        if (kind == RNDIS::ResponseKind::NotReady) {
            ivars->emptyDrains++;
            ResyncResponseLedger(ivars); // the count was off
            return;
        }
        ivars->drainedMessages++;
        if (ServiceDeviceMessage(self, ivars, kind, &header, received) != kIOReturnSuccess) {
            return;
        }
    }
}

void
IMPL(RNDISaurusDriver, InterruptComplete)
{
    (void)action;
    (void)completionTimestamp;
    TransferSlot * slot = &ivars->interruptSlot;
    slot->inFlight = false;

    if (IsDeviceGoneStatus(status) || ivars->interruptPipe == nullptr || slot->bytes == nullptr) {
        return; // teardown in progress (our abort), or the device left
    }
    if (status != kIOReturnSuccess) {
        ivars->interruptConsecutiveErrors++;
        RNDLOG_COUNTED(ivars->interruptErrors, "interrupt IN completed with 0x%08x", status);
        RecordError(ivars, kRNDISaurusErrorStageInterruptIn, status);
        if (status == kUSBHostReturnPipeStalled) {
            kern_return_t ret = ivars->interruptPipe->ClearStall(true);
            if (ret != kIOReturnSuccess) {
                RNDLOG("interrupt IN: ClearStall failed: 0x%08x", ret);
            }
        }
        if (ivars->interruptConsecutiveErrors > kMaxInterruptErrors) {
            ivars->interruptParked = true;
            RNDLOG("interrupt IN reader parked after %u consecutive errors; responses are still polled",
                   ivars->interruptConsecutiveErrors);
            return;
        }
        ArmInterrupt(ivars);
        return;
    }

    ivars->interruptConsecutiveErrors = 0;
    ivars->notifications++;
    if (RNDIS::IsResponseAvailable(slot->bytes, actualByteCount)) {
        ivars->responsesAnnounced++;
        if (ivars->responsesConsumed < ivars->responsesAnnounced) {
            DrainResponses(this, ivars);
        } else {
            ivars->notificationsAlreadyServed++; // a command read it before we got here
        }
    } else {
        if (WithinLogLimit(++ivars->unknownNotifications)) {
            if (actualByteCount >= RNDIS::kResponseAvailableSize) {
                RNDLOG("unknown notification: %u bytes, first words 0x%08x 0x%08x", actualByteCount,
                       RNDIS::ReadLE32(slot->bytes, 0), RNDIS::ReadLE32(slot->bytes, 4));
            } else {
                RNDLOG("short notification: %u byte(s)", actualByteCount);
            }
        }
    }
    ArmInterrupt(ivars);
}

// MARK: - Bring-up state machine

static bool
ShouldLogAttempt(uint32_t attempt)
{
    return attempt <= kLoggedAttempts || attempt % kAttemptLogPeriod == 0;
}

// Arms the timer for the next attempt. Must run on the default queue, which every
// caller does.
static kern_return_t
ScheduleAttempt(RNDISaurusDriver_IVars * ivars, uint32_t delayMs)
{
    if (ivars->bringUpTimer == nullptr) {
        return kIOReturnNotReady;
    }
    const uint64_t now = clock_gettime_nsec_np(CLOCK_UPTIME_RAW);
    kern_return_t ret = ivars->bringUpTimer->WakeAtTime(kIOTimerClockUptimeRaw,
                                                        now + delayMs * kNanosecondsPerMillisecond, 0);
    if (ret != kIOReturnSuccess) {
        RNDLOG("scheduling the next bring-up attempt failed: 0x%08x", ret);
    }
    return ret;
}

// One attempt: RESET first when it is a retry, then the three bring-up commands, then
// the Ethernet interface. A failed attempt schedules the next one with a growing
// delay; only a failure to set up the interface after a successful bring-up gives up,
// because repeating that registration is not safe.
static void
RunBringUpAttempt(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    if (!ControlEndpointUsable(ivars)
        || ivars->bringUpState == kRNDISaurusBringUpUp || ivars->bringUpState == kRNDISaurusBringUpFailed) {
        return;
    }
    ivars->bringUpAttempts++;
    ivars->bringUpState = kRNDISaurusBringUpInProgress;
    const uint32_t attempt = ivars->bringUpAttempts;
    const bool verbose = ShouldLogAttempt(attempt);

    if (attempt > 1) {
        if (verbose) {
            RNDLOG("bring-up attempt %u: RESET first", attempt);
        }
        RNDISReset(self, ivars);
    } else {
        RNDLOG("bring-up attempt 1");
    }

    kern_return_t ret = BringUpRNDIS(self, ivars);
    if (ret == kIOReturnSuccess) {
        ret = SetUpNetworking(self, ivars);
        if (ret != kIOReturnSuccess) {
            TeardownNetworking(ivars);
            ivars->bringUpState = kRNDISaurusBringUpFailed;
            RecordError(ivars, kRNDISaurusErrorStageEthernetInterface, ret);
            RNDLOG("Ethernet interface setup failed: 0x%08x; giving up until the device is unplugged", ret);
            return;
        }
        ivars->linkUpAtNs   = RNDISaurusClockNow();
        ivars->bringUpState = kRNDISaurusBringUpUp;
        RNDLOG("RNDIS control plane is up after %u attempt(s)", attempt);
        return;
    }

    ivars->bringUpState = kRNDISaurusBringUpRetryScheduled;
    const uint32_t delayMs = ivars->nextRetryDelayMs;
    ivars->nextRetryDelayMs = delayMs * 2 < kMaxRetryDelayMs ? delayMs * 2 : kMaxRetryDelayMs;
    if (verbose) {
        RNDLOG("bring-up attempt %u failed: 0x%08x; retrying in %u ms", attempt, ret, delayMs);
    }
    ScheduleAttempt(ivars, delayMs);
}

void
IMPL(RNDISaurusDriver, BringUpTimerFired)
{
    (void)action;
    (void)time;
    RunBringUpAttempt(this, ivars);
}

kern_return_t
StartBringUp(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    SetUpInterrupt(self, ivars);

    kern_return_t ret = IOTimerDispatchSource::Create(ivars->defaultQueue, &ivars->bringUpTimer);
    if (ret != kIOReturnSuccess || ivars->bringUpTimer == nullptr) {
        RNDLOG("IOTimerDispatchSource::Create failed: 0x%08x", ret);
        return ret != kIOReturnSuccess ? ret : kIOReturnNoMemory;
    }
    ret = self->CreateActionBringUpTimerFired(0, &ivars->bringUpTimerAction);
    if (ret != kIOReturnSuccess || ivars->bringUpTimerAction == nullptr) {
        RNDLOG("CreateActionBringUpTimerFired failed: 0x%08x", ret);
        return ret != kIOReturnSuccess ? ret : kIOReturnNoMemory;
    }
    ret = ivars->bringUpTimer->SetHandler(ivars->bringUpTimerAction);
    if (ret != kIOReturnSuccess) {
        RNDLOG("SetHandler on the bring-up timer failed: 0x%08x", ret);
        return ret;
    }

    ivars->bringUpState     = kRNDISaurusBringUpNotStarted;
    ivars->bringUpAttempts  = 0;
    ivars->nextRetryDelayMs = kFirstRetryDelayMs;
    ivars->mediaConnected   = true;
    return ScheduleAttempt(ivars, 0);
}

void
TeardownControlPlane(RNDISaurusDriver_IVars * ivars)
{
    CancelAndRelease(&ivars->bringUpTimer);
    CancelAndRelease(&ivars->bringUpTimerAction);

    // The in-flight read completes with kIOReturnAborted; the slot's action is then
    // cancelled and its buffer released once that completion has run.
    if (ivars->interruptPipe != nullptr && ivars->interruptSlot.inFlight) {
        ivars->interruptPipe->Abort(kIOUSBAbortAsynchronous, kIOReturnAborted, nullptr);
    }
    ReleaseSlot(&ivars->interruptSlot);

    // Two lines: unified logging cuts a dext message at roughly 250 characters.
    if (ivars->bringUpAttempts > 0) {
        RNDLOG("control plane torn down: %u bring-up attempt(s), %u reset(s); %llu notification(s) "
               "(%llu unknown), %llu already served by a command, %llu interrupt error(s)",
               ivars->bringUpAttempts, ivars->resets, ivars->notifications, ivars->unknownNotifications,
               ivars->notificationsAlreadyServed, ivars->interruptErrors);
        RNDLOG("unsolicited messages: %llu drained, %llu drain(s) empty, %llu dropped, "
               "%llu INDICATE_STATUS, %llu KEEPALIVE answered",
               ivars->drainedMessages, ivars->emptyDrains, ivars->droppedMessages,
               ivars->indicateStatusCount, ivars->keepalivesAnswered);
        ivars->bringUpAttempts = 0;
    }
}
