//
//  RNDISaurusDataPath.cpp
//  RNDISaurus — USB tethering driver for RNDIS devices on macOS
//
//  Copyright (C) 2026 N4H4
//  SPDX-License-Identifier: MPL-2.0
//
//  This Source Code Form is subject to the terms of the Mozilla Public License,
//  v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
//  one at https://mozilla.org/MPL/2.0/.
//
//  Packet pool and queue setup, interface registration, the enable/disable sequence
//  and the packet hand-off to the network stack follow Apple's
//  NetworkingDriverKitSample (Copyright © 2025 Apple Inc., MIT; notice in NOTICE.md).
//  Behaviour informed by jwise/HoRNDIS (transmit buffer reuse, clearing a stalled
//  pipe before resubmitting) and by the Linux USB network core (halt recovery on the
//  bulk pipes).
//

// The Ethernet side of the driver: the IOUserNetworkEthernet queues, the bulk IN
// receive path (RNDIS packet batches → frames to the stack) and the bulk OUT transmit
// path (frames from the stack → one RNDIS packet message per transfer).
//
// Everything here runs on the driver's default dispatch queue: the four packet
// queues were created on it, and the completion actions have no other queue. The
// bring-up attempt that registers the interface, the enable/disable calls, the
// transmit wake-up and both bulk completions therefore never run concurrently, which
// is why no state below is locked.

#include <string.h>

#include "RNDISaurusDriverPrivate.hpp"

// MARK: - Constants

static const char * kPoolName = "RNDISaurus";

// Callbacks whose first invocation is logged and the rest ignored (loggedCallbacks bits).
enum : uint32_t {
    kLoggedPromiscuous     = 1u << 0,
    kLoggedMulticast       = 1u << 1,
    kLoggedAllMulticast    = 1u << 2,
    kLoggedWakeOnMagic     = 1u << 3,
    kLoggedHardwareAssists = 1u << 4,
};

// Media reported to the stack. RNDIS has no media negotiation; the table is the one
// the network stack is known to accept, and the active entry is chosen from the USB
// connection speed.
static const uint32_t kMediaTable[] = {
    kIOUserNetworkMediaEthernetAuto,
    kIOUserNetworkMediaEthernet100BaseTX,
    kIOUserNetworkMediaEthernet1000BaseT,
};

// MARK: - Helpers

// True once for each `bit`, so a callback can be logged the first time it arrives.
static bool
FirstTime(RNDISaurusDriver_IVars * ivars, uint32_t bit)
{
    if (ivars->loggedCallbacks & bit) {
        return false;
    }
    ivars->loggedCallbacks |= bit;
    return true;
}

// Logs the first arrival of a callback (one loggedCallbacks bit each) and ignores the rest.
#define RNDLOG_ONCE(ivars, bit, fmt, ...)                                          \
    do {                                                                           \
        if (FirstTime((ivars), (bit))) {                                           \
            RNDLOG(fmt, ##__VA_ARGS__);                                            \
        }                                                                          \
    } while (0)

static uint32_t
SlotIndex(OSAction * action)
{
    const uint32_t * reference = static_cast<const uint32_t *>(action->GetReference());
    return reference != nullptr ? *reference : UINT32_MAX;
}

// Takes one packet from `queue`, or returns NULL when it has none. Packets stay owned
// by the pool: they go back to the stack with EnqueuePacket or to the pool with
// deallocatePacket, and are never released by the driver — hence the annotation,
// which tells the static analyzer not to expect a release.
static IOUserNetworkPacket *
DequeueOne(IOUserNetworkPacketQueue * queue) __attribute__((os_returns_not_retained));

static IOUserNetworkPacket *
DequeueOne(IOUserNetworkPacketQueue * queue)
{
    IOUserNetworkPacket * packets[1] = {};
    if (queue->DequeuePackets(packets, 1) != 1) {
        return nullptr;
    }
    return packets[0];
}

// MARK: - Transfer slots

static const char *
SlotKindName(SlotKind kind)
{
    switch (kind) {
        case SlotKind::BulkIn:    return "bulk IN";
        case SlotKind::BulkOut:   return "bulk OUT";
        case SlotKind::Interrupt: return "interrupt IN";
    }
    return "unknown";
}

kern_return_t
PrepareSlot(RNDISaurusDriver * self, TransferSlot * slot, uint32_t index, SlotKind kind,
            uint32_t capacity)
{
    const char *  name      = SlotKindName(kind);
    const uint64_t direction = kind == SlotKind::BulkOut ? kIOMemoryDirectionOut : kIOMemoryDirectionIn;
    kern_return_t ret       = kIOReturnBadArgument;
    switch (kind) {
        case SlotKind::BulkIn:    ret = self->CreateActionBulkInComplete(sizeof(uint32_t), &slot->action);    break;
        case SlotKind::BulkOut:   ret = self->CreateActionBulkOutComplete(sizeof(uint32_t), &slot->action);   break;
        case SlotKind::Interrupt: ret = self->CreateActionInterruptComplete(sizeof(uint32_t), &slot->action); break;
    }
    if (ret != kIOReturnSuccess || slot->action == nullptr) {
        RNDLOG("%{public}s slot %u: creating the completion action failed: 0x%08x", name, index, ret);
        return ret != kIOReturnSuccess ? ret : kIOReturnNoMemory;
    }
    uint32_t * reference = static_cast<uint32_t *>(slot->action->GetReference());
    if (reference == nullptr) {
        RNDLOG("%{public}s slot %u: action has no reference storage", name, index);
        return kIOReturnInternalError;
    }
    *reference = index;

    // Receive buffers keep their full length; transmit buffers are resized per submission.
    ret = CreateMappedBuffer(direction, capacity, name, &slot->buffer, &slot->bytes);
    if (ret != kIOReturnSuccess) {
        RNDLOG("%{public}s slot %u: buffer setup failed: 0x%08x", name, index, ret);
    }
    return ret;
}

// The buffer goes with the action: a completion still draining may read it.
void
ReleaseSlot(TransferSlot * slot)
{
    IOBufferMemoryDescriptor * buffer = slot->buffer;
    slot->buffer   = nullptr;
    slot->bytes    = nullptr;
    slot->inFlight = false;
    CancelAndRelease(&slot->action, buffer);
}

// MARK: - Bulk pipe halt recovery

// A bulk endpoint that fails a transfer for any reason other than a timeout (and our
// transfers have none) is halted until ClearStall is called on it. ClearStall also
// aborts the pipe's other in-flight transfers, whose completions then arrive with
// kIOReturnAborted and re-arm through the normal path. Recoveries are counted per pipe
// and reset by a successful transfer; a pipe that keeps failing is parked and the link
// reported down, until the interface is re-enabled.

enum class Recovery : uint8_t {
    Cleared, // the halt is cleared (or already was): the pipe can be used again
    Failed,  // ClearStall failed; the pipe is still halted and the attempt was counted
    Parked,  // recovery has given up on this pipe until the interface is re-enabled
    Gone,    // the device left; Stop follows
};

static PipeRecovery *
RecoveryFor(RNDISaurusDriver_IVars * ivars, bool receive)
{
    return receive ? &ivars->rxRecovery : &ivars->txRecovery;
}

// Leaves the pipe idle and tells the stack the link is down so that traffic fails
// over instead of disappearing into a dead interface.
static void
ParkPipe(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars, bool receive, const char * why)
{
    PipeRecovery * state = RecoveryFor(ivars, receive);
    if (state->parked) {
        return;
    }
    state->parked = true;
    RNDLOG("bulk %{public}s parked (%{public}s, %u recover(ies) in a row); "
           "link reported down until the interface is re-enabled",
           receive ? "IN" : "OUT", why, state->consecutiveRecoveries);
    RecordError(ivars, receive ? kRNDISaurusErrorStageBulkInParked : kRNDISaurusErrorStageBulkOutParked,
                kIOReturnNotResponding);
    UpdateLinkStatus(self, ivars);
}

// The way out of a parked pipe, for both pipes at once: re-enabling the interface
// starts the recovery count afresh. State only; the caller reports the link.
static void
UnparkPipes(RNDISaurusDriver_IVars * ivars)
{
    ivars->rxRecovery.parked = false;
    ivars->txRecovery.parked = false;
    ivars->rxRecovery.consecutiveRecoveries = 0;
    ivars->txRecovery.consecutiveRecoveries = 0;
}

// Bookkeeping and ClearStall for a transfer that completed, or could not be submitted,
// with an error that is neither device-gone nor one of our own aborts. Never arms or
// submits anything itself; the caller acts on the answer.
//
// `slotGeneration` is the pipe's halt generation when the failed transfer was
// submitted: if a ClearStall has succeeded since, this error is the same halt seen by
// another slot and needs no second clear. Submission failures pass the pipe's current
// generation, because they are fresh evidence of a halt.
static Recovery
RecoverPipe(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars, bool receive,
            uint32_t slotGeneration, kern_return_t status)
{
    IOUSBHostPipe * pipe  = receive ? ivars->bulkInPipe : ivars->bulkOutPipe;
    PipeRecovery *  state = RecoveryFor(ivars, receive);
    const char *    name  = receive ? "IN" : "OUT";

    if (pipe == nullptr || state->parked) {
        return Recovery::Parked;
    }
    if (slotGeneration < state->haltGeneration) {
        return Recovery::Cleared; // a later ClearStall already dealt with this halt
    }

    state->consecutiveRecoveries++;
    state->recoveries++;
    if (state->consecutiveRecoveries > kMaxPipeRecoveries) {
        ParkPipe(self, ivars, receive, "too many consecutive halt recoveries");
        return Recovery::Parked;
    }

    kern_return_t ret = pipe->ClearStall(true);
    if (ret != kIOReturnSuccess) {
        if (IsDeviceLeavingStatus(ret)) {
            return Recovery::Gone;
        }
        if (WithinLogLimit(state->recoveries)) {
            RNDLOG("bulk %{public}s halted (0x%08x); ClearStall(true) failed: 0x%08x (recovery %u)",
                   name, status, ret, state->consecutiveRecoveries);
        }
        return Recovery::Failed; // the halt stands, so the generation does not move
    }
    state->haltGeneration++;
    if (WithinLogLimit(state->recoveries)) {
        RNDLOG("bulk %{public}s halted (0x%08x); ClearStall(true) ok (recovery %u)",
               name, status, state->consecutiveRecoveries);
    }
    return Recovery::Cleared;
}

// MARK: - Receive path

static uint32_t
ReceivesInFlight(const RNDISaurusDriver_IVars * ivars)
{
    uint32_t count = 0;
    for (uint32_t i = 0; i < kRxTransfers; i++) {
        count += ivars->rxSlots[i].inFlight ? 1 : 0;
    }
    return count;
}

// Submits the slot's receive transfer. A submission the pipe refuses for any reason
// other than the device leaving means the pipe is halted: it is cleared and the
// submission retried once. If the slot still cannot be armed and no other receive is
// in flight to drive another attempt, the pipe is parked.
static void
ArmReceiveSlot(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars, uint32_t index)
{
    TransferSlot * slot = &ivars->rxSlots[index];
    if (slot->inFlight || slot->buffer == nullptr || ivars->bulkInPipe == nullptr
        || ivars->rxRecovery.parked) {
        return;
    }
    kern_return_t ret = ivars->bulkInPipe->AsyncIO(slot->buffer, kRxTransferSize, slot->action, 0);
    if (ret != kIOReturnSuccess) {
        if (IsDeviceLeavingStatus(ret)) {
            return; // unplugged between a completion and its resubmission: Stop follows
        }
        RNDLOG_COUNTED(ivars->rxTransferErrors, "bulk IN slot %u: AsyncIO failed: 0x%08x", index, ret);
        RecordError(ivars, kRNDISaurusErrorStageBulkIn, ret);
        Recovery recovery = RecoverPipe(self, ivars, true, ivars->rxRecovery.haltGeneration, ret);
        if (recovery == Recovery::Cleared) {
            ret = ivars->bulkInPipe->AsyncIO(slot->buffer, kRxTransferSize, slot->action, 0);
        }
        if (ret != kIOReturnSuccess) {
            if (recovery == Recovery::Gone || IsDeviceLeavingStatus(ret)) {
                return;
            }
            if (recovery != Recovery::Parked && ReceivesInFlight(ivars) == 0) {
                ParkPipe(self, ivars, true, "the receive transfer cannot be armed");
            }
            return;
        }
    }
    slot->generation = ivars->rxRecovery.haltGeneration;
    slot->inFlight   = true;
}

static void
ArmAllReceives(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    for (uint32_t i = 0; i < kRxTransfers; i++) {
        ArmReceiveSlot(self, ivars, i);
    }
}

// Hands one Ethernet frame to the stack in a packet taken from the receive
// submission queue; drops it (counted) when the stack has none to give.
static void
DeliverFrame(RNDISaurusDriver_IVars * ivars, const uint8_t * frame, uint32_t frameLength)
{
    IOUserNetworkPacket * packet = DequeueOne(ivars->rxSubmission);
    if (packet == nullptr) {
        ivars->rxDropsNoBuffer++;
        return;
    }
    const uint16_t dataOffset = packet->getDataOffset();
    if (frameLength > kPacketBufferSize - dataOffset) {
        RNDLOG_COUNTED(ivars->rxDropsOversize, "dropping %u-byte frame: larger than a %u-byte packet buffer",
                       frameLength, kPacketBufferSize - dataOffset);
        ivars->pool->deallocatePacket(packet);
        return;
    }
    uint8_t * destination = reinterpret_cast<uint8_t *>(packet->getDataVirtualAddress()) + dataOffset;
    memcpy(destination, frame, frameLength);

    bool ok = packet->setDataOffset(dataOffset) == kIOReturnSuccess
           && packet->setLinkHeaderLength(0) == kIOReturnSuccess
           && packet->setDataLength(frameLength) == kIOReturnSuccess;
    if (!ok || ivars->rxCompletion->EnqueuePacket(packet) != kIOReturnSuccess) {
        ivars->rxDropsNoBuffer++;
        ivars->pool->deallocatePacket(packet);
        return;
    }
    ivars->rxPackets++;
    ivars->rxBytes += frameLength;
}

// Walks the RNDIS packet messages batched in one bulk IN transfer.
static void
ReceiveBatch(RNDISaurusDriver_IVars * ivars, const uint8_t * bytes, uint32_t length)
{
    size_t cursor = 0;
    uint32_t messages = 0;
    ivars->rxTransfers++;
    while (true) {
        const uint8_t * frame = nullptr;
        uint32_t frameLength = 0;
        RNDIS::BatchStep step = RNDIS::NextPacket(bytes, length, &cursor, &frame, &frameLength);
        if (step == RNDIS::BatchStep::End) {
            break;
        }
        if (step == RNDIS::BatchStep::Malformed) {
            RNDLOG_COUNTED(ivars->rxDropsMalformed,
                           "malformed RNDIS packet message at offset %u of a %u-byte transfer; rest dropped",
                           static_cast<unsigned>(cursor), length);
            break;
        }
        messages++;
        if (frameLength < ETHER_HDR_LEN) {
            ivars->rxDropsMalformed++; // a runt cannot even carry an Ethernet header
            continue;
        }
        DeliverFrame(ivars, frame, frameLength);
    }
    if (messages > 1) {
        ivars->rxBatchedTransfers++;
    }
}

void
IMPL(RNDISaurusDriver, BulkInComplete)
{
    (void)completionTimestamp;
    const uint32_t index = SlotIndex(action);
    if (index >= kRxTransfers) {
        RNDLOG("bulk IN completion with invalid slot index %u", index);
        return;
    }
    TransferSlot * slot = &ivars->rxSlots[index];
    slot->inFlight = false;

    if (status == kIOReturnAborted) {
        // Aborted by SetInterfaceEnable(false), by teardown, or by a ClearStall on this
        // pipe. If the interface is (still, or again) enabled the slot is needed back,
        // and so is any sibling that a failed arm left idle.
        if (ivars->interfaceEnabled) {
            ArmAllReceives(this, ivars);
        }
        return;
    }
    if (IsDeviceGoneStatus(status) || ivars->bulkInPipe == nullptr || slot->bytes == nullptr) {
        return; // the device left: nothing to resubmit
    }
    if (status != kIOReturnSuccess) {
        // Any other error leaves the endpoint halted: clear it and re-arm, bounded.
        // After a failed clear the arm below runs the ladder again from the submission
        // side, which parks the pipe if it cannot be used at all.
        RNDLOG_COUNTED(ivars->rxTransferErrors, "bulk IN slot %u completed with 0x%08x (%u bytes)",
                       index, status, actualByteCount);
        RecordError(ivars, kRNDISaurusErrorStageBulkIn, status);
        Recovery recovery = RecoverPipe(this, ivars, true, slot->generation, status);
        if ((recovery == Recovery::Cleared || recovery == Recovery::Failed) && ivars->interfaceEnabled) {
            ArmAllReceives(this, ivars);
        }
        return;
    }
    ivars->rxRecovery.consecutiveRecoveries = 0;
    if (ivars->interfaceEnabled) {
        ReceiveBatch(ivars, slot->bytes, actualByteCount);
        ArmReceiveSlot(this, ivars, index);
    }
}

// MARK: - Transmit path

// Gives a frame back to the stack once its bytes have been copied out (or dropped).
static void
ReturnTxPacket(RNDISaurusDriver_IVars * ivars, IOUserNetworkPacket * packet)
{
    if (ivars->txCompletion->EnqueuePacket(packet) != kIOReturnSuccess) {
        ivars->pool->deallocatePacket(packet);
    }
}

// Largest Ethernet frame one bulk OUT transfer can carry: what the transfer buffer and
// the device's MaxTransferSize allow, and never more than an Ethernet frame.
static uint32_t
MaxTxFrameLength(const RNDISaurusDriver_IVars * ivars)
{
    const uint32_t limit = RNDIS::MaxFrameLength(kTxTransferSize, ivars->deviceMaxTransferSize);
    return limit < ETHER_MAX_LEN ? limit : ETHER_MAX_LEN;
}

// While the pipe is parked nobody will take the stack's frames: they are dropped and
// counted so that the submission queue keeps moving.
static void
DropParkedTx(RNDISaurusDriver_IVars * ivars)
{
    IOUserNetworkPacket * packet = nullptr;
    while ((packet = DequeueOne(ivars->txSubmission)) != nullptr) {
        ivars->txDropsParked++;
        ReturnTxPacket(ivars, packet);
    }
}

// Moves frames from the transmit submission queue into idle transfer slots, one RNDIS
// packet message per bulk OUT transfer, until either runs out. Each frame is returned
// to the stack as soon as it has been copied. A submission the pipe refuses means the
// pipe is halted: it is cleared and the submission retried once; a frame that still
// cannot go out is lost (counted), and the recovery bound eventually parks the pipe.
static void
DrainTx(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    if (!ivars->interfaceEnabled || ivars->bulkOutPipe == nullptr) {
        return;
    }
    const uint32_t maxFrame = MaxTxFrameLength(ivars);
    while (ivars->txFreeCount > 0 && !ivars->txRecovery.parked) {
        IOUserNetworkPacket * packet = DequeueOne(ivars->txSubmission);
        if (packet == nullptr) {
            return;
        }
        const uint32_t frameLength = packet->getDataLength();
        const uint8_t * frame =
            reinterpret_cast<const uint8_t *>(packet->getDataVirtualAddress()) + packet->getDataOffset();
        if (frameLength == 0 || frameLength > maxFrame) {
            RNDLOG_COUNTED(ivars->txDropsOversize, "dropping %u-byte outbound frame: the device accepts at most %u",
                           frameLength, maxFrame);
            ReturnTxPacket(ivars, packet);
            continue;
        }

        const uint32_t index = ivars->txFreeSlots[ivars->txFreeCount - 1];
        TransferSlot * slot = &ivars->txSlots[index];
        bool padded = false;
        const uint32_t total = static_cast<uint32_t>(RNDIS::FramePacket(
            slot->bytes, kTxTransferSize, frame, frameLength, ivars->bulkOutMaxPacketSize, &padded));
        ReturnTxPacket(ivars, packet);
        if (total == 0) {
            RNDLOG_COUNTED(ivars->txDropsOversize, "dropping %u-byte outbound frame: it does not fit a transfer",
                           frameLength);
            continue;
        }
        if (padded) {
            ivars->txPads++;
        }

        kern_return_t ret = slot->buffer->SetLength(total);
        if (ret != kIOReturnSuccess) {
            RNDLOG_COUNTED(ivars->txErrors, "bulk OUT slot %u: SetLength(%u) failed: 0x%08x", index, total, ret);
            RecordError(ivars, kRNDISaurusErrorStageBulkOut, ret);
            continue; // the slot is still free
        }
        ret = ivars->bulkOutPipe->AsyncIO(slot->buffer, total, slot->action, 0);
        if (ret != kIOReturnSuccess) {
            if (IsDeviceLeavingStatus(ret)) {
                return; // unplugged: Stop follows
            }
            RNDLOG_COUNTED(ivars->txErrors, "bulk OUT slot %u: submitting %u bytes failed: 0x%08x",
                           index, total, ret);
            RecordError(ivars, kRNDISaurusErrorStageBulkOut, ret);
            Recovery recovery = RecoverPipe(self, ivars, false, ivars->txRecovery.haltGeneration, ret);
            if (recovery == Recovery::Gone) {
                return;
            }
            if (recovery == Recovery::Cleared) {
                ret = ivars->bulkOutPipe->AsyncIO(slot->buffer, total, slot->action, 0);
            }
            if (ret != kIOReturnSuccess) {
                continue; // the frame is lost; the slot is still free
            }
        }
        slot->generation = ivars->txRecovery.haltGeneration;
        slot->inFlight   = true;
        ivars->txFreeCount--;
        ivars->txPackets++;
        ivars->txBytes += frameLength;
    }
    if (ivars->txRecovery.parked) {
        DropParkedTx(ivars);
    }
}

void
IMPL(RNDISaurusDriver, TxPacketAvailable)
{
    (void)action;
    DrainTx(this, ivars);
}

void
IMPL(RNDISaurusDriver, BulkOutComplete)
{
    (void)completionTimestamp;
    (void)actualByteCount;
    const uint32_t index = SlotIndex(action);
    if (index >= kTxTransfers) {
        RNDLOG("bulk OUT completion with invalid slot index %u", index);
        return;
    }
    TransferSlot * slot = &ivars->txSlots[index];
    if (ivars->bulkOutPipe == nullptr || slot->bytes == nullptr) {
        return; // teardown has already released the slot
    }
    if (slot->inFlight) {
        slot->inFlight = false;
        ivars->txFreeSlots[ivars->txFreeCount++] = index;
    }

    if (status == kIOReturnAborted) {
        // An abort from SetInterfaceEnable(false) or teardown carries the current halt
        // generation; an older one means a ClearStall on this pipe cut the transfer
        // short, and its frame (already handed back to the stack) is lost.
        if (slot->generation < ivars->txRecovery.haltGeneration) {
            ivars->txDropsAborted++;
        }
    } else if (IsDeviceGoneStatus(status)) {
        return; // the device left
    } else if (status != kIOReturnSuccess) {
        // The endpoint is halted: clear it (bounded), then let the drain below carry on
        // with whatever slots are free.
        RNDLOG_COUNTED(ivars->txErrors, "bulk OUT slot %u completed with 0x%08x", index, status);
        RecordError(ivars, kRNDISaurusErrorStageBulkOut, status);
        if (RecoverPipe(this, ivars, false, slot->generation, status) == Recovery::Gone) {
            return;
        }
    } else {
        ivars->txRecovery.consecutiveRecoveries = 0;
    }
    DrainTx(this, ivars); // frames may have queued up while every slot was busy or the interface was down
}

// MARK: - Interface enable / disable

// Enabling stops at the first failure and reports it. Disabling is best effort: a
// queue that is already disabled answers kIOReturnBadArgument, which is not news.
static kern_return_t
SetQueuesEnabled(RNDISaurusDriver_IVars * ivars, bool enable)
{
    IOUserNetworkPacketQueue * queues[] = {
        ivars->txCompletion, ivars->txSubmission, ivars->rxCompletion, ivars->rxSubmission,
    };
    for (IOUserNetworkPacketQueue * queue : queues) {
        if (queue == nullptr) {
            continue;
        }
        kern_return_t ret = queue->SetEnable(enable);
        if (ret != kIOReturnSuccess && enable) {
            RNDLOG("SetEnable(true) on a packet queue failed: 0x%08x", ret);
            return ret;
        }
    }
    return kIOReturnSuccess;
}

// Aborts the transfers in flight; their completions arrive with kIOReturnAborted.
// Asynchronous on purpose: the completions run on this very queue.
static void
AbortBulkPipes(RNDISaurusDriver_IVars * ivars)
{
    if (ivars->bulkInPipe != nullptr) {
        ivars->bulkInPipe->Abort(kIOUSBAbortAsynchronous, kIOReturnAborted, nullptr);
    }
    if (ivars->bulkOutPipe != nullptr) {
        ivars->bulkOutPipe->Abort(kIOUSBAbortAsynchronous, kIOReturnAborted, nullptr);
    }
}

// Stops the data flow: no new transfers, in-flight ones aborted, queues disabled.
// On unplug the stack has usually disabled the interface before Stop runs, in which
// case there is nothing left to do here.
static void
StopDataFlow(RNDISaurusDriver_IVars * ivars)
{
    if (!ivars->interfaceEnabled) {
        return;
    }
    ivars->interfaceEnabled = false;
    AbortBulkPipes(ivars);
    SetQueuesEnabled(ivars, false);
}

kern_return_t
UpdateLinkStatus(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    if (!ivars->interfaceRegistered) {
        return kIOReturnSuccess;
    }
    const bool active = ivars->interfaceEnabled && ivars->mediaConnected
                     && !ivars->rxRecovery.parked && !ivars->txRecovery.parked;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    kern_return_t ret = self->ReportLinkStatus(
        active ? kIOUserNetworkLinkStatusActive : kIOUserNetworkLinkStatusInactive,
        ivars->activeMediaType);
#pragma clang diagnostic pop
    if (ret != kIOReturnSuccess) {
        RNDLOG("ReportLinkStatus(%{public}s) failed: 0x%08x", active ? "active" : "inactive", ret);
        return ret;
    }
    if (active != ivars->linkReportedActive) {
        ivars->linkReportedActive = active;
        RNDLOG("link %{public}s (media 0x%08x%{public}s%{public}s%{public}s)", active ? "up" : "down",
               ivars->activeMediaType,
               ivars->mediaConnected ? "" : ", device reports media disconnected",
               ivars->rxRecovery.parked ? ", bulk IN parked" : "",
               ivars->txRecovery.parked ? ", bulk OUT parked" : "");
    }
    return kIOReturnSuccess;
}

// Starts the data flow: queues on, parked pipes released, receives armed, link
// reported, and whatever the stack already queued for transmit sent.
static kern_return_t
EnableInterface(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    kern_return_t ret = SetQueuesEnabled(ivars, true);
    if (ret != kIOReturnSuccess) {
        return ret;
    }
    ivars->interfaceEnabled = true;
    UnparkPipes(ivars);
    ArmAllReceives(self, ivars);
    ret = UpdateLinkStatus(self, ivars);
    if (ret != kIOReturnSuccess) {
        return ret;
    }
    RNDLOG("interface enabled: %u of %u bulk IN transfer(s) in flight", ReceivesInFlight(ivars), kRxTransfers);
    DrainTx(self, ivars);
    return kIOReturnSuccess;
}

static void
DisableInterface(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    StopDataFlow(ivars);
    UpdateLinkStatus(self, ivars);
}

kern_return_t
IMPL(RNDISaurusDriver, SetInterfaceEnable)
{
    if (isEnable) {
        kern_return_t ret = EnableInterface(this, ivars);
        if (ret != kIOReturnSuccess) {
            RNDLOG("enabling the interface failed: 0x%08x; disabling", ret);
            DisableInterface(this, ivars);
        }
        return ret;
    }
    DisableInterface(this, ivars);
    RNDLOG("interface disabled");
    return kIOReturnSuccess;
}

// MARK: - Interface settings the stack pushes down

kern_return_t
IMPL(RNDISaurusDriver, SetPromiscuousModeEnable)
{
    RNDLOG_ONCE(ivars, kLoggedPromiscuous,
                "SetPromiscuousModeEnable(%{public}s): the RNDIS packet filter already passes everything",
                enable ? "true" : "false");
    return kIOReturnSuccess;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
kern_return_t
IMPL(RNDISaurusDriver, SetMulticastAddresses)
{
    (void)addresses;
    RNDLOG_ONCE(ivars, kLoggedMulticast,
                "SetMulticastAddresses(%u): RNDIS devices ignore multicast lists; all multicast is passed", count);
    return kIOReturnSuccess;
}
#pragma clang diagnostic pop

kern_return_t
IMPL(RNDISaurusDriver, SetAllMulticastModeEnable)
{
    RNDLOG_ONCE(ivars, kLoggedAllMulticast,
                "SetAllMulticastModeEnable(%{public}s): already the packet filter's behavior",
                enable ? "true" : "false");
    return kIOReturnSuccess;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
kern_return_t
IMPL(RNDISaurusDriver, SelectMediaType)
{
    // The link speed is fixed by the USB connection; a manual selection changes
    // nothing on the wire, so the same active media is reported back.
    RNDLOG("SelectMediaType(0x%08x): media is fixed by the USB connection, reporting 0x%08x",
           mediaType, ivars->activeMediaType);
    return UpdateLinkStatus(this, ivars);
}
#pragma clang diagnostic pop

kern_return_t
IMPL(RNDISaurusDriver, SetWakeOnMagicPacketEnable)
{
    RNDLOG_ONCE(ivars, kLoggedWakeOnMagic,
                "SetWakeOnMagicPacketEnable(%{public}s): not supported by the device; ignored",
                enable ? "true" : "false");
    return kIOReturnSuccess;
}

kern_return_t
IMPL(RNDISaurusDriver, SetMTU)
{
    if (mtu > kMTU) {
        RNDLOG("SetMTU(%u) refused: the interface carries standard Ethernet frames (MTU %u)", mtu, kMTU);
        return kIOReturnUnsupported;
    }
    RNDLOG("SetMTU(%u)", mtu);
    return kIOReturnSuccess;
}

kern_return_t
IMPL(RNDISaurusDriver, GetMaxTransferUnit)
{
    *mtu = kMTU;
    return kIOReturnSuccess;
}

kern_return_t
IMPL(RNDISaurusDriver, SetHardwareAssists)
{
    RNDLOG_ONCE(ivars, kLoggedHardwareAssists,
                "SetHardwareAssists(0x%08x): no hardware assists are offered", hardwareAssists);
    return kIOReturnSuccess;
}

kern_return_t
IMPL(RNDISaurusDriver, GetHardwareAssists)
{
    *hardwareAssists = 0;
    return kIOReturnSuccess;
}

// MARK: - Setup and teardown

// Creates the transfer slots with their completion actions.
static kern_return_t
PrepareSlots(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    for (uint32_t i = 0; i < kRxTransfers; i++) {
        kern_return_t ret = PrepareSlot(self, &ivars->rxSlots[i], i, SlotKind::BulkIn, kRxTransferSize);
        if (ret != kIOReturnSuccess) {
            return ret;
        }
    }
    ivars->txFreeCount = 0;
    for (uint32_t i = 0; i < kTxTransfers; i++) {
        kern_return_t ret = PrepareSlot(self, &ivars->txSlots[i], i, SlotKind::BulkOut, kTxTransferSize);
        if (ret != kIOReturnSuccess) {
            return ret;
        }
        ivars->txFreeSlots[ivars->txFreeCount++] = i;
    }
    return kIOReturnSuccess;
}

// Creates the pool and the four queues, all on the default dispatch queue, and hooks
// the transmit submission queue's data-available notification to TxPacketAvailable.
static kern_return_t
CreateQueues(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    if (ivars->defaultQueue == nullptr) {
        RNDLOG("no dispatch queue for the packet queues");
        return kIOReturnNotReady;
    }

    IOUserNetworkPacketBufferPoolOptions options = {};
    options.packetCount         = kPacketPoolSize;
    options.bufferCount         = kPacketPoolSize;
    options.bufferSize          = kPacketBufferSize;
    options.maxBuffersPerPacket = 1;
    options.memorySegmentSize   = 0;
    options.poolFlags           = PoolFlagMapToDext;
    options.dmaSpecification.maxAddressBits = 64;
    kern_return_t ret = IOUserNetworkPacketBufferPool::CreateWithOptions(self, kPoolName, &options, &ivars->pool);
    if (ret != kIOReturnSuccess) {
        RNDLOG("IOUserNetworkPacketBufferPool::CreateWithOptions failed: 0x%08x", ret);
        return ret;
    }

    ret = self->CreateActionTxPacketAvailable(0, &ivars->txPacketAction);
    if (ret != kIOReturnSuccess) {
        RNDLOG("CreateActionTxPacketAvailable failed: 0x%08x", ret);
        return ret;
    }
    ret = IOUserNetworkTxSubmissionQueue::Create(ivars->pool, self, kPacketQueueCapacity, 0,
                                                 ivars->defaultQueue, &ivars->txSubmission);
    if (ret != kIOReturnSuccess) {
        RNDLOG("IOUserNetworkTxSubmissionQueue::Create failed: 0x%08x", ret);
        return ret;
    }
    IODataQueueDispatchSource * dataQueue = nullptr;
    ret = ivars->txSubmission->CopyDataQueue(&dataQueue);
    if (ret == kIOReturnSuccess && dataQueue != nullptr) {
        ret = dataQueue->SetDataAvailableHandler(ivars->txPacketAction);
    }
    OSSafeReleaseNULL(dataQueue);
    if (ret != kIOReturnSuccess) {
        RNDLOG("hooking TxPacketAvailable to the transmit queue failed: 0x%08x", ret);
        return ret;
    }

    ret = IOUserNetworkTxCompletionQueue::Create(ivars->pool, self, kPacketQueueCapacity, 0,
                                                 ivars->defaultQueue, &ivars->txCompletion);
    if (ret != kIOReturnSuccess) {
        RNDLOG("IOUserNetworkTxCompletionQueue::Create failed: 0x%08x", ret);
        return ret;
    }
    ret = IOUserNetworkRxSubmissionQueue::Create(ivars->pool, self, kPacketQueueCapacity, 0,
                                                 ivars->defaultQueue, &ivars->rxSubmission);
    if (ret != kIOReturnSuccess) {
        RNDLOG("IOUserNetworkRxSubmissionQueue::Create failed: 0x%08x", ret);
        return ret;
    }
    ret = IOUserNetworkRxCompletionQueue::Create(ivars->pool, self, kPacketQueueCapacity, 0,
                                                 ivars->defaultQueue, &ivars->rxCompletion);
    if (ret != kIOReturnSuccess) {
        RNDLOG("IOUserNetworkRxCompletionQueue::Create failed: 0x%08x", ret);
        return ret;
    }
    return kIOReturnSuccess;
}

kern_return_t
SetUpNetworking(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars)
{
    kern_return_t ret = CreateQueues(self, ivars);
    if (ret != kIOReturnSuccess) {
        return ret;
    }
    ret = PrepareSlots(self, ivars);
    if (ret != kIOReturnSuccess) {
        return ret;
    }

    ivars->activeMediaType = ivars->deviceSpeed >= kIOUSBHostConnectionSpeedSuper
                                 ? kIOUserNetworkMediaEthernet1000BaseT
                                 : kIOUserNetworkMediaEthernet100BaseTX;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    ret = self->ReportAvailableMediaTypes(kMediaTable, sizeof(kMediaTable) / sizeof(kMediaTable[0]));
#pragma clang diagnostic pop
    if (ret != kIOReturnSuccess) {
        RNDLOG("ReportAvailableMediaTypes failed: 0x%08x", ret);
        return ret;
    }

    ether_addr_t mac = {};
    memcpy(mac.octet, ivars->macAddress, sizeof(mac.octet));
    IOUserNetworkPacketQueue * queues[] = {
        ivars->txSubmission, ivars->txCompletion, ivars->rxSubmission, ivars->rxCompletion,
    };
    ret = self->RegisterEthernetInterface(mac, ivars->pool, queues, sizeof(queues) / sizeof(queues[0]));
    if (ret != kIOReturnSuccess) {
        RNDLOG("RegisterEthernetInterface failed: 0x%08x", ret);
        return ret;
    }
    ivars->interfaceRegistered = true;

    // Publishing the service only now matters: registered before the Ethernet interface
    // exists, the stack never enabled the interface (observed on the Unisoc phone).
    ret = self->RegisterService();
    if (ret != kIOReturnSuccess) {
        RNDLOG("RegisterService failed: 0x%08x", ret);
        return ret;
    }
    RNDLOG("Ethernet interface registered for %02x:%02x:%02x:%02x:%02x:%02x "
           "(%u-packet pool, %u bulk IN x %u bytes, %u bulk OUT x %u bytes, max outbound frame %u)",
           mac.octet[0], mac.octet[1], mac.octet[2], mac.octet[3], mac.octet[4], mac.octet[5],
           kPacketPoolSize, kRxTransfers, kRxTransferSize, kTxTransfers, kTxTransferSize,
           MaxTxFrameLength(ivars));
    return kIOReturnSuccess;
}

void
TeardownNetworking(RNDISaurusDriver_IVars * ivars)
{
    StopDataFlow(ivars);

    for (uint32_t i = 0; i < kRxTransfers; i++) {
        ReleaseSlot(&ivars->rxSlots[i]);
    }
    for (uint32_t i = 0; i < kTxTransfers; i++) {
        ReleaseSlot(&ivars->txSlots[i]);
    }
    ivars->txFreeCount = 0;

    CancelAndRelease(&ivars->txPacketAction);
    OSSafeReleaseNULL(ivars->txSubmission);
    OSSafeReleaseNULL(ivars->txCompletion);
    OSSafeReleaseNULL(ivars->rxSubmission);
    OSSafeReleaseNULL(ivars->rxCompletion);
    OSSafeReleaseNULL(ivars->pool);

    if (ivars->interfaceRegistered) {
        ivars->interfaceRegistered = false;
        ivars->linkReportedActive  = false;
        // Two lines: unified logging cuts a dext message at roughly 250 characters.
        RNDLOG("data plane torn down: rx %llu packet(s) / %llu byte(s) in %llu transfer(s) (%llu batched); "
               "drops %llu no-buffer, %llu malformed, %llu oversize; %llu transfer error(s), "
               "%llu halt recover(ies)%{public}s",
               ivars->rxPackets, ivars->rxBytes, ivars->rxTransfers, ivars->rxBatchedTransfers,
               ivars->rxDropsNoBuffer, ivars->rxDropsMalformed, ivars->rxDropsOversize,
               ivars->rxTransferErrors, ivars->rxRecovery.recoveries,
               ivars->rxRecovery.parked ? ", parked" : "");
        RNDLOG("tx %llu packet(s) / %llu byte(s), %llu pad(s); %llu oversize, %llu error(s), "
               "%llu lost in aborted transfer(s), %llu dropped while parked, %llu halt recover(ies)%{public}s",
               ivars->txPackets, ivars->txBytes, ivars->txPads, ivars->txDropsOversize, ivars->txErrors,
               ivars->txDropsAborted, ivars->txDropsParked, ivars->txRecovery.recoveries,
               ivars->txRecovery.parked ? ", parked" : "");
    }
}
