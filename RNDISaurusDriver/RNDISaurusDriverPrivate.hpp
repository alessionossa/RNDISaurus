//
//  RNDISaurusDriverPrivate.hpp
//  RNDISaurus — USB tethering driver for RNDIS devices on macOS
//
//  Copyright (C) 2026 N4H4
//  SPDX-License-Identifier: MPL-2.0
//
//  This Source Code Form is subject to the terms of the Mozilla Public License,
//  v. 2.0. If a copy of the MPL was not distributed with this file, You can obtain
//  one at https://mozilla.org/MPL/2.0/.
//

// Shared between RNDISaurusDriver.cpp (lifecycle, USB plumbing),
// RNDISaurusControlPlane.cpp (RNDIS control plane over EP0, bring-up state machine,
// interrupt endpoint) and RNDISaurusDataPath.cpp (Ethernet interface, bulk transfers):
// the instance variables, the logging macro, the sizing constants and the helpers
// that cross file boundaries.

#ifndef RNDISaurusDriverPrivate_hpp
#define RNDISaurusDriverPrivate_hpp

#include <os/log.h>
#include <stdint.h>

#include <DriverKit/IOLib.h>
#include <DriverKit/IOBufferMemoryDescriptor.h>
#include <DriverKit/IODispatchQueue.h>
#include <DriverKit/IOTimerDispatchSource.h>
#include <DriverKit/OSAction.h>
#include <USBDriverKit/USBDriverKit.h>
#include <NetworkingDriverKit/NetworkingDriverKit.h>

#include "RNDISaurusDriver.h"
#include "RNDISaurusDiagnostics.h"
#include "RNDISProtocol.hpp"

class RNDISaurusDiagnosticsService;

// Logging. The dext can write only to OS_LOG_DEFAULT: DriverKit offers no way to
// create a log object with a subsystem of its own, as the host app does
// (com.n4h4.RNDISaurus). Each dext message therefore starts with the "[RNDISaurus]"
// tag, which is what to filter on:
//   log stream --predicate 'eventMessage CONTAINS "[RNDISaurus]"'
// Strings and pointers must be marked %{public} or unified logging shows <private>.
#define RNDLOG(fmt, ...) os_log(OS_LOG_DEFAULT, "[RNDISaurus] " fmt, ##__VA_ARGS__)

// Errors of the same kind are logged this many times, then only counted.
constexpr uint64_t kErrorLogLimit = 3;

// True while an occurrence count is still within kErrorLogLimit, i.e. worth a log line.
inline bool
WithinLogLimit(uint64_t count)
{
    return count <= kErrorLogLimit;
}

// Counts one more occurrence in `counter` (an lvalue) and logs it while the count is
// within kErrorLogLimit.
#define RNDLOG_COUNTED(counter, fmt, ...)                                          \
    do {                                                                           \
        if (WithinLogLimit(++(counter))) {                                         \
            RNDLOG(fmt, ##__VA_ARGS__);                                            \
        }                                                                          \
    } while (0)

// MARK: - Data plane sizing

// Bulk IN transfers kept in flight, each able to hold the largest transfer the host
// offered in INITIALIZE. Two so the pipe never idles between a completion and the
// next submission.
constexpr uint32_t kRxTransfers    = 2;
constexpr uint32_t kRxTransferSize = RNDIS::kHostMaxTransferSize;

// Bulk OUT transfers that may be in flight, one Ethernet frame each: 44-byte RNDIS
// header + a maximum-size frame + one optional pad byte fit with room to spare.
constexpr uint32_t kTxTransfers    = 4;
constexpr uint32_t kTxTransferSize = 2048;

// Consecutive halt recoveries (ClearStall + resubmit) a bulk pipe gets, with no
// successful transfer in between, before it is parked and the link reported down.
constexpr uint32_t kMaxPipeRecoveries = 3;

// Packet buffer pool shared by the four network queues: one buffer per packet, each
// large enough for a maximum-size Ethernet frame; the queue capacities together do
// not exceed the pool.
constexpr uint32_t kPacketPoolSize      = 64;
constexpr uint32_t kPacketBufferSize    = 2048;
constexpr uint32_t kPacketQueueCapacity = 16;

// The interface's MTU. RNDIS carries plain Ethernet frames, so the standard value.
constexpr uint32_t kMTU = ETHERMTU;

// MARK: - Shared helpers

// Completion statuses that mean the device is gone or the transfer was cancelled by
// us: nothing to resubmit, and no halt to clear (Stop follows, or the abort was ours).
// Every other error on a bulk or interrupt pipe leaves the endpoint halted and goes
// through that pipe's recovery.
inline bool
IsDeviceGoneStatus(kern_return_t status)
{
    return status == kIOReturnAborted || status == kIOReturnNotAttached
        || status == kIOReturnNoDevice || status == kIOReturnNotResponding
        || status == kIOReturnOffline;
}

// The same question for the result of a synchronous pipe call (an AsyncIO submission,
// a ClearStall): between the device leaving and Stop running, the pipe answers those
// with kIOReturnNotReady (seen on every unplug). Nothing to log or recover; Stop follows.
inline bool
IsDeviceLeavingStatus(kern_return_t status)
{
    return IsDeviceGoneStatus(status) || status == kIOReturnNotReady;
}

// Cancels a dispatch source or action and releases it once every callback has drained
// (an OSAction retains its target until cancelled, so this must never wait for free()).
// Nulls *ref first; a no-op on NULL. Releases immediately if the cancel is refused.
// `companion` (may be NULL) is released at the same moment, for a buffer the drained
// callbacks may still touch. The cancel handler captures only these two pointers.
template <typename T>
inline void
CancelAndRelease(T ** ref, OSObject * companion = nullptr)
{
    T * object = *ref;
    if (object == nullptr) {
        OSSafeReleaseNULL(companion);
        return;
    }
    *ref = nullptr;
    kern_return_t ret = object->Cancel(^{
        object->release();
        if (companion != nullptr) {
            companion->release();
        }
    });
    if (ret != kIOReturnSuccess) {
        RNDLOG("Cancel failed: 0x%08x; releasing immediately", ret);
        object->release();
        OSSafeReleaseNULL(companion);
    }
}

// Creates a buffer descriptor of `capacity` bytes for `direction`, gives it its full
// length (a fresh descriptor has zero length until told otherwise) and maps it for the
// host into *bytes. `name` prefixes the failure logs. On failure *buffer may be left
// non-NULL for the caller to release. (RNDISaurusDriver.cpp)
kern_return_t CreateMappedBuffer(uint64_t direction, uint32_t capacity, const char * name,
                                 IOBufferMemoryDescriptor ** buffer, uint8_t ** bytes);

struct RNDISaurusDriver_IVars;

// Remembers `status` as the last error, with the step (an RNDISaurusErrorStage) that hit
// it, for the diagnostics snapshot. Logging stays with the caller. (RNDISaurusDriver.cpp)
void RecordError(RNDISaurusDriver_IVars * ivars, uint32_t stage, kern_return_t status);

// Copies the driver's state and counters into *snapshot for the diagnostics user
// client. Must run on the driver's default queue, like every other reader and writer of
// the instance variables; the user client arranges that. (RNDISaurusDriver.cpp)
void FillSnapshot(const RNDISaurusDriver_IVars * ivars, RNDISaurusSnapshot * snapshot);

// MARK: - Instance variables

// One asynchronous USB transfer: its buffer, the host mapping of that buffer, the
// completion action whose reference storage holds the slot's index, and the pipe's
// halt generation at the time the transfer was submitted (bulk slots only).
struct TransferSlot {
    IOBufferMemoryDescriptor * buffer;
    uint8_t *                  bytes;
    OSAction *                 action;
    bool                       inFlight;
    uint32_t                   generation;
};

// Which completion method a transfer slot's action targets; also fixes the slot's
// memory direction and log name.
enum class SlotKind : uint8_t {
    BulkIn,     // BulkInComplete
    BulkOut,    // BulkOutComplete
    Interrupt,  // InterruptComplete
};

// Halt recovery (ClearStall + resubmit) of one bulk pipe, bounded. A parked pipe stays
// idle, with the link reported down, until the interface is re-enabled.
struct PipeRecovery {
    uint32_t haltGeneration;        // bumped by every successful ClearStall on the pipe
    uint32_t consecutiveRecoveries; // recoveries since the last successful completion
    uint64_t recoveries;            // ClearStall rounds in total, for the teardown log
    bool     parked;
};

struct RNDISaurusDriver_IVars {
    // Both interfaces are stored only once opened, so non-NULL means "open by us".
    IOUSBHostInterface *       controlInterface;   // the matched provider, retained
    IOUSBHostInterface *       dataInterface;      // CDC-Data sibling at control + 1
    IOUSBHostPipe *            interruptPipe;      // RESPONSE_AVAILABLE notifications
    IOUSBHostPipe *            bulkInPipe;
    IOUSBHostPipe *            bulkOutPipe;
    IOBufferMemoryDescriptor * commandBuffer;      // kControlBufferSize bytes for EP0 I/O
    uint8_t *                  commandBytes;       // host mapping of commandBuffer

    uint8_t  controlInterfaceNumber;
    uint8_t  dataInterfaceNumber;
    uint8_t  deviceSpeed;                          // tIOUSBHostConnectionSpeed
    uint16_t interruptMaxPacketSize;
    uint16_t bulkInMaxPacketSize;
    uint16_t bulkOutMaxPacketSize;

    // Device identity from the registry, for the diagnostics snapshot.
    uint32_t vendorID;                             // idVendor, or 0xFFFFFFFF if unreadable
    uint32_t productID;                            // idProduct, or 0xFFFFFFFF if unreadable
    char     productName[64];                      // USB product string, or empty

    // The child service the host app opens for diagnostics; created after the USB
    // setup, terminated in teardown. NULL when it could not be created.
    RNDISaurusDiagnosticsService * diagnosticsService;

    // Everything below runs on the driver's default dispatch queue: Start and Stop,
    // the bring-up timer, the interrupt completion, the four network queues, the bulk
    // completions and the diagnostics user client's snapshot method. Nothing here
    // needs a lock. The synchronous calls made from that queue (control requests,
    // ClearStall) hold everything else up while they run.
    IODispatchQueue * defaultQueue;

    // The last error recorded with RecordError, for the diagnostics snapshot.
    uint32_t      lastErrorStage;                  // RNDISaurusErrorStage
    kern_return_t lastError;
    uint32_t      lastErrorAttempt;                // bringUpAttempts at the time
    uint64_t      errorEvents;                     // RecordError calls since Start

    // RNDIS control plane state.
    bool rndisInitialized;                         // INITIALIZE completed; HALT on teardown
    bool filterSet;                                // non-zero packet filter set; clear on teardown

    uint32_t nextRequestID;                        // never hands out 0
    uint32_t deviceMaxTransferSize;                // from INITIALIZE_CMPLT
    uint32_t maxPacketsPerTransfer;
    uint32_t packetAlignmentFactor;
    uint8_t  macAddress[6];                        // OID_802_3_PERMANENT_ADDRESS

    // Bring-up state machine. Start opens USB and schedules the first attempt; every
    // attempt runs INITIALIZE → MAC query → packet filter and, on success, registers
    // the Ethernet interface. Failed attempts are retried with a back-off after a
    // RESET for as long as the device stays attached. The states are the diagnostics
    // header's, so the snapshot carries them as they are.
    RNDISaurusBringUpState  bringUpState;
    uint32_t                bringUpAttempts;       // attempts started; 1 is the first
    uint32_t                resets;                // RESETs sent (one before every retry)
    uint32_t                nextRetryDelayMs;
    uint64_t                linkUpAtNs;            // RNDISaurusClockNow() when the state
                                                   // became Up; 0 before. Set once: Failed
                                                   // never becomes Up, and a replug starts a
                                                   // new driver instance.
    IOTimerDispatchSource * bringUpTimer;          // on defaultQueue
    OSAction *              bringUpTimerAction;

    // Interrupt endpoint: one RESPONSE_AVAILABLE buffer, re-armed after every
    // completion from the first bring-up attempt until teardown.
    TransferSlot interruptSlot;
    uint32_t     interruptConsecutiveErrors;
    bool         interruptParked;                  // reader gave up; responses are still polled
    uint64_t     notifications;                    // completions that carried data
    uint64_t     unknownNotifications;             // not the 8-byte RESPONSE_AVAILABLE
    uint64_t     responsesAnnounced;               // RESPONSE_AVAILABLE notifications
    uint64_t     responsesConsumed;                // messages any GET has read
    uint64_t     notificationsAlreadyServed;       // announced message a command had already read
    uint64_t     emptyDrains;                      // drain whose GET found nothing after all
    uint64_t     drainedMessages;                  // messages read outside a command
    uint64_t     droppedMessages;                  // stale/unexpected/malformed, dropped
    uint64_t     indicateStatusCount;
    uint64_t     keepalivesAnswered;
    uint64_t     interruptErrors;

    // Link state as reported to the stack.
    bool mediaConnected;                           // last INDICATE_STATUS; true until told otherwise
    bool linkReportedActive;                       // what ReportLinkStatus was last told

    // Ethernet interface.
    IOUserNetworkPacketBufferPool *  pool;
    IOUserNetworkTxSubmissionQueue * txSubmission; // frames from the stack
    IOUserNetworkTxCompletionQueue * txCompletion; // frames handed back to the stack
    IOUserNetworkRxSubmissionQueue * rxSubmission; // empty packets from the stack
    IOUserNetworkRxCompletionQueue * rxCompletion; // received frames to the stack
    OSAction *                       txPacketAction;
    bool     interfaceRegistered;
    bool     interfaceEnabled;                     // between SetInterfaceEnable(true/false)
    uint32_t activeMediaType;                      // reported with the link status
    uint32_t loggedCallbacks;                      // bit set per callback already logged

    TransferSlot rxSlots[kRxTransfers];
    TransferSlot txSlots[kTxTransfers];
    uint32_t     txFreeSlots[kTxTransfers];        // stack of idle transmit slot indices
    uint32_t     txFreeCount;

    PipeRecovery rxRecovery;                       // bulk IN
    PipeRecovery txRecovery;                       // bulk OUT

    // Data plane counters, reported in the log at teardown.
    uint64_t rxPackets;
    uint64_t rxBytes;
    uint64_t rxDropsNoBuffer;                      // no packet from the stack, or its queue full
    uint64_t rxDropsMalformed;                     // batch walk stopped at a bad header
    uint64_t rxDropsOversize;                      // frame larger than a packet buffer
    uint64_t rxTransferErrors;                     // bulk IN completed with, or could not be armed for, an error
    uint64_t rxTransfers;                          // bulk IN transfers completed and walked
    uint64_t rxBatchedTransfers;                   // of those, transfers carrying more than one PACKET message
    uint64_t txPackets;
    uint64_t txBytes;
    uint64_t txDropsOversize;                      // frame the device cannot accept
    uint64_t txDropsAborted;                       // frames lost in transfers a ClearStall aborted
    uint64_t txDropsParked;                        // frames dropped while bulk OUT was parked
    uint64_t txErrors;                             // bulk OUT submission or completion failed
    uint64_t txPads;                               // pad bytes appended to avoid a zero-length packet
};

// MARK: - Control plane entry points (RNDISaurusControlPlane.cpp)

// Arms the interrupt endpoint reader (its absence or failure is tolerated: responses
// are polled anyway), creates the bring-up timer and schedules the first attempt.
// Requires the USB interfaces, pipes, command buffer and default queue.
kern_return_t StartBringUp(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars);

// Undoes StartBringUp: cancels the timer and the interrupt reader (their buffer and
// actions are released once every callback has drained) and logs the control-plane
// counters. Safe on a partially set-up state and more than once.
void TeardownControlPlane(RNDISaurusDriver_IVars * ivars);

// The RNDIS farewell: SET packet filter 0, then HALT, each only if the corresponding
// bring-up step had succeeded. On unplug both fail fast because the device is gone.
void RNDISFarewell(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars);

// MARK: - Data plane entry points (RNDISaurusDataPath.cpp)

// Gives `slot` a completion action of the given kind with `index` in the action's
// reference storage, and a buffer of `capacity` bytes mapped for the host, in the
// direction the kind implies. Whatever was created stays in the slot on failure, for
// ReleaseSlot to clean up.
kern_return_t PrepareSlot(RNDISaurusDriver * self, TransferSlot * slot, uint32_t index,
                          SlotKind kind, uint32_t capacity);

// Cancels the slot's action and releases action and buffer once every callback has
// drained. A no-op on an empty slot.
void ReleaseSlot(TransferSlot * slot);

// Creates the packet pool, the four queues and the bulk transfer slots, registers the
// Ethernet interface with the MAC address queried from the device, and publishes the
// service. Requires the USB pipes, the RNDIS control plane and the default queue.
kern_return_t SetUpNetworking(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars);

// Undoes SetUpNetworking: stops the data flow, cancels the transfer actions (their
// buffers are released once every completion has drained) and releases the queues
// and the pool. Safe on a partially set-up state and more than once.
void TeardownNetworking(RNDISaurusDriver_IVars * ivars);

// Reports the link to the stack from the interface's enabled state and the device's
// last media indication; the only caller of ReportLinkStatus. A no-op before the
// interface is registered. Logs transitions only.
kern_return_t UpdateLinkStatus(RNDISaurusDriver * self, RNDISaurusDriver_IVars * ivars);

#endif /* RNDISaurusDriverPrivate_hpp */
