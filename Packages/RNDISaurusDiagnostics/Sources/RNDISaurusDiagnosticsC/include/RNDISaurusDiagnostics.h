//
//  RNDISaurusDiagnostics.h
//  RNDISaurus — USB tethering driver for RNDIS devices on macOS
//
//  Copyright (C) 2026 N4H4
//  SPDX-License-Identifier: MIT
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in the
//  Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
//  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
//  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
//  AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
//  WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

// The diagnostics contract between the driver extension and its clients (the host app
// and the embedded probe): the selector of the user client's snapshot method and the
// snapshot's layout. Plain C so that the same header is compiled into the dext and
// imported into Swift as the RNDISaurusDiagnosticsC module. Every field has a fixed
// width and an explicit position, and
// the struct starts with its own size and layout version so that an app and a driver
// built from different revisions detect the mismatch instead of misreading each other.

#ifndef RNDISaurusDiagnostics_h
#define RNDISaurusDiagnostics_h

#include <stdint.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

// Layout version of RNDISaurusSnapshot; bumped whenever the struct changes.
enum { kRNDISaurusDiagnosticsVersion = 3 };

// The clock every timestamp in the snapshot is taken on: monotonic, system-wide, and
// counting through system sleep. Part of the contract rather than each side's business,
// because the driver stamps a value that a client subtracts from its own reading: the
// two have to be the same clock or the difference is meaningless. Swift's
// `ContinuousClock` is this clock, but its `Instant` can be neither built from nor
// reduced to an absolute reading, so clients call this too.
static inline uint64_t
RNDISaurusClockNow(void)
{
    return clock_gettime_nsec_np(CLOCK_MONOTONIC_RAW);
}

// The only `type` argument of IOServiceOpen the driver accepts.
enum { kRNDISaurusDiagnosticsUserClientType = 0 };

// Selectors of the user client's external methods.
enum RNDISaurusDiagnosticsSelector {
    kRNDISaurusDiagnosticsSelectorSnapshot = 0, // no input; output: one RNDISaurusSnapshot
    kRNDISaurusDiagnosticsSelectorCount
};

// Where the RNDIS bring-up stands (RNDISaurusSnapshot.bringUpState).
enum RNDISaurusBringUpState {
    kRNDISaurusBringUpNotStarted     = 0, // USB open, first attempt scheduled
    kRNDISaurusBringUpInProgress     = 1, // an attempt is running
    kRNDISaurusBringUpRetryScheduled = 2, // the last attempt failed; the next one is timed
    kRNDISaurusBringUpUp             = 3, // control plane up and Ethernet interface registered
    kRNDISaurusBringUpFailed         = 4  // the interface could not be set up; idle until unplug
};

// The value of vendorID and productID when the registry property could not be read.
enum { kRNDISaurusUnknownID = 0xFFFFFFFFu };

// USB connection speed (RNDISaurusSnapshot.deviceSpeed): the values of
// tIOUSBHostConnectionSpeed, which the driver pins with static assertions.
enum RNDISaurusDeviceSpeed {
    kRNDISaurusSpeedNone         = 0,
    kRNDISaurusSpeedFull         = 1,
    kRNDISaurusSpeedLow          = 2,
    kRNDISaurusSpeedHigh         = 3,
    kRNDISaurusSpeedSuper        = 4,
    kRNDISaurusSpeedSuperPlus    = 5,
    kRNDISaurusSpeedSuperPlusBy2 = 6
};

// Media reported with the link (RNDISaurusSnapshot.activeMediaType): the values of the
// IOUserNetworkMediaType constants the driver uses, pinned with static assertions.
enum RNDISaurusMediaType {
    kRNDISaurusMediaNone              = 0x00000000,
    kRNDISaurusMediaEthernetAuto      = 0x00000020, // IFM_ETHER | IFM_AUTO
    kRNDISaurusMediaEthernet100BaseTX = 0x00000026, // IFM_ETHER | IFM_100_TX
    kRNDISaurusMediaEthernet1000BaseT = 0x00000030  // IFM_ETHER | IFM_1000_T
};

// Which step recorded the last error (RNDISaurusSnapshot.lastErrorStage).
enum RNDISaurusErrorStage {
    kRNDISaurusErrorStageNone              = 0,
    kRNDISaurusErrorStageUSBSetup          = 1,  // opening the interfaces, pipes or command buffer
    kRNDISaurusErrorStageInitialize        = 2,  // REMOTE_NDIS_INITIALIZE_MSG
    kRNDISaurusErrorStageQueryMAC          = 3,  // QUERY OID_802_3_PERMANENT_ADDRESS
    kRNDISaurusErrorStageSetFilter         = 4,  // SET OID_GEN_CURRENT_PACKET_FILTER
    kRNDISaurusErrorStageEthernetInterface = 5,  // packet pool, queues or RegisterEthernetInterface
    kRNDISaurusErrorStageInterruptIn       = 6,  // the notification endpoint
    kRNDISaurusErrorStageBulkIn            = 7,  // a receive transfer
    kRNDISaurusErrorStageBulkOut           = 8,  // a transmit transfer
    kRNDISaurusErrorStageBulkInParked      = 9,  // receive recovery gave up
    kRNDISaurusErrorStageBulkOutParked     = 10  // transmit recovery gave up
};

// What the driver knows about one matched device, copied atomically with respect to
// the driver's own work. 368 bytes; every 8-byte field is 8-byte aligned.
typedef struct RNDISaurusSnapshot {
    uint32_t size;                    // sizeof(RNDISaurusSnapshot) as the driver was built
    uint32_t version;                 // kRNDISaurusDiagnosticsVersion as the driver was built

    // Session.
    uint64_t linkUpAtNs;              // RNDISaurusClockNow() when the RNDIS session came up
                                      // and the Ethernet interface was registered; 0 before
                                      // that. Subtract it from RNDISaurusClockNow() to get
                                      // the session's age. Not re-stamped when the device's
                                      // media indication flaps.

    // Device.
    uint32_t vendorID;                // idVendor, or kRNDISaurusUnknownID
    uint32_t productID;               // idProduct, or kRNDISaurusUnknownID
    uint8_t  deviceSpeed;             // RNDISaurusDeviceSpeed
    uint8_t  controlInterfaceNumber;
    uint8_t  dataInterfaceNumber;
    uint8_t  reserved0;
    uint8_t  macAddress[6];           // OID_802_3_PERMANENT_ADDRESS; zero before it is known
    uint16_t reserved1;

    // Bring-up.
    uint32_t bringUpState;            // RNDISaurusBringUpState
    uint32_t bringUpAttempts;         // attempts started; 1 is the first
    uint32_t resets;                  // REMOTE_NDIS_RESET_MSG sent (one before every retry)
    uint32_t nextRetryDelayMs;
    uint32_t deviceMaxTransferSize;   // from INITIALIZE_CMPLT
    uint32_t maxPacketsPerTransfer;
    uint32_t packetAlignmentFactor;
    uint32_t activeMediaType;         // RNDISaurusMediaType reported with the link

    // Flags (0 or 1).
    uint8_t  rndisInitialized;
    uint8_t  filterSet;
    uint8_t  interfaceRegistered;
    uint8_t  interfaceEnabled;        // between SetInterfaceEnable(true/false)
    uint8_t  mediaConnected;          // last device media indication
    uint8_t  linkReportedActive;      // what the network stack was last told
    uint8_t  rxParked;                // bulk IN recovery gave up until re-enable
    uint8_t  txParked;                // bulk OUT recovery gave up until re-enable
    uint8_t  interruptParked;         // the notification reader gave up; responses are polled
    uint8_t  reserved2[3];

    // Last error.
    uint32_t lastErrorStage;          // RNDISaurusErrorStage
    uint32_t lastErrorAttempt;        // bringUpAttempts when it was recorded
    int32_t  lastError;               // kern_return_t
    uint32_t reserved3;
    uint64_t errorEvents;             // errors recorded since Start

    // Receive path.
    uint64_t rxPackets;
    uint64_t rxBytes;
    uint64_t rxDropsNoBuffer;         // no packet from the stack, or its queue full
    uint64_t rxDropsMalformed;        // batch walk stopped at a bad header, or a runt
    uint64_t rxDropsOversize;         // frame larger than a packet buffer
    uint64_t rxTransferErrors;        // bulk IN completed with, or could not be armed for, an error
    uint64_t rxRecoveries;            // ClearStall rounds on bulk IN
    uint64_t rxTransfers;             // bulk IN transfers completed and walked
    uint64_t rxBatchedTransfers;      // of those, transfers carrying more than one PACKET message

    // Transmit path.
    uint64_t txPackets;
    uint64_t txBytes;
    uint64_t txDropsOversize;         // frame the device cannot accept
    uint64_t txDropsAborted;          // frames lost in transfers a ClearStall aborted
    uint64_t txDropsParked;           // frames dropped while bulk OUT was parked
    uint64_t txErrors;                // bulk OUT submission or completion failed
    uint64_t txRecoveries;            // ClearStall rounds on bulk OUT
    uint64_t txPads;                  // pad bytes appended to avoid a zero-length packet

    // Control plane.
    uint64_t notifications;           // interrupt completions that carried data
    uint64_t unknownNotifications;    // not the 8-byte RESPONSE_AVAILABLE
    uint64_t drainedMessages;         // messages read outside a command
    uint64_t droppedMessages;         // stale, unexpected or malformed responses
    uint64_t indicateStatusCount;
    uint64_t keepalivesAnswered;
    uint64_t interruptErrors;
    uint64_t reserved4;

    char     productName[64];         // USB product string, NUL-terminated, truncated to fit
} RNDISaurusSnapshot;

#ifdef __cplusplus
}
#endif

#endif /* RNDISaurusDiagnostics_h */
