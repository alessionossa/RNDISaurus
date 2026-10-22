//
//  RNDISProtocol.hpp
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
//  Behaviour informed by the Linux RNDIS host driver, jwise/HoRNDIS and
//  JellyBrick/ReRNDIS. The constant naming and the packet framing and batch-walk
//  interfaces follow XiaoMiku01/TetherKit (MIT; notice in NOTICE.md).
//

#ifndef RNDISProtocol_hpp
#define RNDISProtocol_hpp

#include <stddef.h>
#include <stdint.h>

// RNDIS wire format: message builders, parsers and constants.
//
// Everything here works on plain byte buffers and depends only on the C standard
// headers, so it has no DriverKit dependency and can be compiled and exercised
// outside the driver. Every wire field is a 32-bit little-endian value. Messages are
// read and written one field at a time through ReadLE32/WriteLE32 instead of being
// overlaid with packed structs, which keeps every access independent of the
// buffer's alignment and of the host's byte order.
//
// QUERY and SET messages and their completions locate their information buffer with
// an InformationBufferOffset counted from byte 8, where the RequestID word sits, not
// from byte 0 of the message.
namespace RNDIS {

// Message types. A completion has the same type with kMessageCompletionFlag set.
enum : uint32_t {
    kMessageTypePacket         = 0x00000001,
    kMessageTypeInitialize     = 0x00000002,
    kMessageTypeHalt           = 0x00000003, // never answered by the device
    kMessageTypeQuery          = 0x00000004,
    kMessageTypeSet            = 0x00000005,
    kMessageTypeReset          = 0x00000006,
    kMessageTypeIndicateStatus = 0x00000007, // unsolicited, device to host
    kMessageTypeKeepalive      = 0x00000008, // either direction; must be answered
    kMessageCompletionFlag     = 0x80000000,
};

// True when the device answers `messageType` with a completion. HALT is the only
// request that is never answered: the device forgets all RNDIS state and stays silent.
inline bool ExpectsCompletion(uint32_t messageType)
{
    return messageType != kMessageTypeHalt;
}

// Human-readable name of a message type (request or completion), for logging.
const char * MessageTypeName(uint32_t messageType);

// Status codes (NDIS_STATUS values). Any status with both top bits set is an error.
enum : uint32_t {
    kStatusSuccess         = 0x00000000,
    kStatusFailure         = 0xC0000001,
    kStatusNotSupported    = 0xC00000BB,
    kStatusInvalidLength   = 0xC0010014,
    kStatusInvalidData     = 0xC0010015,
    kStatusMediaConnect    = 0x4001000B, // INDICATE_STATUS payloads
    kStatusMediaDisconnect = 0x4001000C,
};

// Human-readable name of an INDICATE_STATUS code, for logging; "other" when unknown.
const char * StatusName(uint32_t status);

// The RESPONSE_AVAILABLE notification the device sends on its interrupt endpoint when
// a message is queued for GET_ENCAPSULATED_RESPONSE. "Remote NDIS to USB Mapping"
// defines it as 8 bytes: the LE32 notification code 1, then a reserved zero word.
constexpr size_t kResponseAvailableSize = 8;

// True when `length` bytes of `buffer` are a RESPONSE_AVAILABLE notification.
bool IsResponseAvailable(const uint8_t * buffer, size_t length);

// Object identifiers. The driver queries the permanent address and sets the packet
// filter; the maximum-frame-size and physical-medium OIDs are kept for planned queries.
enum : uint32_t {
    kOIDGeneralMaximumFrameSize    = 0x00010106,
    kOIDGeneralCurrentPacketFilter = 0x0001010E,
    kOIDGeneralPhysicalMedium      = 0x00010202,
    kOID8023PermanentAddress       = 0x01010101,
};

// Packet filter bits for kOIDGeneralCurrentPacketFilter.
enum : uint32_t {
    kPacketTypeDirected     = 0x00000001,
    kPacketTypeMulticast    = 0x00000002,
    kPacketTypeAllMulticast = 0x00000004,
    kPacketTypeBroadcast    = 0x00000008,
    kPacketTypePromiscuous  = 0x00000020,
};

// The filter every known host implementation sends. Android/Linux gadgets only check
// that it is non-zero; a non-zero filter is what starts data flow, zero pauses it.
constexpr uint32_t kDefaultPacketFilter =
    kPacketTypeDirected | kPacketTypeBroadcast | kPacketTypeAllMulticast | kPacketTypePromiscuous;

// Version advertised in INITIALIZE.
constexpr uint32_t kMajorVersion = 1;
constexpr uint32_t kMinorVersion = 0;

// Size of the buffer offered to GET_ENCAPSULATED_RESPONSE: the 1024-byte minimum that
// "Remote NDIS to USB Mapping" sets for control-channel transfers.
constexpr size_t kControlBufferSize = 1024;

// Largest bulk-IN transfer the host is prepared to receive, advertised in INITIALIZE.
// 16 KiB is the value that works across devices; larger values hurt some gadgets, and
// mainline Linux gadgets ignore it anyway and answer with their own limit.
constexpr uint32_t kHostMaxTransferSize = 16384;

// Size of the zeroed information buffer sent with a QUERY whose answer has a fixed
// size. Some devices only answer such a QUERY when it already reserves room for the
// answer; a QUERY for a variable-size OID goes out without one. 48 bytes is enough for
// every fixed-size OID this driver queries.
constexpr uint32_t kFixedOIDPlaceholderBytes = 48;

// Message sizes in bytes (fixed parts).
constexpr size_t kHeaderSize                 = 8;  // MessageType, MessageLength
constexpr size_t kInitializeSize             = 24;
constexpr size_t kInitializeCompleteSize     = 52;
constexpr size_t kHaltSize                   = 12;
constexpr size_t kResetSize                  = 12; // MessageType, MessageLength, Reserved
constexpr size_t kResetCompleteSize          = 16; // MessageType, MessageLength, Status, AddressingReset
constexpr size_t kQuerySetHeaderSize         = 28; // + information buffer
constexpr size_t kQueryCompleteHeaderSize    = 24; // + information buffer
constexpr size_t kSetCompleteSize            = 16;
constexpr size_t kIndicateStatusHeaderSize   = 20; // + optional status buffer
constexpr size_t kKeepaliveSize              = 12;
constexpr size_t kKeepaliveCompleteSize      = 16;
constexpr size_t kPacketHeaderSize           = 44;

// Offset of the information buffer when it directly follows a QUERY/SET header,
// expressed relative to the RequestID field (28 - 8).
constexpr uint32_t kQuerySetInlineInformationOffset = 20;

// Offset of an Ethernet frame that directly follows a PACKET header, expressed
// relative to the DataOffset field (44 - 8).
constexpr uint32_t kPacketInlineDataOffset = 36;

// Little-endian accessors. `offset` is in bytes from `buffer`.
uint32_t ReadLE32(const uint8_t * buffer, size_t offset);
void     WriteLE32(uint8_t * buffer, size_t offset, uint32_t value);

// True for any NDIS_STATUS error code.
inline bool IsFailureStatus(uint32_t status)
{
    return (status & 0xC0000000u) == 0xC0000000u;
}

// The common first four fields of every control message except RESET_CMPLT, whose
// layout is MessageType, MessageLength, Status, AddressingReset (no RequestID): for
// that one message `requestID` and `status` hold Status and AddressingReset instead,
// and ParseResetComplete is the accessor to use.
struct Header {
    uint32_t messageType;
    uint32_t messageLength;
    uint32_t requestID;
    uint32_t status; // only meaningful for completions and INDICATE_STATUS
};

// Fields of INITIALIZE_CMPLT the driver cares about.
struct InitializeResult {
    uint32_t status;
    uint32_t majorVersion;
    uint32_t minorVersion;
    uint32_t deviceFlags;           // 1 = connectionless
    uint32_t medium;                // 0 = 802.3
    uint32_t maxPacketsPerTransfer; // packets the device may batch per bulk-IN transfer
    uint32_t maxTransferSize;       // largest transfer the device accepts on bulk OUT
    uint32_t packetAlignmentFactor; // batched messages are aligned to 1 << factor
};

// Builders write a complete message at the start of `buffer` (zero-filling every byte
// they claim) and return its length, or 0 when `capacity` is too small.
size_t BuildInitialize(uint8_t * buffer, size_t capacity, uint32_t requestID,
                       uint32_t maxTransferSize);
size_t BuildQuery(uint8_t * buffer, size_t capacity, uint32_t requestID, uint32_t oid,
                  uint32_t informationLength);
size_t BuildSet(uint8_t * buffer, size_t capacity, uint32_t requestID, uint32_t oid,
                const uint8_t * information, uint32_t informationLength);
size_t BuildSetPacketFilter(uint8_t * buffer, size_t capacity, uint32_t requestID,
                            uint32_t filter);
size_t BuildHalt(uint8_t * buffer, size_t capacity, uint32_t requestID);
// RESET carries no RequestID (its third word is Reserved), so its completion can only
// be matched by type.
size_t BuildReset(uint8_t * buffer, size_t capacity);
size_t BuildKeepaliveComplete(uint8_t * buffer, size_t capacity, uint32_t requestID);

// Parsers validate against `length` (bytes actually received) and against the
// message's own MessageLength, and never read past either. They return false for
// anything malformed.
bool ParseHeader(const uint8_t * buffer, size_t length, Header * header);
bool ParseInitializeComplete(const uint8_t * buffer, size_t length,
                             InitializeResult * result);
// On success `*information` points into `buffer` and `*informationLength` is the
// device's InformationBufferLength (0 when the status is not success).
bool ParseQueryComplete(const uint8_t * buffer, size_t length, uint32_t * status,
                        const uint8_t ** information, uint32_t * informationLength);
bool ParseSetComplete(const uint8_t * buffer, size_t length, uint32_t * status);
// RESET_CMPLT: Status at offset 8, AddressingReset at 12 (1 = the device dropped its
// addressing state, i.e. the packet filter must be SET again).
bool ParseResetComplete(const uint8_t * buffer, size_t length, uint32_t * status,
                        uint32_t * addressingReset);
// Only the status word is read. The optional status buffer is skipped: devices are not
// consistent about what its offset is counted from.
bool ParseIndicateStatus(const uint8_t * buffer, size_t length, uint32_t * status);

// What a GET_ENCAPSULATED_RESPONSE transfer turned out to be, for the command loop.
enum class ResponseKind {
    NotReady,               // fewer than kHeaderSize bytes: nothing queued yet
    Malformed,              // header does not parse
    Completion,             // the completion of the request we are waiting for
    CompletionOtherRequest, // right completion type, wrong RequestID (a late reply)
    IndicateStatus,         // unsolicited INDICATE_STATUS; handle and keep waiting
    Keepalive,              // device-initiated KEEPALIVE; answer and keep waiting
    Unexpected,             // any other message type
};

// Classifies a received transfer against the request (type without completion flag,
// RequestID) whose completion is expected. `header` is filled whenever it parsed.
// A RESET completion is matched by type alone (it carries no RequestID). Pass
// `expectedMessageType` 0 when no completion is expected at all, e.g. when draining
// the queue after a RESPONSE_AVAILABLE notification: every completion is then
// reported as Unexpected.
ResponseKind ClassifyResponse(const uint8_t * buffer, size_t length,
                              uint32_t expectedMessageType, uint32_t expectedRequestID,
                              Header * header);

// MARK: - Data plane: REMOTE_NDIS_PACKET_MSG

// Writes the 44-byte header of a PACKET message carrying one Ethernet frame of
// `frameLength` bytes that the caller copies right after the header. Returns
// kPacketHeaderSize, or 0 when `capacity` is too small. The out-of-band and
// per-packet-info fields are left zero, as every known device expects.
size_t BuildPacketHeader(uint8_t * buffer, size_t capacity, uint32_t frameLength);

// Largest Ethernet frame FramePacket can place in a transfer buffer of `capacity` bytes
// for a device that accepts at most `deviceMaxTransferSize` bytes per transfer (its
// INITIALIZE_CMPLT value; a value not larger than the header is treated as unknown):
// header, frame and one pad byte must fit the buffer, header and frame the device's
// limit. The caller caps it at what the interface carries.
uint32_t MaxFrameLength(size_t capacity, uint32_t deviceMaxTransferSize);

// Writes one complete PACKET message carrying `frame` at the start of `buffer` and
// returns the bulk OUT transfer length, or 0 when the frame is empty or does not leave
// room for the pad byte. When the length divides evenly by the endpoint's packet size
// (`bulkOutMaxPacketSize`, 0 when unknown), USB would close the transfer with a
// zero-length packet, which RNDIS devices do not handle; one zero byte past the end of
// the message then shortens the last packet instead, and `*padded` (may be NULL)
// reports it.
size_t FramePacket(uint8_t * buffer, size_t capacity, const uint8_t * frame, uint32_t frameLength,
                   uint16_t bulkOutMaxPacketSize, bool * padded);

// Outcome of one step through a received bulk-IN transfer.
enum class BatchStep {
    Frame,     // *frame / *frameLength describe the next Ethernet frame
    End,       // clean end: no bytes left, or only zero padding
    Malformed, // a header failed validation; stop walking and drop what is left
};

// Walks the PACKET messages batched in a bulk-IN transfer of `length` bytes.
// `*cursor` starts at 0 and is advanced past each message, including any alignment
// padding the device folded into that message's MessageLength. No header field is used
// before it is checked against both the bytes received and the MessageLength the
// message declares; a MessageLength shorter than the header is rejected so the walk
// always advances.
BatchStep NextPacket(const uint8_t * buffer, size_t length, size_t * cursor,
                     const uint8_t ** frame, uint32_t * frameLength);

} // namespace RNDIS

#endif /* RNDISProtocol_hpp */
