//
//  RNDISProtocol.cpp
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
//  JellyBrick/ReRNDIS, which also suggested the zero-length-packet pad on transmit.
//  The query-header helper, packet framing and batch walk follow XiaoMiku01/TetherKit
//  (MIT; notice in NOTICE.md).
//

#include "RNDISProtocol.hpp"

#include <string.h>

namespace RNDIS {

// Field offsets shared by the fixed-layout messages (bytes from message start).
namespace {
constexpr size_t kOffsetMessageType   = 0;
constexpr size_t kOffsetMessageLength = 4;
constexpr size_t kOffsetRequestID     = 8;
constexpr size_t kOffsetStatus        = 12; // completions; INDICATE_STATUS has it at 8

// INITIALIZE
constexpr size_t kOffsetInitMajorVersion    = 12;
constexpr size_t kOffsetInitMinorVersion    = 16;
constexpr size_t kOffsetInitMaxTransferSize = 20;

// INITIALIZE_CMPLT
constexpr size_t kOffsetInitCMajorVersion          = 16;
constexpr size_t kOffsetInitCMinorVersion          = 20;
constexpr size_t kOffsetInitCDeviceFlags           = 24;
constexpr size_t kOffsetInitCMedium                = 28;
constexpr size_t kOffsetInitCMaxPacketsPerTransfer = 32;
constexpr size_t kOffsetInitCMaxTransferSize       = 36;
constexpr size_t kOffsetInitCPacketAlignmentFactor = 40;

// QUERY / SET (request) and QUERY_CMPLT: the InformationBuffer length/offset pair sits
// at the same positions in all three.
constexpr size_t kOffsetQuerySetOID               = 12;
constexpr size_t kOffsetQuerySetInformationLength = 16;
constexpr size_t kOffsetQuerySetInformationOffset = 20;

// INDICATE_STATUS
constexpr size_t kOffsetIndicateStatus = 8;

// RESPONSE_AVAILABLE notification: the first word is the notification code.
constexpr uint32_t kNotificationResponseAvailable = 1;

// RESET_CMPLT (no RequestID)
constexpr size_t kOffsetResetCStatus          = 8;
constexpr size_t kOffsetResetCAddressingReset = 12;

// PACKET: DataOffset is relative to its own field, i.e. to message start + 8.
constexpr size_t kOffsetPacketDataOffset = 8;
constexpr size_t kOffsetPacketDataLength = 12;
constexpr size_t kPacketDataOffsetBase   = 8;

// Zero-fills `messageLength` bytes and writes the three leading fields every request
// shares. Returns false when the buffer cannot hold the message.
bool
WriteMessageStart(uint8_t * buffer, size_t capacity, size_t messageLength,
                  uint32_t messageType, uint32_t requestID)
{
    if (buffer == nullptr || capacity < messageLength || messageLength > UINT32_MAX) {
        return false;
    }
    memset(buffer, 0, messageLength);
    WriteLE32(buffer, kOffsetMessageType, messageType);
    WriteLE32(buffer, kOffsetMessageLength, static_cast<uint32_t>(messageLength));
    WriteLE32(buffer, kOffsetRequestID, requestID);
    return true;
}

// Writes a QUERY or SET: the shared header, the OID and an information buffer of
// `informationLength` bytes copied from `information`, or left zeroed when it is NULL
// (a QUERY's placeholder: the device ignores the content, only the size matters, see
// kFixedOIDPlaceholderBytes).
size_t
BuildQuerySet(uint8_t * buffer, size_t capacity, uint32_t requestID, uint32_t messageType,
              uint32_t oid, const uint8_t * information, uint32_t informationLength)
{
    const size_t messageLength = kQuerySetHeaderSize + informationLength;
    if (!WriteMessageStart(buffer, capacity, messageLength, messageType, requestID)) {
        return 0;
    }
    WriteLE32(buffer, kOffsetQuerySetOID, oid);
    WriteLE32(buffer, kOffsetQuerySetInformationLength, informationLength);
    // With no information buffer the offset stays 0 as well: some devices refuse a
    // message that points at a buffer of length zero.
    WriteLE32(buffer, kOffsetQuerySetInformationOffset,
              informationLength != 0 ? kQuerySetInlineInformationOffset : 0);
    if (information != nullptr && informationLength != 0) {
        memcpy(buffer + kQuerySetHeaderSize, information, informationLength);
    }
    return messageLength;
}

// Parses the header and checks that it is a completion of `requestType` with at least
// `minimumLength` bytes.
bool
ParseCompletionHeader(const uint8_t * buffer, size_t length, uint32_t requestType,
                      size_t minimumLength, Header * header)
{
    if (!ParseHeader(buffer, length, header)) {
        return false;
    }
    return header->messageType == (requestType | kMessageCompletionFlag)
        && header->messageLength >= minimumLength;
}
} // namespace

uint32_t
ReadLE32(const uint8_t * buffer, size_t offset)
{
    const uint8_t * p = buffer + offset;
    return static_cast<uint32_t>(p[0])
         | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16)
         | (static_cast<uint32_t>(p[3]) << 24);
}

void
WriteLE32(uint8_t * buffer, size_t offset, uint32_t value)
{
    uint8_t * p = buffer + offset;
    p[0] = static_cast<uint8_t>(value);
    p[1] = static_cast<uint8_t>(value >> 8);
    p[2] = static_cast<uint8_t>(value >> 16);
    p[3] = static_cast<uint8_t>(value >> 24);
}

// MARK: - Builders

size_t
BuildInitialize(uint8_t * buffer, size_t capacity, uint32_t requestID,
                uint32_t maxTransferSize)
{
    if (!WriteMessageStart(buffer, capacity, kInitializeSize, kMessageTypeInitialize, requestID)) {
        return 0;
    }
    WriteLE32(buffer, kOffsetInitMajorVersion, kMajorVersion);
    WriteLE32(buffer, kOffsetInitMinorVersion, kMinorVersion);
    WriteLE32(buffer, kOffsetInitMaxTransferSize, maxTransferSize);
    return kInitializeSize;
}

size_t
BuildQuery(uint8_t * buffer, size_t capacity, uint32_t requestID, uint32_t oid,
           uint32_t informationLength)
{
    return BuildQuerySet(buffer, capacity, requestID, kMessageTypeQuery, oid,
                         nullptr, informationLength);
}

size_t
BuildSet(uint8_t * buffer, size_t capacity, uint32_t requestID, uint32_t oid,
         const uint8_t * information, uint32_t informationLength)
{
    if (informationLength != 0 && information == nullptr) {
        return 0;
    }
    return BuildQuerySet(buffer, capacity, requestID, kMessageTypeSet, oid,
                         information, informationLength);
}

size_t
BuildSetPacketFilter(uint8_t * buffer, size_t capacity, uint32_t requestID, uint32_t filter)
{
    uint8_t value[4];
    WriteLE32(value, 0, filter);
    return BuildSet(buffer, capacity, requestID, kOIDGeneralCurrentPacketFilter,
                    value, sizeof(value));
}

size_t
BuildHalt(uint8_t * buffer, size_t capacity, uint32_t requestID)
{
    if (!WriteMessageStart(buffer, capacity, kHaltSize, kMessageTypeHalt, requestID)) {
        return 0;
    }
    return kHaltSize;
}

size_t
BuildReset(uint8_t * buffer, size_t capacity)
{
    // The third word is Reserved, which WriteMessageStart zero-fills.
    if (!WriteMessageStart(buffer, capacity, kResetSize, kMessageTypeReset, 0)) {
        return 0;
    }
    return kResetSize;
}

size_t
BuildKeepaliveComplete(uint8_t * buffer, size_t capacity, uint32_t requestID)
{
    if (!WriteMessageStart(buffer, capacity, kKeepaliveCompleteSize,
                           kMessageTypeKeepalive | kMessageCompletionFlag, requestID)) {
        return 0;
    }
    WriteLE32(buffer, kOffsetStatus, kStatusSuccess);
    return kKeepaliveCompleteSize;
}

// MARK: - Parsers

bool
ParseHeader(const uint8_t * buffer, size_t length, Header * header)
{
    if (buffer == nullptr || header == nullptr || length < kHeaderSize) {
        return false;
    }
    const uint32_t messageLength = ReadLE32(buffer, kOffsetMessageLength);
    // From here on, MessageLength decides which fields exist, so it must itself fit in
    // the bytes received. It may be shorter: devices can zero-pad the transfer.
    if (messageLength < kHeaderSize || messageLength > length) {
        return false;
    }
    header->messageType   = ReadLE32(buffer, kOffsetMessageType);
    header->messageLength = messageLength;
    header->requestID     = messageLength >= kOffsetRequestID + 4 ? ReadLE32(buffer, kOffsetRequestID) : 0;
    header->status        = messageLength >= kOffsetStatus + 4 ? ReadLE32(buffer, kOffsetStatus) : 0;
    return true;
}

bool
ParseInitializeComplete(const uint8_t * buffer, size_t length, InitializeResult * result)
{
    Header header;
    if (result == nullptr
        || !ParseCompletionHeader(buffer, length, kMessageTypeInitialize,
                                  kInitializeCompleteSize, &header)) {
        return false;
    }
    result->status                = header.status;
    result->majorVersion          = ReadLE32(buffer, kOffsetInitCMajorVersion);
    result->minorVersion          = ReadLE32(buffer, kOffsetInitCMinorVersion);
    result->deviceFlags           = ReadLE32(buffer, kOffsetInitCDeviceFlags);
    result->medium                = ReadLE32(buffer, kOffsetInitCMedium);
    result->maxPacketsPerTransfer = ReadLE32(buffer, kOffsetInitCMaxPacketsPerTransfer);
    result->maxTransferSize       = ReadLE32(buffer, kOffsetInitCMaxTransferSize);
    result->packetAlignmentFactor = ReadLE32(buffer, kOffsetInitCPacketAlignmentFactor);
    return true;
}

bool
ParseQueryComplete(const uint8_t * buffer, size_t length, uint32_t * status,
                   const uint8_t ** information, uint32_t * informationLength)
{
    Header header;
    if (status == nullptr || information == nullptr || informationLength == nullptr
        || !ParseCompletionHeader(buffer, length, kMessageTypeQuery,
                                  kQueryCompleteHeaderSize, &header)) {
        return false;
    }
    const uint32_t infoLength = ReadLE32(buffer, kOffsetQuerySetInformationLength);
    const uint32_t infoOffset = ReadLE32(buffer, kOffsetQuerySetInformationOffset);
    // InformationBufferOffset counts from byte 8 (see the namespace comment), so the
    // buffer has to fit in what follows that byte. Offset and length are compared
    // separately: their sum could overflow.
    const uint32_t available = header.messageLength - kOffsetRequestID;
    if (infoOffset > available || infoLength > available - infoOffset) {
        return false;
    }
    *status            = header.status;
    *information       = buffer + kOffsetRequestID + infoOffset;
    *informationLength = infoLength;
    return true;
}

bool
ParseSetComplete(const uint8_t * buffer, size_t length, uint32_t * status)
{
    Header header;
    if (status == nullptr
        || !ParseCompletionHeader(buffer, length, kMessageTypeSet, kSetCompleteSize, &header)) {
        return false;
    }
    *status = header.status;
    return true;
}

bool
ParseResetComplete(const uint8_t * buffer, size_t length, uint32_t * status,
                   uint32_t * addressingReset)
{
    Header header;
    if (status == nullptr || addressingReset == nullptr
        || !ParseCompletionHeader(buffer, length, kMessageTypeReset, kResetCompleteSize, &header)) {
        return false;
    }
    *status          = ReadLE32(buffer, kOffsetResetCStatus);
    *addressingReset = ReadLE32(buffer, kOffsetResetCAddressingReset);
    return true;
}

bool
ParseIndicateStatus(const uint8_t * buffer, size_t length, uint32_t * status)
{
    Header header;
    if (status == nullptr || !ParseHeader(buffer, length, &header)
        || header.messageType != kMessageTypeIndicateStatus
        || header.messageLength < kOffsetIndicateStatus + 4) {
        return false;
    }
    *status = ReadLE32(buffer, kOffsetIndicateStatus);
    return true;
}

// MARK: - Classification

const char *
StatusName(uint32_t status)
{
    switch (status) {
        case kStatusMediaConnect:    return "media connect";
        case kStatusMediaDisconnect: return "media disconnect";
        default:                     return "other";
    }
}

bool
IsResponseAvailable(const uint8_t * buffer, size_t length)
{
    return buffer != nullptr && length >= kResponseAvailableSize
        && ReadLE32(buffer, 0) == kNotificationResponseAvailable;
}

const char *
MessageTypeName(uint32_t messageType)
{
    switch (messageType) {
        case kMessageTypePacket:                                return "PACKET";
        case kMessageTypeInitialize:                            return "INITIALIZE";
        case kMessageTypeInitialize | kMessageCompletionFlag:   return "INITIALIZE_CMPLT";
        case kMessageTypeHalt:                                  return "HALT";
        case kMessageTypeQuery:                                 return "QUERY";
        case kMessageTypeQuery | kMessageCompletionFlag:        return "QUERY_CMPLT";
        case kMessageTypeSet:                                   return "SET";
        case kMessageTypeSet | kMessageCompletionFlag:          return "SET_CMPLT";
        case kMessageTypeReset:                                 return "RESET";
        case kMessageTypeReset | kMessageCompletionFlag:        return "RESET_CMPLT";
        case kMessageTypeIndicateStatus:                        return "INDICATE_STATUS";
        case kMessageTypeKeepalive:                             return "KEEPALIVE";
        case kMessageTypeKeepalive | kMessageCompletionFlag:    return "KEEPALIVE_CMPLT";
        default:                                                return "other";
    }
}

ResponseKind
ClassifyResponse(const uint8_t * buffer, size_t length, uint32_t expectedMessageType,
                 uint32_t expectedRequestID, Header * header)
{
    if (length < kHeaderSize) {
        return ResponseKind::NotReady;
    }
    if (!ParseHeader(buffer, length, header)) {
        return ResponseKind::Malformed;
    }
    if (expectedMessageType != 0
        && header->messageType == (expectedMessageType | kMessageCompletionFlag)) {
        if (expectedMessageType == kMessageTypeReset) {
            return ResponseKind::Completion; // RESET_CMPLT has no RequestID to compare
        }
        return header->requestID == expectedRequestID ? ResponseKind::Completion
                                                      : ResponseKind::CompletionOtherRequest;
    }
    if (header->messageType == kMessageTypeIndicateStatus) {
        return ResponseKind::IndicateStatus;
    }
    if (header->messageType == kMessageTypeKeepalive) {
        return ResponseKind::Keepalive;
    }
    return ResponseKind::Unexpected;
}

// MARK: - Data plane

size_t
BuildPacketHeader(uint8_t * buffer, size_t capacity, uint32_t frameLength)
{
    if (buffer == nullptr || capacity < kPacketHeaderSize
        || frameLength > UINT32_MAX - kPacketHeaderSize) {
        return 0;
    }
    memset(buffer, 0, kPacketHeaderSize);
    WriteLE32(buffer, kOffsetMessageType, kMessageTypePacket);
    WriteLE32(buffer, kOffsetMessageLength, static_cast<uint32_t>(kPacketHeaderSize) + frameLength);
    WriteLE32(buffer, kOffsetPacketDataOffset, kPacketInlineDataOffset);
    WriteLE32(buffer, kOffsetPacketDataLength, frameLength);
    return kPacketHeaderSize;
}

uint32_t
MaxFrameLength(size_t capacity, uint32_t deviceMaxTransferSize)
{
    if (capacity <= kPacketHeaderSize) {
        return 0;
    }
    size_t limit = capacity - kPacketHeaderSize - 1; // the header and one pad byte
    if (deviceMaxTransferSize > kPacketHeaderSize && deviceMaxTransferSize - kPacketHeaderSize < limit) {
        limit = deviceMaxTransferSize - kPacketHeaderSize;
    }
    return limit > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(limit);
}

size_t
FramePacket(uint8_t * buffer, size_t capacity, const uint8_t * frame, uint32_t frameLength,
            uint16_t bulkOutMaxPacketSize, bool * padded)
{
    if (padded != nullptr) {
        *padded = false;
    }
    if (frame == nullptr || frameLength == 0 || frameLength > MaxFrameLength(capacity, 0)) {
        return 0;
    }
    const size_t headerLength = BuildPacketHeader(buffer, capacity, frameLength);
    if (headerLength == 0) {
        return 0;
    }
    memcpy(buffer + headerLength, frame, frameLength);
    size_t total = headerLength + frameLength;
    if (bulkOutMaxPacketSize != 0 && total % bulkOutMaxPacketSize == 0) {
        buffer[total++] = 0;
        if (padded != nullptr) {
            *padded = true;
        }
    }
    return total;
}

BatchStep
NextPacket(const uint8_t * buffer, size_t length, size_t * cursor,
           const uint8_t ** frame, uint32_t * frameLength)
{
    if (buffer == nullptr || cursor == nullptr || frame == nullptr || frameLength == nullptr
        || *cursor > length) {
        return BatchStep::Malformed;
    }
    const size_t remaining = length - *cursor;
    if (remaining < kPacketHeaderSize) {
        return BatchStep::End; // nothing, or a padding tail shorter than a header
    }
    const uint8_t * message = buffer + *cursor;
    const uint32_t messageType = ReadLE32(message, kOffsetMessageType);
    if (messageType == 0) {
        return BatchStep::End; // zero padding up to the end of the transfer
    }
    const uint32_t messageLength = ReadLE32(message, kOffsetMessageLength);
    const uint32_t dataOffset    = ReadLE32(message, kOffsetPacketDataOffset);
    const uint32_t dataLength    = ReadLE32(message, kOffsetPacketDataLength);
    // The frame must lie inside the message, and the message inside the transfer.
    // Two-step checks so a hostile offset + length pair cannot wrap around.
    if (messageType != kMessageTypePacket
        || messageLength < kPacketHeaderSize || messageLength > remaining) {
        return BatchStep::Malformed;
    }
    const uint32_t available = messageLength - kPacketDataOffsetBase;
    if (dataOffset > available || dataLength > available - dataOffset) {
        return BatchStep::Malformed;
    }
    *frame       = message + kPacketDataOffsetBase + dataOffset;
    *frameLength = dataLength;
    *cursor     += messageLength;
    return BatchStep::Frame;
}

} // namespace RNDIS
