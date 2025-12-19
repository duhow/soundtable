#include "OscSender.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <cstring>
#include <iostream>

namespace {
// Helper to build a sockaddr_in from host and port (IPv4, localhost-focused).
[[nodiscard]] sockaddr_in makeAddress(const std::string& host,
                                      std::uint16_t port)
{
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);

    if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
        // Fallback to 127.0.0.1 if parsing fails.
        std::cerr << "[rectai-tracker] Invalid OSC host '" << host
                  << "', falling back to 127.0.0.1" << std::endl;
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
    }

    return addr;
}
}  // namespace

OscSender::OscSender(const std::string& host, const std::uint16_t port)
{
    socketFd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socketFd_ < 0) {
        std::cerr << "[rectai-tracker] Failed to create OSC UDP socket" << std::endl;
        return;
    }

    const auto addr = makeAddress(host, port);
    if (::connect(socketFd_, reinterpret_cast<const sockaddr*>(&addr),
                  sizeof(addr)) < 0) {
        std::cerr << "[rectai-tracker] Failed to connect OSC UDP socket to "
                  << host << ":" << port << std::endl;
        ::close(socketFd_);
        socketFd_ = -1;
        return;
    }

    std::cout << "[rectai-tracker] OSC sender connected to " << host << ":"
              << port << std::endl;
}

OscSender::~OscSender()
{
    if (socketFd_ >= 0) {
        ::close(socketFd_);
        socketFd_ = -1;
    }
}

bool OscSender::sendObject(const std::int32_t trackingId,
                           const std::string& logicalId,
                           const float x,
                           const float y,
                           const float angleDegrees)
{
    if (!isOk()) {
        return false;
    }

    // Address: /rectai/object
    // Type tags: ",isfff" (int32, string, float, float, float)
    const auto message = buildRectaiObject(trackingId, logicalId, x, y,
                                           angleDegrees);
    return send(message);
}

bool OscSender::sendRemove(const std::int32_t trackingId)
{
    if (!isOk()) {
        return false;
    }

    // Address: /rectai/remove
    // Type tags: ",i" (int32)
    const auto message = buildRectaiRemove(trackingId);
    return send(message);
}

bool OscSender::sendTuio2DobjSet(const std::int32_t sessionId,
                                 const std::int32_t symbolId,
                                 const float x,
                                 const float y,
                                 const float angleRadians,
                                 const float vx,
                                 const float vy,
                                 const float angularVelocity)
{
    if (!isOk()) {
        return false;
    }

    const auto message = buildTuio2DobjSet(sessionId, symbolId, x, y,
                                           angleRadians, vx, vy,
                                           angularVelocity);
    return send(message);
}

bool OscSender::sendTuio2DobjAlive(
    const std::vector<std::int32_t>& sessionIds)
{
    if (!isOk()) {
        return false;
    }

    const auto message = buildTuio2DobjAlive(sessionIds);
    return send(message);
}

bool OscSender::sendTuio2DobjFseq(const std::int32_t frameSeq)
{
    if (!isOk()) {
        return false;
    }

    const auto message = buildTuio2DobjFseq(frameSeq);
    return send(message);
}

bool OscSender::sendHelloTuio11(const std::string& trackerId,
                                const std::string& trackerVersion)
{
    if (!isOk()) {
        return false;
    }

    const auto message = buildHelloTuio11(trackerId, trackerVersion);
    return send(message);
}

OscSender::Message OscSender::buildRectaiObject(
    const std::int32_t trackingId,
    const std::string& logicalId,
    const float x,
    const float y,
    const float angleDegrees)
{
    std::string buffer;
    buffer.reserve(128);

    appendPaddedString("/rectai/object", buffer);
    appendPaddedString(",isfff", buffer);

    appendInt32(trackingId, buffer);
    appendPaddedString(logicalId, buffer);
    appendFloat32(x, buffer);
    appendFloat32(y, buffer);
    appendFloat32(angleDegrees, buffer);

    return buffer;
}

OscSender::Message OscSender::buildRectaiRemove(
    const std::int32_t trackingId)
{
    std::string buffer;
    buffer.reserve(64);

    appendPaddedString("/rectai/remove", buffer);
    appendPaddedString(",i", buffer);
    appendInt32(trackingId, buffer);

    return buffer;
}

OscSender::Message OscSender::buildTuio2DobjSet(
    const std::int32_t sessionId,
    const std::int32_t symbolId,
    const float x,
    const float y,
    const float angleRadians,
    const float vx,
    const float vy,
    const float angularVelocity)
{
    // Address: /tuio/2Dobj
    // Type tags: ",siiffffffff" (string, int32, int32,
    //                             8 floats: x, y, a, X, Y, A, m, r)
    // We use:
    //   - "set" as the command string.
    //   - sessionId, symbolId
    //   - x, y, angleRadians
    //   - X, Y, A taken from velocity arguments (vx, vy, angularVelocity)
    //   - m, r set to 0.0f for now.
    std::string buffer;
    buffer.reserve(128);

    appendPaddedString("/tuio/2Dobj", buffer);
    appendPaddedString(",siiffffffff", buffer);

    appendPaddedString("set", buffer);
    appendInt32(sessionId, buffer);
    appendInt32(symbolId, buffer);
    appendFloat32(x, buffer);
    appendFloat32(y, buffer);
    appendFloat32(angleRadians, buffer);
    appendFloat32(vx, buffer);            // X (linear velocity X)
    appendFloat32(vy, buffer);            // Y (linear velocity Y)
    appendFloat32(angularVelocity, buffer);  // A (angular velocity)
    appendFloat32(0.0F, buffer);          // m (motion speed, unused)
    appendFloat32(0.0F, buffer);          // r (rotation accel, unused)

    return buffer;
}

OscSender::Message OscSender::buildTuio2DobjAlive(
    const std::vector<std::int32_t>& sessionIds)
{
    std::string buffer;
    buffer.reserve(64 + static_cast<std::size_t>(sessionIds.size()) * 8U);

    appendPaddedString("/tuio/2Dobj", buffer);

    // One string ("alive") plus N int32 values.
    std::string typeTags = ",s";
    typeTags.reserve(2 + sessionIds.size());
    for (std::size_t i = 0; i < sessionIds.size(); ++i) {
        typeTags.push_back('i');
    }
    appendPaddedString(typeTags, buffer);

    appendPaddedString("alive", buffer);
    for (const auto id : sessionIds) {
        appendInt32(id, buffer);
    }

    return buffer;
}

OscSender::Message OscSender::buildTuio2DobjFseq(
    const std::int32_t frameSeq)
{
    std::string buffer;
    buffer.reserve(64);

    appendPaddedString("/tuio/2Dobj", buffer);
    appendPaddedString(",si", buffer);

    appendPaddedString("fseq", buffer);
    appendInt32(frameSeq, buffer);

    return buffer;
}

OscSender::Message OscSender::buildHelloTuio11(
    const std::string& trackerId,
    const std::string& trackerVersion)
{
    // Address: /tuio/hello
    // Type tags: ",sss" (string trackerId, string tuioVersion,
    //                       string trackerVersion)
    constexpr const char* kAddress = "/tuio/hello";
    constexpr const char* kTypeTags = ",sss";
    constexpr const char* kTuioVersion = "1.1";

    std::string buffer;
    buffer.reserve(128);

    appendPaddedString(kAddress, buffer);
    appendPaddedString(kTypeTags, buffer);

    appendPaddedString(trackerId, buffer);
    appendPaddedString(kTuioVersion, buffer);
    appendPaddedString(trackerVersion, buffer);

    return buffer;
}

bool OscSender::send(const Message& message)
{
    if (!isOk()) {
        return false;
    }

    const auto sent = ::send(socketFd_, message.data(), message.size(), 0);
    return sent == static_cast<ssize_t>(message.size());
}

bool OscSender::sendBundle(const std::vector<Message>& messages)
{
    if (!isOk()) {
        return false;
    }

    if (messages.empty()) {
        return true;
    }

    if (messages.size() == 1U) {
        return send(messages.front());
    }

    // OSC bundle encoding:
    //   "#bundle" + NUL padding to 8 bytes,
    //   8-byte timetag (we use 0 == immediate),
    //   then for each element: int32 size (network order) + contents.
    std::string bundle;

    // Approximate reserve: header + each message plus its size prefix.
    std::size_t totalSize = 16;  // address + timetag
    for (const auto& msg : messages) {
        totalSize += 4U + msg.size();
    }
    bundle.reserve(totalSize);

    appendPaddedString("#bundle", bundle);

    // Timetag: 8 bytes. We use 0 for "immediately".
    bundle.append(8, '\0');

    for (const auto& msg : messages) {
        const auto size = static_cast<std::uint32_t>(msg.size());
        const std::uint32_t networkSize = htonl(size);

        char sizeBytes[sizeof(networkSize)];
        std::memcpy(sizeBytes, &networkSize, sizeof(networkSize));
        bundle.append(sizeBytes, sizeBytes + sizeof(networkSize));

        bundle.append(msg.data(), msg.data() + msg.size());
    }

    const auto sent = ::send(socketFd_, bundle.data(), bundle.size(), 0);
    return sent == static_cast<ssize_t>(bundle.size());
}

void OscSender::appendPaddedString(const std::string& value,
                                   std::string& buffer)
{
    // OSC strings are null-terminated and padded to 4-byte boundaries.
    const auto startSize = buffer.size();
    buffer.append(value);
    buffer.push_back('\0');

    const auto len = buffer.size() - startSize;
    const auto padding = (4 - (len % 4)) % 4;
    buffer.append(padding, '\0');
}

void OscSender::appendInt32(const std::int32_t value, std::string& buffer)
{
    std::uint32_t networkValue =
        htonl(static_cast<std::uint32_t>(value));

    char bytes[sizeof(networkValue)];
    std::memcpy(bytes, &networkValue, sizeof(networkValue));
    buffer.append(bytes, bytes + sizeof(networkValue));
}

void OscSender::appendFloat32(const float value, std::string& buffer)
{
    static_assert(sizeof(float) == sizeof(std::uint32_t),
                  "Unexpected float size");

    std::uint32_t asInt{};
    std::memcpy(&asInt, &value, sizeof(float));

    std::uint32_t networkValue = htonl(asInt);
    char bytes[sizeof(networkValue)];
    std::memcpy(bytes, &networkValue, sizeof(networkValue));
    buffer.append(bytes, bytes + sizeof(networkValue));
}
