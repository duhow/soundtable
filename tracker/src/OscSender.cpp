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
    return sendMessage("/rectai/object", ",isfff", logicalId, trackingId, x,
                       y, angleDegrees);
}

bool OscSender::sendRemove(const std::int32_t trackingId)
{
    if (!isOk()) {
        return false;
    }

    // Address: /rectai/remove
    // Type tags: ",i" (int32)
    return sendMessage("/rectai/remove", ",i", trackingId);
}

bool OscSender::sendMessage(const std::string& address,
                            const std::string& typeTags,
                            const std::string& logicalId,
                            const std::int32_t trackingId,
                            const float x,
                            const float y,
                            const float angleDegrees)
{
    std::string buffer;
    buffer.reserve(128);

    appendPaddedString(address, buffer);
    appendPaddedString(typeTags, buffer);

    appendInt32(trackingId, buffer);
    appendPaddedString(logicalId, buffer);
    appendFloat32(x, buffer);
    appendFloat32(y, buffer);
    appendFloat32(angleDegrees, buffer);

    const auto sent = ::send(socketFd_, buffer.data(), buffer.size(), 0);
    return sent == static_cast<ssize_t>(buffer.size());
}

bool OscSender::sendMessage(const std::string& address,
                            const std::string& typeTags,
                            const std::int32_t trackingId)
{
    std::string buffer;
    buffer.reserve(64);

    appendPaddedString(address, buffer);
    appendPaddedString(typeTags, buffer);
    appendInt32(trackingId, buffer);

    const auto sent = ::send(socketFd_, buffer.data(), buffer.size(), 0);
    return sent == static_cast<ssize_t>(buffer.size());
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
