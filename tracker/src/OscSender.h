#pragma once

#include <cstdint>
#include <string>

// Minimal OSC sender for the rectai-tracker.
// This is a very small, custom implementation that only supports
// the subset of OSC we need for now:
//   - Address patterns as strings.
//   - Arguments: int32, float32, string.
// Messages are sent over UDP to the configured host:port.
class OscSender {
public:
    OscSender(const std::string& host, std::uint16_t port);
    ~OscSender();

    [[nodiscard]] bool isOk() const { return socketFd_ >= 0; }

    // Sends a /rectai/object message with the given tracking id,
    // logical id, normalised position and angle in DEGREES
    // (range [0, 360]). The core converts this to radians when
    // populating ObjectInstance.
    bool sendObject(std::int32_t trackingId,
                    const std::string& logicalId,
                    float x,
                    float y,
                    float angleDegrees);

    bool sendRemove(std::int32_t trackingId);

private:
    bool sendMessage(const std::string& address,
                     const std::string& typeTags,
                     const std::string& logicalId,
                     std::int32_t trackingId,
                     float x,
                     float y,
                     float angleDegrees);

    bool sendMessage(const std::string& address,
                     const std::string& typeTags,
                     std::int32_t trackingId);

    static void appendPaddedString(const std::string& value,
                                   std::string& buffer);
    static void appendInt32(std::int32_t value, std::string& buffer);
    static void appendFloat32(float value, std::string& buffer);

    int socketFd_{-1};
};
