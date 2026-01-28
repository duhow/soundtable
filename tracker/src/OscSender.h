#pragma once

#include <cstdint>
#include <string>
#include <vector>

// Minimal OSC sender for the soundtable-tracker.
// This is a very small, custom implementation that only supports
// the subset of OSC we need for now:
//   - Address patterns as strings.
//   - Arguments: int32, float32, string.
// Messages are sent over UDP to the configured host:port.
class OscSender {
public:
    OscSender(const std::string& host, std::uint16_t port);
    ~OscSender();

    using Message = std::string;

    [[nodiscard]] bool isOk() const { return socketFd_ >= 0; }

    // Low-level helpers to build OSC-encoded messages without sending them.
    // These are useful when batching multiple messages into a single
    // OSC bundle.
    [[nodiscard]] Message buildSoundtableObject(std::int32_t trackingId,
                                            const std::string& logicalId,
                                            float x,
                                            float y,
                                            float angleDegrees);

    [[nodiscard]] Message buildSoundtableRemove(std::int32_t trackingId);

    [[nodiscard]] Message buildTuio2DobjSet(std::int32_t sessionId,
                                            std::int32_t symbolId,
                                            float x,
                                            float y,
                                            float angleRadians,
                                            float vx = 0.0F,
                                            float vy = 0.0F,
                                            float angularVelocity = 0.0F);

    [[nodiscard]] Message buildTuio2DobjAlive(const std::vector<std::int32_t>& sessionIds);

    [[nodiscard]] Message buildTuio2DobjFseq(std::int32_t frameSeq);

    [[nodiscard]] Message buildHelloTuio11(const std::string& trackerId,
                                           const std::string& trackerVersion);

    // Send a single OSC message buffer as-is.
    bool send(const Message& message);

    // Send a group of messages batched into a single OSC bundle when
    // there are two or more messages. If there is only one message,
    // this falls back to sending it as a plain OSC message.
    bool sendBundle(const std::vector<Message>& messages);

    // Sends a /soundtable/object message with the given tracking id,
    // logical id, normalised position and angle in DEGREES
    // (range [0, 360]). The core converts this to radians when
    // populating ObjectInstance.
    bool sendObject(std::int32_t trackingId,
                    const std::string& logicalId,
                    float x,
                    float y,
                    float angleDegrees);

    bool sendRemove(std::int32_t trackingId);

    // Sends a basic TUIO 1.1 2Dobj "set" message:
    //   /tuio/2Dobj set s_id i_id x y a X Y A m r
    // We populate s_id, i_id, x, y, a and the velocity
    // components X, Y, A (linear X/Y and angular velocity).
    // The motion speed (m) and rotation acceleration (r)
    // are currently left at zero.
    bool sendTuio2DobjSet(std::int32_t sessionId,
                          std::int32_t symbolId,
                          float x,
                          float y,
                          float angleRadians,
                          float vx = 0.0F,
                          float vy = 0.0F,
                          float angularVelocity = 0.0F);

    // Sends a TUIO 1.1 2Dobj "alive" message listing the currently
    // active session ids.
    bool sendTuio2DobjAlive(const std::vector<std::int32_t>& sessionIds);

    // Sends a TUIO 1.1 2Dobj "fseq" message with the given frame
    // sequence number.
    bool sendTuio2DobjFseq(std::int32_t frameSeq);

    // Sends a /tuio/hello negotiation message advertising TUIO 1.1
    // support from soundtable-tracker. This is a small, custom message
    // understood by soundtable-core and is not part of the standard
    // Reactable protocol.
    bool sendHelloTuio11(const std::string& trackerId,
                         const std::string& trackerVersion);

private:
    static void appendPaddedString(const std::string& value,
                                   std::string& buffer);
    static void appendInt32(std::int32_t value, std::string& buffer);
    static void appendFloat32(float value, std::string& buffer);

    int socketFd_{-1};
};
