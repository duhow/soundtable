#include "MainComponentHelpers.h"

#include <algorithm>
#include <cmath>

namespace soundtable::ui {

juce::Colour colourFromArgb(const std::uint32_t argb)
{
    // JUCE almacena internamente los colores como ARGB en un `uint32`,
    // por lo que podemos pasar directamente nuestro valor `0xAARRGGBB`
    // al constructor que acepta un entero en ese formato.
    return juce::Colour(static_cast<juce::uint32>(argb));
}

std::string makeConnectionKey(const soundtable::Connection& c)
{
    std::string key;
    key.reserve(c.from_module_id.size() + c.from_port_name.size() +
                c.to_module_id.size() + c.to_port_name.size() + 4U);
    key.append(c.from_module_id)
        .append(1U, ':')
        .append(c.from_port_name)
        .append(1U, '>')
        .append(c.to_module_id)
        .append(1U, ':')
        .append(c.to_port_name);
    return key;
}

std::string makeObjectPairKey(std::int64_t a, std::int64_t b)
{
    if (a > b) {
        std::swap(a, b);
    }
    std::string key;
    key.reserve(32U);
    key.append(std::to_string(a)).append(1U, '>').append(
        std::to_string(b));
    return key;
}

std::string makeModulePairKey(const std::string& fromId,
                              const std::string& toId)
{
    std::string key;
    key.reserve(fromId.size() + toId.size() + 1U);
    key.append(fromId).append(1U, '>').append(toId);
    return key;
}

std::string makeModulePairKey(const soundtable::Connection& conn)
{
    std::string key;
    key.reserve(conn.from_module_id.size() + conn.to_module_id.size() + 1U);
    key.append(conn.from_module_id).append(1U, '>').append(conn.to_module_id);
    return key;
}

bool isConnectionGeometricallyActive(const soundtable::ObjectInstance& fromObj,
                                     const soundtable::ObjectInstance& toObj)
{
    // ObjectInstance positions are expressed in a centre-origin table
    // coordinate system where the table centre is (0,0) and the table
    // border lies at radius 1. Work directly in that space so geometric
    // tests remain independent of the current component size.

    const float fromDx = fromObj.x();
    const float fromDy = fromObj.y();
    const float toDx = toObj.x();
    const float toDy = toObj.y();

    // Ignore degenerate cases extremely close to the table centre to
    // avoid noisy angle estimates.
    if (std::abs(fromDx) < 1.0e-4F && std::abs(fromDy) < 1.0e-4F) {
        return false;
    }
    if (std::abs(toDx) < 1.0e-4F && std::abs(toDy) < 1.0e-4F) {
        return false;
    }

    const float fromAngle = std::atan2(fromDy, fromDx);
    const float toAngle = std::atan2(toDy, toDx);

    float diff = toAngle - fromAngle;
    const float pi = juce::MathConstants<float>::pi;
    const float twoPi = juce::MathConstants<float>::twoPi;
    while (diff > pi) {
        diff -= twoPi;
    }
    while (diff < -pi) {
        diff += twoPi;
    }

    // Use a 120º cone (60º half-angle) around the centre so that
    // modules only form a dynamic connection when they are reasonably
    // aligned in direction, not across the whole quadrant.
    const float halfCone = juce::degreesToRadians(60.0F);
    return std::abs(diff) <= halfCone;
}

juce::File loadFile(const juce::String& relativePath)
{
    static constexpr const char* kReactableRoot = "com.reactable/";

    const juce::String fullRelativePath = juce::String(kReactableRoot) +
                                          relativePath;

    juce::File candidates[8];
    int candidateCount = 0;

    const juce::File cwd = juce::File::getCurrentWorkingDirectory();
    candidates[candidateCount++] = cwd.getChildFile(fullRelativePath);
    candidates[candidateCount++] =
        cwd.getChildFile("../" + fullRelativePath);
    candidates[candidateCount++] =
        cwd.getChildFile("../../" + fullRelativePath);

    const juce::File exeDir = juce::File::getSpecialLocation(
                                      juce::File::currentExecutableFile)
                                      .getParentDirectory();
    candidates[candidateCount++] = exeDir.getChildFile(fullRelativePath);
    candidates[candidateCount++] =
        exeDir.getChildFile("../" + fullRelativePath);
    candidates[candidateCount++] =
        exeDir.getChildFile("../../" + fullRelativePath);
    candidates[candidateCount++] =
        exeDir.getParentDirectory().getChildFile(fullRelativePath);
    candidates[candidateCount++] = exeDir.getParentDirectory().getChildFile(
        "../" + fullRelativePath);

    for (int i = 0; i < candidateCount; ++i) {
        const juce::File& f = candidates[i];
        if (f.existsAsFile()) {
            return f;
        }
    }

    return juce::File();
}

bool lineSegmentsIntersect(const juce::Point<float>& p1,
                          const juce::Point<float>& p2,
                          const juce::Point<float>& q1,
                          const juce::Point<float>& q2,
                          float threshold)
{
    // Check if segment p1->p2 is close to segment q1->q2 by computing
    // the minimum distance between the two segments.
    const auto segmentPointDistance = [](const juce::Point<float>& p,
                                         const juce::Point<float>& a,
                                         const juce::Point<float>& b) {
        const auto ap = p - a;
        const auto ab = b - a;
        const float abLengthSq = ab.x * ab.x + ab.y * ab.y;
        if (abLengthSq < 1e-6F) {
            return ap.getDistanceFrom(juce::Point<float>(0.0F, 0.0F));
        }
        const float t = juce::jlimit(
            0.0F, 1.0F,
            (ap.x * ab.x + ap.y * ab.y) / abLengthSq);
        const auto closest = a + ab * t;
        return p.getDistanceFrom(closest);
    };

    // Check both endpoints of each segment against the other segment.
    const float d1 = segmentPointDistance(p1, q1, q2);
    const float d2 = segmentPointDistance(p2, q1, q2);
    const float d3 = segmentPointDistance(q1, p1, p2);
    const float d4 = segmentPointDistance(q2, p1, p2);

    const float minDist = std::min({d1, d2, d3, d4});
    return minDist <= threshold;
}

bool generateConnectionFromModules(
    const AudioModule& moduleA,
    const AudioModule& moduleB,
    bool isHardlink,
    Connection& outConnection)
{
    std::string fromId;
    std::string toId;

    if (moduleA.CanConnectTo(moduleB)) {
      fromId = moduleA.id();
      toId = moduleB.id();
    } else if (moduleB.CanConnectTo(moduleA)) {
      fromId = moduleB.id();
      toId = moduleA.id();
    } else {
      return false;
    }

    Connection conn{fromId, "out", toId, "in", isHardlink};
    outConnection = conn;
    return true;
}


}  // namespace soundtable::ui
