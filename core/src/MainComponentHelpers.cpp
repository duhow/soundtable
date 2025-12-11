#include "MainComponentHelpers.h"

#include <algorithm>
#include <cmath>

namespace rectai::ui {

juce::Colour colourFromArgb(const std::uint32_t argb)
{
    const auto alpha = static_cast<juce::uint8>((argb >> 24U) & 0xFFU);
    const auto red = static_cast<juce::uint8>((argb >> 16U) & 0xFFU);
    const auto green = static_cast<juce::uint8>((argb >> 8U) & 0xFFU);
    const auto blue = static_cast<juce::uint8>(argb & 0xFFU);
    // juce::Colour stores colours internally as ARGB; this constructor
    // expects components in that order.
    return juce::Colour(alpha, red, green, blue);
}

std::string makeConnectionKey(const rectai::Connection& c)
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

bool isConnectionGeometricallyActive(const rectai::ObjectInstance& fromObj,
                                     const rectai::ObjectInstance& toObj)
{
    constexpr float centreX = 0.5F;
    constexpr float centreY = 0.5F;

    const float fromDx = fromObj.x() - centreX;
    const float fromDy = fromObj.y() - centreY;
    const float toDx = toObj.x() - centreX;
    const float toDy = toObj.y() - centreY;

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

    const float halfCone = juce::MathConstants<float>::pi / 3.0F;  // 120º cone
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

}  // namespace rectai::ui
