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

}  // namespace rectai::ui
