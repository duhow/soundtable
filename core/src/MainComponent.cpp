#include "MainComponent.h"

#include <algorithm>
#include <cmath>

#include "AudioEngine.h"

MainComponent::MainComponent(AudioEngine& audioEngine)
    : audioEngine_(audioEngine)
{
    setSize(1280, 720);

    // Example scene with a couple of modules and objects.
    rectai::Module osc1("osc1", rectai::ModuleKind::kOscillator);
    osc1.AddOutputPort("out", true);

    rectai::Module filter1("filter1", rectai::ModuleKind::kFilter);
    filter1.AddInputPort("in", true);
    filter1.AddOutputPort("out", true);

    (void)scene_.AddModule(osc1);
    (void)scene_.AddModule(filter1);

    rectai::Connection connection{
        .from_module_id = "osc1",
        .from_port_name = "out",
        .to_module_id = "filter1",
        .to_port_name = "in",
    };
    (void)scene_.AddConnection(connection);

    // Normalized positions on the table.
    scene_.UpsertObject(rectai::ObjectInstance(1, "osc1", 0.3F, 0.5F, 0.0F));
    scene_.UpsertObject(
        rectai::ObjectInstance(2, "filter1", 0.7F, 0.5F, 0.0F));

    // Periodically map scene state to audio parameters.
    startTimerHz(60);
}

void MainComponent::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colours::black);

    const auto bounds = getLocalBounds().toFloat();
    const auto centre = bounds.getCentre();

    // Draw fixed central circle.
    const float baseRadius = 6.0F;
    g.setColour(juce::Colours::white);
    g.fillEllipse(centre.x - baseRadius, centre.y - baseRadius,
                  baseRadius * 2.0F, baseRadius * 2.0F);

    // Draw animated BPM pulses as expanding, fading rings.
    for (const auto& pulse : pulses_) {
        const float t = juce::jlimit(0.0F, 1.0F, pulse.age);
        const float maxRadius = pulse.strong ? 90.0F : 70.0F;
        const float radius = baseRadius + t * maxRadius;
        const float alpha = 1.0F - t;
        const float thickness = pulse.strong ? 3.0F : 2.0F;

        g.setColour(juce::Colours::white.withAlpha(alpha));
        g.drawEllipse(centre.x - radius, centre.y - radius, radius * 2.0F,
                      radius * 2.0F, thickness);
    }

    // Draw connections as lines from centre to each object.
    g.setColour(juce::Colours::grey);
    const auto& objects = scene_.objects();
    for (const auto& [id, object] : objects) {
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        juce::Line<float> line(centre.x, centre.y, cx, cy);

        if (mutedObjects_.find(id) != mutedObjects_.end()) {
            const float dashLengths[] = {6.0F, 4.0F};
            g.drawDashedLine(line, dashLengths,
                             static_cast<int>(std::size(dashLengths)), 2.0F);
        } else {
            g.drawLine(line, 2.0F);
        }
    }

    // Draw objects as circles.
    const float radius = 30.0F;

    for (const auto& entry : objects) {
        const auto& object = entry.second;
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        g.setColour(juce::Colours::deepskyblue);
        g.fillEllipse(cx - radius, cy - radius, radius * 2.0F, radius * 2.0F);

        g.setColour(juce::Colours::white);
        g.setFont(16.0F);
        g.drawText(object.logical_id(),
                   juce::Rectangle<float>(cx - radius, cy - radius,
                                          radius * 2.0F, radius * 2.0F),
                   juce::Justification::centred, false);
    }

    // Title.
    g.setColour(juce::Colours::white);
    g.setFont(18.0F);
    g.drawText("RectaiTable - Example scene", 0.0F, 0.0F, bounds.getWidth(),
               30.0F, juce::Justification::centred);
}

void MainComponent::resized()
{
}

void MainComponent::mouseDown(const juce::MouseEvent& event)
{
    const auto bounds = getLocalBounds().toFloat();
    const float radius = 30.0F;

    draggedObjectId_ = 0;

    // First, try to select an object by clicking on its circle.
    for (const auto& [id, object] : scene_.objects()) {
        const auto cx = bounds.getX() + object.x() * bounds.getWidth();
        const auto cy = bounds.getY() + object.y() * bounds.getHeight();

        const auto dx = static_cast<float>(event.position.x) - cx;
        const auto dy = static_cast<float>(event.position.y) - cy;
        const auto distanceSquared = dx * dx + dy * dy;

        if (distanceSquared <= radius * radius) {
            draggedObjectId_ = id;
            break;
        }
    }

    // If no object was selected, check if the click is near a line
    // connecting an object to the centre to toggle mute.
    if (draggedObjectId_ == 0) {
        const auto centre = bounds.getCentre();
        const auto click = event.position;
        constexpr float maxDistance = 6.0F;
        const float maxDistanceSq = maxDistance * maxDistance;

        const auto& objects = scene_.objects();

        auto isPointNearSegment = [](juce::Point<float> p, juce::Point<float> a,
                                     juce::Point<float> b,
                                     const float maxDistSq) {
            const auto ab = b - a;
            const auto ap = p - a;
            const auto abLenSq = ab.x * ab.x + ab.y * ab.y;
            float t = 0.0F;
            if (abLenSq > 0.0F) {
                t = (ap.x * ab.x + ap.y * ab.y) / abLenSq;
                t = juce::jlimit(0.0F, 1.0F, t);
            }
            const auto closest = a + ab * t;
            const auto dx = p.x - closest.x;
            const auto dy = p.y - closest.y;
            const auto distSq = dx * dx + dy * dy;
            return distSq <= maxDistSq;
        };

        for (const auto& [id, object] : objects) {
            const auto cx = bounds.getX() + object.x() * bounds.getWidth();
            const auto cy = bounds.getY() + object.y() * bounds.getHeight();

            if (isPointNearSegment(click, centre, {cx, cy}, maxDistanceSq)) {
                const auto itMuted = mutedObjects_.find(id);
                const bool nowMuted = itMuted == mutedObjects_.end();
                if (nowMuted) {
                    mutedObjects_.insert(id);
                } else {
                    mutedObjects_.erase(itMuted);
                }

                repaint();
                break;
            }
        }
    }
}

void MainComponent::mouseDrag(const juce::MouseEvent& event)
{
    if (draggedObjectId_ == 0) {
        return;
    }

    const auto bounds = getLocalBounds().toFloat();

    const float normX = (static_cast<float>(event.position.x) - bounds.getX()) /
                        bounds.getWidth();
    const float normY = (static_cast<float>(event.position.y) - bounds.getY()) /
                        bounds.getHeight();

    auto objects = scene_.objects();
    const auto it = objects.find(draggedObjectId_);
    if (it == objects.end()) {
        return;
    }

    auto updated = it->second;
    updated.set_position(normX, normY);
    scene_.UpsertObject(updated);

    repaint();
}

void MainComponent::mouseUp(const juce::MouseEvent&)
{
    draggedObjectId_ = 0;
}

void MainComponent::timerCallback()
{
    // Simple mapping: use the x position of the object with logical_id "osc1"
    // to control the oscillator frequency.
    const auto& objects = scene_.objects();
    auto it = std::find_if(objects.begin(), objects.end(),
                           [](const auto& pair) {
                               return pair.second.logical_id() == "osc1";
                           });

    float targetLevel = 0.0F;

    if (it != objects.end()) {
        const auto x = juce::jlimit(0.0F, 1.0F, it->second.x());
        const bool isMuted =
            mutedObjects_.find(it->first) != mutedObjects_.end();

        // Map [0, 1] -> [200 Hz, 1000 Hz].
        const double frequency = 200.0 + static_cast<double>(x) * 800.0;
        audioEngine_.setFrequency(frequency);

        targetLevel = isMuted ? 0.0F : 0.1F;
    }

    audioEngine_.setLevel(targetLevel);

    // Update BPM pulse animation.
    const double fps = 60.0;  // Timer frequency.
    const double dt = 1.0 / fps;

    // Age existing pulses and remove the ones that have fully faded.
    constexpr double pulseLifetimeSeconds = 1.0;
    const float ageStep = static_cast<float>(dt / pulseLifetimeSeconds);

    for (auto& pulse : pulses_) {
        pulse.age += ageStep;
    }

    pulses_.erase(std::remove_if(pulses_.begin(), pulses_.end(),
                                 [](const Pulse& p) { return p.age >= 1.0F; }),
                  pulses_.end());

    // Spawn a new pulse on every beat; every 4th beat is stronger.
    const double bps = bpm_ / 60.0;  // beats per second
    beatPhase_ += bps / fps;
    if (beatPhase_ >= 1.0) {
        beatPhase_ -= 1.0;

        const bool strong = (beatIndex_ % 4 == 0);
        pulses_.push_back(Pulse{0.0F, strong});

        ++beatIndex_;
        if (beatIndex_ >= 4) {
            beatIndex_ = 0;
        }
    }

    repaint();
}
