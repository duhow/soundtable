#include "MainComponent.h"

#include <algorithm>

MainComponent::MainComponent()
{
    setSize(800, 600);

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
}

void MainComponent::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colours::black);

    const auto bounds = getLocalBounds().toFloat();
    const auto center_y = bounds.getCentreY();

    // Draw connections as lines between objects.
    g.setColour(juce::Colours::grey);
    const auto& objects = scene_.objects();
    for (const auto& connection : scene_.connections()) {
        const auto from_it = std::find_if(objects.begin(), objects.end(),
                                          [&connection](const auto& pair) {
                                              return pair.second.logical_id() ==
                                                     connection.from_module_id;
                                          });
        const auto to_it = std::find_if(objects.begin(), objects.end(),
                                        [&connection](const auto& pair) {
                                            return pair.second.logical_id() ==
                                                   connection.to_module_id;
                                        });
        if (from_it == objects.end() || to_it == objects.end()) {
            continue;
        }

        const auto from = from_it->second;
        const auto to = to_it->second;

        const auto from_x = bounds.getX() + from.x() * bounds.getWidth();
        const auto from_y = bounds.getY() + from.y() * bounds.getHeight();
        const auto to_x = bounds.getX() + to.x() * bounds.getWidth();
        const auto to_y = bounds.getY() + to.y() * bounds.getHeight();

        g.drawLine(from_x, from_y, to_x, to_y, 2.0F);
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
