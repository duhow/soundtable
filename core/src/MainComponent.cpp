#include "MainComponent.h"

MainComponent::MainComponent()
{
    setSize(800, 600);

    // Escena de ejemplo con algunos modulos y objetos.
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

    // Posiciones normalizadas sobre la mesa.
    scene_.UpsertObject(rectai::ObjectInstance(1, "osc1", 0.3F, 0.5F, 0.0F));
    scene_.UpsertObject(
        rectai::ObjectInstance(2, "filter1", 0.7F, 0.5F, 0.0F));
}

void MainComponent::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colours::black);

    const auto bounds = getLocalBounds().toFloat();
    const auto center_y = bounds.getCentreY();

    // Dibujar conexiones como lineas entre objetos.
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

    // Dibujar objetos como crculos.
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

    // Ttulo.
    g.setColour(juce::Colours::white);
    g.setFont(18.0F);
    g.drawText("RectaiTable - Escena de ejemplo", 0.0F, 0.0F, bounds.getWidth(),
               30.0F, juce::Justification::centred);
}

void MainComponent::resized()
{
}
