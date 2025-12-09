#include "MainComponent.h"

MainComponent::MainComponent()
{
    setSize(800, 600);
}

void MainComponent::paint(juce::Graphics& g)
{
    g.fillAll(juce::Colours::black);
    g.setColour(juce::Colours::white);
    g.setFont(24.0f);
    g.drawText("RectaiTable – Core (JUCE)", getLocalBounds(), juce::Justification::centred, true);
}

void MainComponent::resized()
{
}
