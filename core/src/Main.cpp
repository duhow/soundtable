#include <juce_gui_extra/juce_gui_extra.h>

#include "MainComponent.h"

class MainWindow : public juce::DocumentWindow {
public:
    explicit MainWindow(juce::String name)
        : juce::DocumentWindow(name,
                               juce::Colours::darkgrey,
                               juce::DocumentWindow::allButtons)
    {
        setUsingNativeTitleBar(true);
        setResizable(true, true);
        setContentOwned(new MainComponent(), true);
        centreWithSize(getWidth(), getHeight());
        setVisible(true);
    }

    void closeButtonPressed() override
    {
        juce::JUCEApplicationBase::quit();
    }

private:
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainWindow)
};

class RectaiApplication : public juce::JUCEApplication {
public:
    RectaiApplication() = default;

    const juce::String getApplicationName() override { return "RectaiTable"; }
    const juce::String getApplicationVersion() override { return "0.1.0"; }

    void initialise(const juce::String&) override
    {
        mainWindow_ = std::make_unique<MainWindow>(getApplicationName());
    }

    void shutdown() override
    {
        mainWindow_.reset();
    }

private:
    std::unique_ptr<MainWindow> mainWindow_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(RectaiApplication)
};

START_JUCE_APPLICATION(RectaiApplication)
