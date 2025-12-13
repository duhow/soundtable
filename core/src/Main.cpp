#include <juce_gui_extra/juce_gui_extra.h>

#include "AudioEngine.h"
#include "MainComponent.h"

class MainWindow : public juce::DocumentWindow {
public:
    MainWindow(juce::String name, AudioEngine& audioEngine)
        : juce::DocumentWindow(name,
                               juce::Colours::darkgrey,
                               juce::DocumentWindow::allButtons)
    {
        setUsingNativeTitleBar(true);
        setResizable(true, true);
        setContentOwned(new MainComponent(audioEngine), true);
        centreWithSize(getWidth(), getHeight());
        setVisible(true);
    }

    void closeButtonPressed() override
    {
        juce::JUCEApplicationBase::quit();
    }

    bool keyPressed(const juce::KeyPress& key) override
    {
        // Escape closes the application.
        if (key.getKeyCode() == juce::KeyPress::escapeKey) {
            juce::JUCEApplicationBase::quit();
            return true;
        }

        const bool isAltEnter =
            key.getKeyCode() == juce::KeyPress::returnKey &&
            key.getModifiers().isAltDown();

        const bool isF11 =
            key.getKeyCode() == juce::KeyPress::F11Key;

        if (isAltEnter || isF11) {
            toggleFullscreen();
            return true;
        }

        return juce::DocumentWindow::keyPressed(key);
    }

private:
    void toggleFullscreen()
    {
        auto* peer = getPeer();
        if (peer == nullptr) {
            return;
        }

        if (!isInKioskMode_) {
            // Enter borderless fullscreen: remember current bounds and
            // switch to a non-native title bar so we control decorations.
            normalBounds_ = getBounds();
            setUsingNativeTitleBar(false);
            setTitleBarHeight(0);
            peer->setFullScreen(true);
            isInKioskMode_ = true;
        } else {
            // Leave fullscreen and restore previous window bounds and title.
            peer->setFullScreen(false);
            setTitleBarHeight(24);
            setUsingNativeTitleBar(true);
            if (!normalBounds_.isEmpty()) {
                setBounds(normalBounds_);
            }
            isInKioskMode_ = false;
        }
    }

    bool isInKioskMode_{false};
    juce::Rectangle<int> normalBounds_{};
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainWindow)
};

class RectaiApplication : public juce::JUCEApplication {
public:
    RectaiApplication() = default;

    const juce::String getApplicationName() override { return "RectaiTable"; }
    const juce::String getApplicationVersion() override { return "0.1.0"; }

    void initialise(const juce::String&) override
    {
        mainWindow_ =
            std::make_unique<MainWindow>(getApplicationName(), audioEngine_);
    }

    void shutdown() override
    {
        mainWindow_.reset();
    }

private:
    std::unique_ptr<MainWindow> mainWindow_;
    AudioEngine audioEngine_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(RectaiApplication)
};

START_JUCE_APPLICATION(RectaiApplication)
