#include <juce_gui_extra/juce_gui_extra.h>

#include "AudioEngine.h"
#include "MainComponent.h"
#include "RectaiLookAndFeel.h"
#include "BinaryData.h"

class MainWindow : public juce::DocumentWindow {
public:
    MainWindow(juce::String name, AudioEngine& audioEngine,
               juce::String initialSessionPath)
        : juce::DocumentWindow(name,
                               juce::Colours::darkgrey,
                               juce::DocumentWindow::allButtons)
    {
        setUsingNativeTitleBar(true);
        setResizable(true, true);
        {
            auto iconImage = juce::ImageFileFormat::loadFrom(
                BinaryData::reactablelogo_png,
                BinaryData::reactablelogo_pngSize);
            if (iconImage.isValid()) {
                setIcon(iconImage);
            }
        }
        setContentOwned(
            new MainComponent(audioEngine,
                               std::move(initialSessionPath)),
            true);
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

    void initialise(const juce::String& commandLineParameters) override
    {
        // Install a custom LookAndFeel that, when a bundled font is
        // present, uses a single embedded typeface instead of
        // querying all system fonts via fontconfig.
        juce::LookAndFeel::setDefaultLookAndFeel(&lookAndFeel_);

        // Parse command-line arguments to extract an optional
        // session file path. Any token starting with '-' is
        // currently treated as a (reserved) option and ignored
        // here, while the last positional argument is interpreted
        // as the candidate session file.
        juce::StringArray tokens;
        tokens.addTokens(commandLineParameters, true);
        tokens.trim();
        tokens.removeEmptyStrings();

        juce::StringArray positional;
        for (const auto& token : tokens) {
            if (token.startsWithChar('-')) {
                // Reserved for future CLI options such as --help or
                // --version.
                continue;
            }
            positional.add(token);
        }

        juce::String initialSessionPath;
        if (positional.size() > 0) {
            initialSessionPath = positional[positional.size() - 1];

            // Normalise simple quoted paths such as "Reactive 2.rtz" or
            // 'Reactive 2.rtz' that may appear as a single token on some
            // platforms. If the argument starts and ends with the same quote
            // character, strip the outer quotes so that juce::File sees the
            // actual filesystem path.
            if (initialSessionPath.length() >= 2) {
                const juce::juce_wchar first = initialSessionPath[0];
                const juce::juce_wchar last =
                    initialSessionPath[initialSessionPath.length() - 1];
                const bool isDoubleQuoted =
                    (first == '"' && last == '"');
                const bool isSingleQuoted =
                    (first == '\'' && last == '\'');
                if (isDoubleQuoted || isSingleQuoted) {
                    initialSessionPath = initialSessionPath.substring(
                        1, initialSessionPath.length() - 1);
                }
            }
        }

        mainWindow_ = std::make_unique<MainWindow>(
            getApplicationName(), audioEngine_, initialSessionPath);
    }

    void shutdown() override
    {
        juce::LookAndFeel::setDefaultLookAndFeel(nullptr);
        mainWindow_.reset();
    }

private:
    RectaiLookAndFeel lookAndFeel_;
    std::unique_ptr<MainWindow> mainWindow_;
    AudioEngine audioEngine_;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(RectaiApplication)
};

START_JUCE_APPLICATION(RectaiApplication)
