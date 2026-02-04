#include "SoundtableLookAndFeel.h"
#include "ResourceManager.h"

namespace {

// Try to locate an optional bundled TTF that the user can drop
// into the project without changing code. We intentionally do not
// ship any third-party font in the repo; this loader is purely
// best-effort.
[[nodiscard]] juce::File findEmbeddedFontFile()
{
    const juce::File candidates[] = {
        // Preferred location: user resource directory managed by
        // ResourceManager, typically
        // ~/.local/share/soundtable/Resources/default.ttf.
        soundtable::ResourceManager::getInstance()
            .getResourceFile("Resources/default.ttf"),

        // Development-time fallbacks next to the repository or binary.
        juce::File::getSpecialLocation(
            juce::File::currentApplicationFile)
            .getParentDirectory()
            .getParentDirectory()
            .getChildFile("resources/default.ttf"),
    };

    for (const auto& file : candidates) {
        if (file.existsAsFile()) {
            return file;
        }
    }

    return {};
}

}  // namespace

SoundtableLookAndFeel::SoundtableLookAndFeel()
{
    const auto fontFile = findEmbeddedFontFile();
    if (!fontFile.existsAsFile()) {
        // No bundled font found: fall back to a well-known
        // sans-serif family that is typically available on most
        // Linux distributions. Fontconfig will resolve this name
        // using its caches without needing to scan every file in
        // /usr/share/fonts.
        setDefaultSansSerifTypefaceName("DejaVu Sans");
        return;
    }

    juce::FileInputStream input(fontFile);
    if (!input.openedOk()) {
        return;
    }

    juce::MemoryBlock data;
    if (!input.readIntoMemoryBlock(data)) {
        return;
    }

    if (data.getSize() == 0) {
        return;
    }

    embeddedTypeface_ = juce::Typeface::createSystemTypefaceFor(
        data.getData(), static_cast<size_t>(data.getSize()));

    if (embeddedTypeface_ != nullptr) {
        // Force JUCE to use this typeface as the default sans serif
        // font, avoiding a full fontconfig scan over the system
        // font directories.
        setDefaultSansSerifTypeface(embeddedTypeface_);
    } else {
        // If the embedded font could not be created for any reason,
        // still provide a deterministic sans-serif fallback.
        setDefaultSansSerifTypefaceName("DejaVu Sans");
    }
}
