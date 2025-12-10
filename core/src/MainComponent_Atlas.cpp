#include "MainComponent.h"

#include "MainComponentHelpers.h"

using rectai::ui::colourFromArgb;

bool MainComponent::loadAtlasResources()
{
    juce::File candidates[8];
    int candidateCount = 0;

    const juce::File cwd = juce::File::getCurrentWorkingDirectory();
    candidates[candidateCount++] =
        cwd.getChildFile("com.reactable/Resources");
    candidates[candidateCount++] =
        cwd.getChildFile("../com.reactable/Resources");
    candidates[candidateCount++] =
        cwd.getChildFile("../../com.reactable/Resources");

    const juce::File exeDir = juce::File::getSpecialLocation(
                                      juce::File::currentExecutableFile)
                                      .getParentDirectory();
    candidates[candidateCount++] =
        exeDir.getChildFile("com.reactable/Resources");
    candidates[candidateCount++] =
        exeDir.getChildFile("../com.reactable/Resources");
    candidates[candidateCount++] =
        exeDir.getChildFile("../../com.reactable/Resources");
    candidates[candidateCount++] = exeDir.getParentDirectory().getChildFile(
        "com.reactable/Resources");
    candidates[candidateCount++] =
        exeDir.getParentDirectory().getChildFile("../com.reactable/Resources");

    for (int i = 0; i < candidateCount; ++i) {
        const juce::File& baseDir = candidates[i];
        const juce::File pngFile = baseDir.getChildFile("atlas_2048.png");
        const juce::File xmlFile = baseDir.getChildFile("atlas_2048.xml");

        if (!pngFile.existsAsFile() || !xmlFile.existsAsFile()) {
            continue;
        }

        juce::Image image = juce::ImageFileFormat::loadFrom(pngFile);
        if (!image.isValid()) {
            continue;
        }

        juce::XmlDocument doc(xmlFile);
        std::unique_ptr<juce::XmlElement> root(doc.getDocumentElement());
        if (root == nullptr || !root->hasTagName("atlas")) {
            continue;
        }

        std::unordered_map<std::string, AtlasSprite> sprites;
        forEachXmlChildElementWithTagName(*root, sprite, "sprite")
        {
            const auto fullName = sprite->getStringAttribute("name");
            const int x = sprite->getIntAttribute("x");
            const int y = sprite->getIntAttribute("y");
            const int w = sprite->getIntAttribute("width");
            const int h = sprite->getIntAttribute("height");

            if (w <= 0 || h <= 0) {
                continue;
            }

            std::string key = fullName.toStdString();
            const auto slashPos = key.find_last_of('/');
            if (slashPos != std::string::npos && slashPos + 1U < key.size()) {
                key = key.substr(slashPos + 1U);
            }

            sprites[key] = AtlasSprite{juce::Rectangle<int>(x, y, w, h)};
        }

        if (!sprites.empty()) {
            atlasImage_ = std::move(image);
            atlasSprites_ = std::move(sprites);
            atlasLoaded_ = true;
            return true;
        }
    }

    atlasLoaded_ = false;
    atlasImage_ = juce::Image();
    atlasSprites_.clear();
    return false;
}
