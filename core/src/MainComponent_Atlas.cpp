#include "MainComponent.h"

#include "MainComponentHelpers.h"

using rectai::ui::colourFromArgb;

bool MainComponent::loadAtlasResources()
{
    const juce::File pngFile = rectai::ui::loadFile("Resources/atlas_2048.png");
    if (!pngFile.existsAsFile()) {
        atlasLoaded_ = false;
        atlasImage_ = juce::Image();
        atlasSprites_.clear();
        return false;
    }

    const juce::File xmlFile = rectai::ui::loadFile("Resources/atlas_2048.xml");
    if (!xmlFile.existsAsFile()) {
        atlasLoaded_ = false;
        atlasImage_ = juce::Image();
        atlasSprites_.clear();
        return false;
    }

    juce::Image image = juce::ImageFileFormat::loadFrom(pngFile);
    if (!image.isValid()) {
        atlasLoaded_ = false;
        atlasImage_ = juce::Image();
        atlasSprites_.clear();
        return false;
    }

    // Convert to alpha mask (SingleChannel) so that drawing it uses the
    // current graphics colour (tinting). Without this, the original
    // pixels (likely black or dark) would be drawn as-is.
    if (image.getFormat() != juce::Image::SingleChannel) {
        juce::Image alphaImage(juce::Image::SingleChannel, image.getWidth(),
                               image.getHeight(), true);
        juce::Graphics g(alphaImage);
        g.drawImageAt(image, 0, 0);
        image = alphaImage;
    }

    juce::XmlDocument doc(xmlFile);
    std::unique_ptr<juce::XmlElement> root(doc.getDocumentElement());
    if (root == nullptr || !root->hasTagName("atlas")) {
        atlasLoaded_ = false;
        atlasImage_ = juce::Image();
        atlasSprites_.clear();
        return false;
    }

    std::unordered_map<std::string, AtlasSprite> sprites;
    for (auto* sprite = root->getChildByName("sprite"); sprite != nullptr;
         sprite = sprite->getNextElementWithTagName("sprite"))
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

    atlasLoaded_ = false;
    atlasImage_ = juce::Image();
    atlasSprites_.clear();
    return false;
}
