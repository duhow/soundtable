# soundtable

A vibe-coded [Reactable](https://reactable.com/) clone app for Linux.

Based on [JUCE Framework](https://juce.com/), [Fluidsynth](https://www.fluidsynth.org/), [reacTIVision](https://reactivision.sourceforge.net/) + [OpenCV](https://opencv.org/).

<p align="center">
  <img src="resources/snapshot.png" height="300">
</p>

> Soundtable uses a tangible interface where the player controls the system by manipulating objects.
> By putting these objects on the surface, turning them or connecting them to each other,
> players can combine different elements like synthesisers, effects, samples and control elements in order
> to create a unique composition.
> 
> The resulting sonic flows are represented graphically on the table surface,
> always showing the real waveforms that travel from one object to the other,
> turning music into something visible and tangible.

## Features

- Supports TUIO 1.1 (OSC `UDP/3333`) and works with TUIO Simulator
- Use fiducials with your camera (`tracker`)
- Load sessions from [Reactable Community](http://community.reactable.com/community/) (`.rtz` files)
- Play **Loop** audio samples in WAV, FLAC, Opus formats

> [!IMPORTANT]
> Implementation is still work in progress.

## Running

💾 [Download the latest release](https://github.com/duhow/soundtable/releases/latest) available and start playing!

On first run, resource data is extracted to `$HOME/.local/share/soundtable`.

To load a Reactable session:

- Download any `.rtz` files from [Reactable Community](http://community.reactable.com/community/).
- Drag the file to the AppImage executable, or open via terminal as argument.

## Architecture overview

The project is split into two main components, both built with CMake:

### Core app

- JUCE-based desktop application that renders the table UI, handles audio, MIDI and user interaction.
- Owns the main domain model (scene, objects, audio modules, connections) and paints everything from that model.
- Receives tracking data over TUIO/OSC (e.g. from the tracker or any TUIO 1.1 compatible source) and turns it into objects on the table.

### Tracker

- Standalone process that captures camera frames (OpenCV), detects fiducials and sends them as TUIO/OSC messages.
- Can be replaced by any other TUIO 1.1 sender (e.g. reacTIVision, TUIO Simulator) as long as it targets the same port (`UDP/3333`).

## History

Like any other fan of Reactable, I was amazed with the [psychosynth](https://github.com/arximboldi/psychosynth) project years ago, but it seemed to not gain enough attraction.
And the Reactable Mobile application sure was very entertaining, but it misses the physical part of handling the controls with your hands and not your fingers. 

Also I don't have the money or space to keep that huge table in my house, but still I'd love to play it some time.

So while talking to a friend recently, we discussed that LLMs were a thing and nowadays are working good enough, I thought about what project could I try to make... And so this idea came to my mind. 

### Copyright

Reactable content and materials are trademarks and copyrights of Reactable Systems or its licensors. All rights reserved.

Soundtable project is not affiliated with Reactable or its licensors.
