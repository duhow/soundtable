### Code layout (quick tour)

- `src/Main.cpp`: JUCE entry point, creates the main window and `MainComponent`.
- `src/MainComponent*.{h,cpp}`: main UI component and helpers; owns the live scene and paints the table.
- `src/core/Scene.{h,cpp}`: core domain model (objects, audio modules, connections, scene state).
- `src/TrackingOscReceiver.{h,cpp}`: translates incoming TUIO/OSC messages into scene updates.
