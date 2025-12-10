#pragma once

#include <string>
#include <vector>
#include <array>

#include "core/Scene.h"

namespace rectai {

// --- Shared data structures derived from the Reactable .rtp schema ---

// Generic envelope used by multiple module types (Filter, Delay, Loop, etc.).
struct Envelope {
        float attack{0.0F};
        float decay{0.0F};
        float duration{0.0F};
        float release{0.0F};
        std::vector<float> points_x;
        std::vector<float> points_y;
};

// Loop definition used by Loop tangibles.
struct LoopDefinition {
        int beats{0};
        std::string filename;
        int order{0};
};

// Tone / scale definition used by Tonalizer.
struct ToneDefinition {
        int key{0};
        std::vector<float> scale;  // Typically 12 values, but kept generic.
};

// Sequence track as described by <sequence> in Sequencer.
struct SequenceTrack {
        std::vector<int> rows;
        int speed{0};
        std::string speed_type;
        std::vector<float> step_frequencies;
        std::vector<int> steps;

        // tenori0..tenori12 layers. Index 0 corresponds to tenori0, etc.
        static constexpr int kTenoriLayerCount = 13;
        std::array<std::vector<int>, kTenoriLayerCount> tenori_layers{};

        std::vector<float> volumes;
};

// Instrument entry for Sampleplay tangibles.
struct SampleInstrument {
        std::string name;
};

// --- Concrete audio module implementations that leverage AudioModule ---

class OscillatorModule : public AudioModule {
 public:
         enum class Waveform {
                  kSine = 0,
                  kSaw,
                  kSquare,
                  kNoise,
         };

         explicit OscillatorModule(const std::string& id,
													float default_freq = 0.5F,
													float default_gain = 0.5F);

         [[nodiscard]] float default_parameter_value(
							const std::string& name) const override;

         // Reactable Oscillator tangibles also carry an envelope.
         [[nodiscard]] const Envelope& envelope() const { return envelope_; }
         Envelope& mutable_envelope() { return envelope_; }

         [[nodiscard]] Waveform waveform() const { return waveform_; }
         void set_waveform(Waveform waveform);
         void set_waveform_from_subtype(const std::string& subtype);
         void cycle_waveform();
         [[nodiscard]] std::string subtype_string() const;

 private:
         Envelope envelope_{};
         Waveform waveform_{Waveform::kSine};
};

// Output / Master module.
class OutputModule : public AudioModule {
 public:
         explicit OutputModule(const std::string& id);
};

// Tonalizer module (scales/keys).
class TonalizerModule : public AudioModule {
 public:
         explicit TonalizerModule(const std::string& id);

         [[nodiscard]] bool is_global_controller() const override
         {
                  return true;
         }

         [[nodiscard]] const std::vector<ToneDefinition>& tones() const
         {
                  return tones_;
         }

         std::vector<ToneDefinition>& mutable_tones() { return tones_; }

 private:
         std::vector<ToneDefinition> tones_;
};

// Filter module.
class FilterModule : public AudioModule {
 public:
         enum class Mode {
                  kLowPass = 0,
                  kBandPass,
                  kHighPass,
         };

         explicit FilterModule(const std::string& id,
														float default_cutoff = 0.5F,
														float default_q = 0.5F);

         [[nodiscard]] float default_parameter_value(
						const std::string& name) const override;

         [[nodiscard]] const Envelope& envelope() const { return envelope_; }
         Envelope& mutable_envelope() { return envelope_; }

         [[nodiscard]] Mode mode() const { return mode_; }
		 void set_mode(Mode mode);
		 void cycle_mode();
         void set_mode_from_subtype(const std::string& subtype);

 private:
         Envelope envelope_{};
         Mode mode_{Mode::kBandPass};
};

// Global volume / dynamics / FX send module.
class VolumeModule : public AudioModule {
 public:
        explicit VolumeModule(const std::string& id);

        [[nodiscard]] bool is_global_controller() const override
        {
                return true;
        }
};

// Tempo / global clock module.
class TempoModule : public AudioModule {
 public:
        explicit TempoModule(const std::string& id);

        [[nodiscard]] bool is_global_controller() const override
        {
                return true;
        }
};

// Accelerometer-based modulation module.
class AccelerometerModule : public AudioModule {
 public:
        explicit AccelerometerModule(const std::string& id);
};

// Low Frequency Oscillator module.
class LfoModule : public AudioModule {
 public:
        explicit LfoModule(const std::string& id);
};

// Step sequencer module.
class SequencerModule : public AudioModule {
 public:
        explicit SequencerModule(const std::string& id);

        [[nodiscard]] const std::vector<SequenceTrack>& tracks() const
        {
                return tracks_;
        }

        std::vector<SequenceTrack>& mutable_tracks() { return tracks_; }

 private:
        std::vector<SequenceTrack> tracks_;
};

// Delay / echo module.
class DelayModule : public AudioModule {
 public:
        explicit DelayModule(const std::string& id);

        [[nodiscard]] const Envelope& envelope() const { return envelope_; }
        Envelope& mutable_envelope() { return envelope_; }

        // Tangible-level hardlinks can be expressed later as Connections,
        // but we keep the original ids here for parsing.
        [[nodiscard]] const std::vector<int>& hardlink_targets() const
        {
                return hardlink_targets_;
        }

        std::vector<int>& mutable_hardlink_targets()
        {
                return hardlink_targets_;
        }

 private:
        Envelope envelope_{};
        std::vector<int> hardlink_targets_;
};

// Modulation effect module (e.g. ringmod).
class ModulatorModule : public AudioModule {
 public:
        explicit ModulatorModule(const std::string& id);

        [[nodiscard]] const Envelope& envelope() const { return envelope_; }
        Envelope& mutable_envelope() { return envelope_; }

 private:
        Envelope envelope_{};
};

// Waveshaper / distortion module.
class WaveShaperModule : public AudioModule {
 public:
        explicit WaveShaperModule(const std::string& id);

        [[nodiscard]] const Envelope& envelope() const { return envelope_; }
        Envelope& mutable_envelope() { return envelope_; }

 private:
        Envelope envelope_{};
};

// Input module (external audio).
class InputModule : public AudioModule {
 public:
        explicit InputModule(const std::string& id);

        [[nodiscard]] const Envelope& envelope() const { return envelope_; }
        Envelope& mutable_envelope() { return envelope_; }

 private:
        Envelope envelope_{};
};

// Loop player module.
class LoopModule : public AudioModule {
 public:
        explicit LoopModule(const std::string& id);

        [[nodiscard]] const Envelope& envelope() const { return envelope_; }
        Envelope& mutable_envelope() { return envelope_; }

        [[nodiscard]] const std::vector<LoopDefinition>& loops() const
        {
                return loops_;
        }

        std::vector<LoopDefinition>& mutable_loops() { return loops_; }

 private:
        Envelope envelope_{};
        std::vector<LoopDefinition> loops_;
};

// Sample-based playback module.
class SampleplayModule : public AudioModule {
 public:
        explicit SampleplayModule(const std::string& id);

        // Instruments as declared in the Reactable .rtp patch.
        [[nodiscard]] const std::vector<SampleInstrument>& instruments() const
        {
                return instruments_;
        }

        std::vector<SampleInstrument>& mutable_instruments()
        {
                return instruments_;
        }

        // Soundfont handling -------------------------------------------------
        //
        // The Sampleplay module can be associated with a single SoundFont
        // file (typically .sf2). For now we only provide a lightweight
        // loader that validates the basic SF2 header (RIFF/sfbk) and stores
        // the path for later use by the audio engine.

        [[nodiscard]] bool has_soundfont() const
        {
                return soundfont_loaded_;
        }

        [[nodiscard]] const std::string& soundfont_path() const
        {
                return soundfont_path_;
        }

        // Attempts to associate this module with a SoundFont file.
        // On success, returns true and clears `error_message` (if non-null).
        // On failure, returns false and writes a short description of the
        // problem into `error_message` (if provided).
        bool LoadSoundfont(const std::string& path,
                           std::string* error_message = nullptr);

 private:
        std::vector<SampleInstrument> instruments_;
        std::string soundfont_path_;
        bool soundfont_loaded_{false};
};

}  // namespace rectai
