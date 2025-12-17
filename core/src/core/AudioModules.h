#pragma once

#include <string>
#include <vector>
#include <array>
#include <algorithm>

#include "core/Scene.h"

namespace rectai {

// Canonical id used for the invisible master Output module.
inline constexpr const char* MASTER_OUTPUT_ID = "-1";

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

// Step representation used by the internal sequencer.
//
// The current runtime is monophonic and primarily uses the single
// `pitch` field, but `pitches` is already present so that future
// polyphonic modes can attach multiple notes to a single step
// without changing the data model.
struct SequencerStep {
                bool enabled{true};
                float velocity01{1.0F};
                int pitch{60};  // Primary MIDI note, C4 by default.
                std::vector<int> pitches;  // Optional polyphonic pitches.
};

// Fixed-size preset bank for the Sequencer.
struct SequencerPreset {
        static constexpr int kNumSteps = 16;
        std::array<SequencerStep, kNumSteps> steps{};
};

// Instrument entry for Sampleplay tangibles.
struct SampleInstrument {
        std::string name;
        int bank{0};
        int program{0};
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

         [[nodiscard]] float default_parameter_value(const std::string& name) const override;

         // Reactable Oscillator tangibles also carry an envelope.
         [[nodiscard]] const Envelope& envelope() const { return envelope_; }
         Envelope& mutable_envelope() { return envelope_; }

         [[nodiscard]] Waveform waveform() const { return waveform_; }
         void set_waveform(Waveform waveform);
         void set_waveform_from_subtype(const std::string& subtype);
         void cycle_waveform();
         [[nodiscard]] std::string subtype_string() const;
         [[nodiscard]] int waveform_index() const
         {
                  return static_cast<int>(waveform_);
         }

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

         // Envelope configuration methods.
         void set_envelope_attack(float attack_ms);
         void set_envelope_decay(float decay_ms);
         void set_envelope_duration(float duration_ms);
         void set_envelope_release(float release_ms);

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

        // Canonical BPM range for the global Tempo controller.
        static constexpr float kMinBpm = 40.0F;
        static constexpr float kMaxBpm = 400.0F;

        // Clamp a raw BPM value to the supported range.
        [[nodiscard]] static float ClampBpm(float bpm)
        {
                return std::clamp(bpm, kMinBpm, kMaxBpm);
        }

        // Map a normalised [0,1] value to a BPM in [kMinBpm,kMaxBpm].
        [[nodiscard]] static float BpmFromNormalised(float value01)
        {
                const float clamped = std::clamp(value01, 0.0F, 1.0F);
                return kMinBpm + (kMaxBpm - kMinBpm) * clamped;
        }

        // Convert a BPM value in the valid range back to [0,1].
        [[nodiscard]] static float NormalisedFromBpm(float bpm)
        {
                const float clamped = ClampBpm(bpm);
                return (clamped - kMinBpm) / (kMaxBpm - kMinBpm);
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
         enum class Mode {
                 kMonophonic = 0,
                 kPolyphonic,
                 kRandom,
         };

        explicit SequencerModule(const std::string& id);

        // Reactable Sequencer version as declared in the .rtp
        // tangible. When the attribute `version` is not present we
        // treat it as version 1 (legacy pulse-based mode). Version 2
        // and above are reserved for future melodic/advanced modes.
        [[nodiscard]] int version() const { return version_; }
        void set_version(int version) { version_ = version; }

        [[nodiscard]] const std::vector<SequenceTrack>& tracks() const
        {
                return tracks_;
        }

        std::vector<SequenceTrack>& mutable_tracks() { return tracks_; }

                // High-level presets/steps API (MVP: monophonic only).
                [[nodiscard]] Mode mode() const { return mode_; }
                void set_mode(Mode mode) { mode_ = mode; }

                static constexpr int kNumPresets = 6;
                [[nodiscard]] int current_preset() const { return current_preset_; }
                void set_current_preset(int index)
                {
                        if (index < 0) {
                                current_preset_ = 0;
                        } else if (index >= kNumPresets) {
                                current_preset_ = kNumPresets - 1;
                        } else {
                                current_preset_ = index;
                        }
                }

                [[nodiscard]] const SequencerPreset& preset(int index) const
                {
                        return presets_[static_cast<std::size_t>(index)];
                }

                SequencerPreset& mutable_preset(int index)
                {
                        return presets_[static_cast<std::size_t>(index)];
                }

                // Populate high-level presets/steps from low-level
                // SequenceTrack data loaded from .rtp files.
                void SyncPresetsFromTracks();

 private:
        std::vector<SequenceTrack> tracks_;
                Mode mode_{Mode::kMonophonic};
                int current_preset_{0};  // 0..kNumPresets-1
                std::array<SequencerPreset, kNumPresets> presets_{};
                int version_{1};
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

        // Instruments as rebuilt from the SoundFont presets. Each
        // entry represents a concrete (bank, program, name) preset
        // that can be triggered via FluidSynth.
        [[nodiscard]] const std::vector<SampleInstrument>& instruments() const
        {
                return instruments_;
        }

        std::vector<SampleInstrument>& mutable_instruments()
        {
                return instruments_;
        }

        // Returns the index of the currently active instrument, or
        // -1 when there are no instruments defined.
        [[nodiscard]] int active_instrument_index() const;

        // Sets the active instrument index, clamping it to the valid
        // range [0, instruments().size()). If there are no
        // instruments, the index is set to -1.
        void set_active_instrument_index(int index);

        // Convenience helper used by the UI: advances the active
        // instrument index to the next available entry, wrapping
        // around at the end of the list.
        void CycleInstrument();

        // Returns a pointer to the currently active instrument, or
        // nullptr if there is none.
        [[nodiscard]] const SampleInstrument* active_instrument() const;

        // Logical bank/channel handling -----------------------------------
        //
        // Reactable patches for Sampleplay use the "channel" attribute
        // together with a list of <instrument> elements to describe
        // several "modes" (for example, drums vs synths). In this
        // project we interpret that attribute as a logical bank index
        // and map each bank to a specific SoundFont preset.

        // Current logical channel/bank index as loaded from the .rtp
        // attribute "channel" (typically 0 for drums, 1 for synths).
        [[nodiscard]] int channel() const { return channel_; }
        void set_channel(int channel) { channel_ = channel; }

        // Default preset indices per logical channel. The i-th element
        // in this vector represents the index in instruments_ of the
        // preset associated with logical bank i. Negative values
        // indicate that no valid preset was found for that bank.
        void set_default_preset_indices(std::vector<int> indices)
        {
                default_preset_indices_ = std::move(indices);
        }

        [[nodiscard]] const std::vector<int>& default_preset_indices() const
        {
                return default_preset_indices_;
        }

        // Switches logical bank (for example, from drums to synths) by
        // advancing the channel index and selecting the default preset
        // associated with that bank when available.
        void CycleBank();

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

        // Raw filename as declared in the Reactable patch (e.g.
        // "default.sf2"). This is kept separate from `soundfont_path_`,
        // which stores the fully resolved path validated by
        // LoadSoundfont(). UI code can use this value together with
        // rectai::ui::loadFile to locate the asset within the
        // com.reactable/ tree.
        [[nodiscard]] const std::string& raw_soundfont_name() const
        {
                return raw_soundfont_name_;
        }

        void set_raw_soundfont_name(std::string name)
        {
                raw_soundfont_name_ = std::move(name);
        }

        // Attempts to associate this module with a SoundFont file.
        // On success, returns true and clears `error_message` (if non-null).
        // On failure, returns false and writes a short description of the
        // problem into `error_message` (if provided).
        bool LoadSoundfont(const std::string& path,
                           std::string* error_message = nullptr);

 private:
        std::vector<SampleInstrument> instruments_;
        std::string raw_soundfont_name_;
        std::string soundfont_path_;
        bool soundfont_loaded_{false};
        int active_instrument_index_{-1};
        int channel_{0};
        std::vector<int> default_preset_indices_;
};

}  // namespace rectai
