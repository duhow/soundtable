#include "core/SoundfontUtils.h"

// To avoid depending on FluidSynth's opaque C structs from C++, we
// forward-declare only the small subset of the C API that we need and
// treat SoundFont/preset handles as opaque void* pointers. This keeps
// the C++ side independent of the exact struct layout while still
// linking against the system FluidSynth library.

extern "C" {

struct _fluid_settings_t;
struct _fluid_synth_t;

typedef struct _fluid_settings_t fluid_settings_t;
typedef struct _fluid_synth_t fluid_synth_t;

fluid_settings_t* new_fluid_settings(void);
void delete_fluid_settings(fluid_settings_t*);

int fluid_settings_setstr(fluid_settings_t* settings,
                          const char* name,
                          const char* str);

fluid_synth_t* new_fluid_synth(fluid_settings_t* settings);
void delete_fluid_synth(fluid_synth_t* synth);

int fluid_synth_sfload(fluid_synth_t* synth, const char* filename,
                       int reset_presets);
int fluid_synth_sfunload(fluid_synth_t* synth, int sfid,
                         int reset_presets);

void* fluid_synth_get_sfont_by_id(fluid_synth_t* synth, int sfid);

void* fluid_sfont_get_preset(void* sfont, int bank, int prenum);
int fluid_preset_get_banknum(void* preset);
int fluid_preset_get_num(void* preset);
const char* fluid_preset_get_name(void* preset);

}  // extern "C"

namespace rectai {

bool EnumerateSoundfontPresets(const std::string& path,
                               std::vector<SoundfontPreset>& out_presets,
                               std::string* const error_message)
{
    if (error_message != nullptr) {
        *error_message = {};
    }

    out_presets.clear();

    fluid_settings_t* settings = new_fluid_settings();
    if (settings == nullptr) {
        if (error_message != nullptr) {
            *error_message = "Failed to allocate FluidSynth settings";
        }
        return false;
    }

    // We only need offline preset enumeration here, so disable any
    // attempt by FluidSynth to create a real audio driver to avoid
    // warnings about SDL3/audio backends on systems where they are
    // not initialised.
    (void)fluid_settings_setstr(settings, "audio.driver", "null");

    fluid_synth_t* synth = new_fluid_synth(settings);
    if (synth == nullptr) {
        delete_fluid_settings(settings);
        if (error_message != nullptr) {
            *error_message = "Failed to create FluidSynth synthesiser";
        }
        return false;
    }

    const int sfid = fluid_synth_sfload(synth, path.c_str(), 0);
    if (sfid < 0) {
        if (error_message != nullptr) {
            *error_message = "FluidSynth could not load soundfont";
        }
        delete_fluid_synth(synth);
        delete_fluid_settings(settings);
        return false;
    }

    void* sfont = fluid_synth_get_sfont_by_id(synth, sfid);
    if (sfont == nullptr) {
        if (error_message != nullptr) {
            *error_message = "FluidSynth returned null soundfont handle";
        }
        fluid_synth_sfunload(synth, sfid, 1);
        delete_fluid_synth(synth);
        delete_fluid_settings(settings);
        return false;
    }

    // Probe a reasonable range of bank/program combinations using the
    // public getter API. This avoids relying on any internal
    // iteration macros that may differ across FluidSynth versions.
    //
    // Some soundfonts place drumkits or alternative presets on
    // banks >= 128 (e.g. 128 for GM drum channel). To avoid
    // missing those, we scan a slightly wider range of banks.
    for (int bank = 0; bank < 256; ++bank) {
        for (int program = 0; program < 128; ++program) {
            void* preset = fluid_sfont_get_preset(sfont, bank, program);
            if (preset == nullptr) {
                continue;
            }

            SoundfontPreset p;
            p.bank = fluid_preset_get_banknum(preset);
            p.program = fluid_preset_get_num(preset);
            const char* name = fluid_preset_get_name(preset);
            if (name != nullptr) {
                p.name = name;
            }
            out_presets.push_back(std::move(p));
        }
    }

    fluid_synth_sfunload(synth, sfid, 1);
    delete_fluid_synth(synth);
    delete_fluid_settings(settings);

    if (out_presets.empty()) {
        if (error_message != nullptr) {
            *error_message = "Soundfont contains no presets";
        }
        return false;
    }

    return true;
}

}  // namespace rectai
