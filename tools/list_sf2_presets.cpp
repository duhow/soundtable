#include <fluidsynth.h>

#include <cstdio>
#include <cstdlib>
#include <string>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <soundfont.sf2>\n", argv[0]);
        return 1;
    }

    const std::string path = argv[1];

    fluid_settings_t* settings = new_fluid_settings();
    if (settings == nullptr) {
        std::fprintf(stderr, "Failed to allocate FluidSynth settings\n");
        return 1;
    }

    // This tool sólo necesita enumerar presets offline; pedimos a
    // FluidSynth que use el driver de audio "null" para evitar
    // que intente inicializar backends como SDL3 y emita warnings
    // innecesarios en stdout.
    (void) fluid_settings_setstr(settings, "audio.driver", "null");

    fluid_synth_t* synth = new_fluid_synth(settings);
    if (synth == nullptr) {
        std::fprintf(stderr, "Failed to create FluidSynth synthesiser\n");
        delete_fluid_settings(settings);
        return 1;
    }

    const int sfid = fluid_synth_sfload(synth, path.c_str(), 0);
    if (sfid < 0) {
        std::fprintf(stderr, "Failed to load soundfont: %s\n", path.c_str());
        delete_fluid_synth(synth);
        delete_fluid_settings(settings);
        return 1;
    }

    fluid_sfont_t* sfont = fluid_synth_get_sfont_by_id(synth, sfid);
    if (sfont == nullptr) {
        std::fprintf(stderr, "fluid_synth_get_sfont_by_id returned null for %s\n", path.c_str());
        fluid_synth_sfunload(synth, sfid, 0);
        delete_fluid_synth(synth);
        delete_fluid_settings(settings);
        return 1;
    }

    std::printf("Listing presets for %s:\n", path.c_str());
    std::printf("bank\tprogram\tname\n");
    // Fallback approach compatible con varias versiones: probar
    // todas las combinaciones razonables de banco/programa usando
    // fluid_sfont_get_preset.
    // Algunos bancos de percusión se ubican en bank >= 128, así
    // que ampliamos el rango de bancos respecto al 0..127 clásico.
    for (int bank = 0; bank < 256; ++bank) {
        for (int program = 0; program < 128; ++program) {
            fluid_preset_t* preset = fluid_sfont_get_preset(sfont, bank, program);
            if (preset == nullptr) {
                continue;
            }
            const int realBank = fluid_preset_get_banknum(preset);
            const int realProgram = fluid_preset_get_num(preset);
            const char* name = fluid_preset_get_name(preset);
            std::printf("%d\t%d\t%s\n", realBank, realProgram,
                        name != nullptr ? name : "");
        }
    }

    fluid_synth_sfunload(synth, sfid, 0);
    delete_fluid_synth(synth);
    delete_fluid_settings(settings);

    return 0;
}
