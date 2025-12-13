# Diseño del módulo Sequencer y señales MIDI

Este documento describe el diseño del módulo **Sequencer** y de la capa de señales **MIDI internas** para el proyecto RectaiTable. El objetivo es:

- Ser compatible con la semántica original de Reactable (patches `.rtp`, tangibles tipo `Sequencer`, `Sampleplay`, `Oscillator`, etc.).
- Unificar el envío de **eventos de nota** hacia distintos módulos.
- Preparar el terreno para **3 modos de secuenciador**: Monophonic, Polyphonic y Random (implementando primero Monophonic).

---

## 1. Modelo de señal MIDI interna

### 1.1. Evento de nota interno

Definimos una representación ligera de nota MIDI que no depende de JUCE, pero que se pueda mapear fácilmente a `juce::MidiMessage` (o a llamadas directas al motor de audio):

- `channel` (int): canal lógico MIDI.
- `note` (int): número de nota MIDI (0–127).
- `velocity01` (float): velocidad normalizada en [0, 1].
- `timeBeats` (double): posición temporal en beats, relativa al tempo global.
- `isNoteOn` (bool): indica si es un NoteOn (`true`) o NoteOff (`false`).

Ejemplo de estructura C++ (nombre orientativo):

```cpp
struct MidiNoteEvent {
    int channel{0};
    int note{0};
    float velocity01{0.0F};
    double timeBeats{0.0};
    bool isNoteOn{true};
};
```

Esta estructura se usará como moneda común entre el secuenciador y los módulos que consumen notas (por ejemplo `SampleplayModule`, `OscillatorModule`).

### 1.2. Tipos de señal en los puertos

Actualmente `PortDescriptor` diferencia únicamente si un puerto es de audio (`bool is_audio`). Para soportar MIDI y control, se propone:

- Añadir un enum:

```cpp
enum class PortSignalKind {
    kAudio = 0,
    kMidi,
    kControl,
};
```

- Evolucionar `PortDescriptor` a:

```cpp
struct PortDescriptor {
    std::string name;
    PortSignalKind kind{PortSignalKind::kAudio};
};
```

De esta forma:

- Puertos de audio seguirán marcados como `kAudio`.
- Puertos que transportan eventos de nota usarán `kMidi`.
- Puertos futuros para automatización lenta o CV digital podrán usar `kControl`.

### 1.3. Capacidades del módulo para audio / MIDI

`AudioModule` ya contiene flags para audio (`produces_audio_`, `consumes_audio_`). Para MIDI se añadirán:

- `bool produces_midi_`.
- `bool consumes_midi_`.

Y se adaptará `CanConnectTo` para permitir:

- Conexiones **audio → audio** (comportamiento actual).
- Conexiones **MIDI → MIDI**.
- (Opcional a futuro) reglas específicas para `kControl`.

Reglas básicas de conexión propuestas:

- Un módulo sólo puede conectar una salida a una entrada si el **tipo de señal** es compatible.
- Los **global controllers** (`TonalizerModule`, `TempoModule`, `VolumeModule`) siguen **fuera** del grafo explícito de conexiones: no se conectan mediante `Scene::connections`.

### 1.4. Enrutado MIDI dentro del motor de audio

En el `AudioEngine` se añadirá una capa de ruteo para MIDI:

- A partir de `Scene::connections()`, el engine construirá una tabla de rutas MIDI:
  - Por ejemplo: `Sequencer(id=5) → Sampleplay(id=48)`.
- Durante el procesamiento de audio, el `Sequencer` generará `MidiNoteEvent` que se inyectarán en un **scheduler MIDI**.
- El scheduler recorrerá las rutas activas y llamará a las acciones adecuadas según el tipo de módulo destino:
  - Sampleplay: traducir a `triggerSampleplayNote(bank, program, midiKey, velocity01)`.
  - Oscillator: convertir la nota MIDI a frecuencia en Hz (`midiNoteToFrequency`) y mapear la velocidad a nivel por voz (`setVoice(...)`).

Esta capa está separada de JUCE y del grafo de audio puro para mantener el modelo de dominio (`Scene` + `AudioModule`) limpio.

---

## 2. Rol de los módulos frente a MIDI

### 2.1. SequencerModule

- Función principal: **producir eventos de nota MIDI** según una secuencia de 16 pasos y el tempo global.
- Los puertos del secuenciador se marcarán como:
  - Salida: `PortSignalKind::kMidi`.

### 2.2. SampleplayModule

- Consigue reproducir muestras (vía SoundFont / FluidSynth) en respuesta a notas MIDI.
- Cambios conceptuales:
  - El módulo **consume MIDI**.
  - Las notas entrantes se traducen a llamadas a `SampleplaySynth`:
    - `triggerSampleplayNote(bank, program, midiKey, velocity01)`.
  - `channel` de los eventos MIDI se puede mapear a los **bancos lógicos** ya presentes (`channel`, `default_preset_indices_`).

### 2.3. OscillatorModule

- También puede **consumir MIDI** para tocar melodías monofónicas o polifónicas.
- Estrategia:
  - Convertir nota MIDI → frecuencia en Hz.
  - Asignar voces disponibles en el `AudioEngine` (`setVoice(index, frequency, level)`).
  - La `velocity01` se mapea a `level`.

### 2.4. TonalizerModule como controlador global

- `TonalizerModule` sigue siendo `is_global_controller() == true`.
- No participa en conexiones del grafo, pero sí en la lógica de notas:
  - Antes de emitir una `MidiNoteEvent` efectiva, el motor MIDI podrá consultar el Tonalizer:
    - Transformar el pitch según **escala** y **tonalidad**.
  - Esto mantiene la compatibilidad con el comportamiento original de Reactable, donde el tonalizer afecta a todo el sistema.

---

## 3. Diseño de datos del Sequencer (general, 3 modos)

El secuenciador está pensado como un **loop de 16 pasos** con **6 presets** por objeto tangible, donde cada preset guarda una secuencia completa.

### 3.1. Estructura de presets

- Cada `SequencerModule` contiene un banco de presets:

```cpp
static constexpr int kNumPresets = 6;
static constexpr int kNumSteps = 16;

struct SequencerStep {
    bool enabled{true};
    float velocity01{1.0F};
    int pitch{60}; // MIDI note (C4 por defecto)
    // Extensible para más datos en el futuro.
};

struct SequencerPreset {
    std::array<SequencerStep, kNumSteps> steps;
    // metadata opcional: nombre, color, etc.
};
```

- El módulo mantiene:

```cpp
enum class SequencerMode {
    kMonophonic = 0,
    kPolyphonic,
    kRandom,
};

class SequencerModule : public AudioModule {
public:
    explicit SequencerModule(const std::string& id);

    const std::vector<SequenceTrack>& tracks() const;
    std::vector<SequenceTrack>& mutable_tracks();

    // API extendida para presets/steps (a implementar):
    // - Acceso a presets
    // - Lectura/escritura de steps
    // - Modo de secuenciador, preset actual, etc.

private:
    SequencerMode mode_{SequencerMode::kMonophonic};
    int currentPreset_{0}; // 0..5
    std::array<SequencerPreset, kNumPresets> presets_;
};
```

> Nota: `SequenceTrack` ya existe en `AudioModules.h` y está inspirado en el esquema Reactable (`<sequence>`, capas tenori, velocidades, etc.). La idea es **no romper** esa estructura, sino usarla como origen de datos para poblar `SequencerPreset` / `SequencerStep`.

### 3.2. Relación con el `.rtp` original

- El tangible `Sequencer` en el `.rtp` actual contiene atributos:
  - `subtype="sequencer"` o `"tenori"`.
  - `speed_type="binary"`, `speed="1"`, `point` (selección de preset), etc.
- El loader (`ReactableRtpLoader`) se encargará de:
  - Interpretar `subtype` → `SequencerMode`.
  - Interpretar `point` → `currentPreset_`.
  - Mapear la información de `<sequence>` y capas tenori (`tenori0..12`) a la estructura interna de presets y steps.

### 3.3. Modos soportados

- **Monophonic**: cada step tiene a lo sumo **una nota**.
- **Polyphonic**: cada step puede contener **varias notas**.
- **Random**: reutiliza la misma estructura de datos, pero cambia la lógica de reproducción (orden, elección de pitch, etc.).

Para soportar Polyphonic/Random más adelante sin romper el diseño:

- Se puede extender `SequencerStep` con:

```cpp
std::vector<int> pitches; // para poly
```

- En modo Monophonic se usaría únicamente `pitch` (o `pitches[0]`).

---

## 4. Lógica del Sequencer Monophonic

### 4.1. Sincronización con el TempoModule

- El secuenciador se sincroniza con el `TempoModule` global:
  - `tempo` (BPM) → conversión de muestras a beats.
  - Se asume una resolución fija de **16 pasos por compás** (cada step = semicorchea), compatible con el diseño original.

- En el `AudioEngine` se mantiene un acumulador de fase en beats:

```cpp
double sequencerPhaseBeats = 0.0;
```

- En cada callback de audio:

```cpp
const double seconds = numSamples / sampleRate_;
const double beats = seconds * (tempoBpm / 60.0);
sequencerPhaseBeats += beats;
```

- A partir de `sequencerPhaseBeats` se determina el índice de step actual:

```cpp
const double beatsPerStep = 1.0 / 4.0; // 16 steps por compás
int stepIndex = static_cast<int>(sequencerPhaseBeats / beatsPerStep) % kNumSteps;
```

### 4.2. Generación de notas por step

Al cruzar de un step al siguiente:

1. Si había una nota activa en el step anterior, se genera un `NoteOff`.
2. Se mira el step actual:
   - Si `enabled == false` → no se genera nota nueva.
   - Si `enabled == true` y `velocity01 > 0` → se genera un `NoteOn` con:
     - `pitch` del step (posiblemente transformado por Tonalizer).
     - `velocity01` del step.

Estos eventos se insertan en la cola MIDI con `timeBeats` apropiado para que el scheduler los dispare en el bloque de audio correcto.

### 4.3. Panel de pitch (Monophonic)

El modo monofónico tiene un panel especial para editar alturas:

- A nivel de modelo:
  - Cada `SequencerStep` ya tiene un `pitch`.
  - El panel modifica ese campo para el preset actual.
- Integración con Tonalizer:
  - El `pitch` puede interpretarse como grado de escala o como nota MIDI absoluta;
  - Antes de convertir a frecuencia / Sampleplay, se puede pasar por el Tonalizer para aplicar escala/tonalidad.

### 4.4. Interacciones de usuario (steps y volumen)

- **Desactivar paso**: el gesto de pulsar uno de los 16 slots alrededor del objeto toggleará `enabled` del `SequencerStep` correspondiente.
- **Cambiar volumen de step**: un gesto que parte del punto y se aleja del objeto modificará `velocity01` del step (por ejemplo mapeando distancia a [0, 1]).
- Todos estos cambios se almacenan en el **preset actual** (`currentPreset_`) y se reflejan en la siguiente iteración del secuenciador.

---

## 5. Encaje con Polyphonic y Random

### 5.1. Polyphonic

- Reutiliza la misma infraestructura:
  - Mismo `SequencerModule`.
  - Mismos `SequencerPreset` y steps, pero con soporte para varios `pitches` por step.
- En el engine, el runtime polifónico:
  - Al entrar en un step:
    - Envía múltiples `NoteOn` (uno por pitch activo).
  - Al salir del step:
    - Envía los correspondientes `NoteOff`.
- El modo „Tenori” del patch Reactable se puede mapear a esta representación, usando las capas `tenori0..12` ya presentes en `SequenceTrack`.

### 5.2. Random

- Comparte la misma estructura de presets y steps.
- Cambia la **política de avance**:
  - En lugar de ir en orden 0,1,2,...,15, el runtime elige el siguiente step según una regla aleatoria (por ejemplo, entre todos los steps `enabled`).
  - Alternativamente, puede aleatorizar el `pitch` dentro de un rango/escala manteniendo el grid temporal.
- La interfaz externa del módulo (puertos, parámetros comunes, presets) se mantiene igual; sólo cambia la implementación interna del scheduler.

### 5.3. Runtimes específicos por modo

Para aislar la lógica de reproducción de la estructura de datos, se pueden definir runtimes separados:

- `MonophonicSequencerRuntime`.
- `PolyphonicSequencerRuntime`.
- `RandomSequencerRuntime`.

Cada uno toma como entrada:

- Referencia al `SequencerModule` (lectura de presets, modo, preset actual).
- Estado del tempo (`TempoModule`).
- Posición temporal (fase en beats) que proporciona el `AudioEngine`.

Y produce:

- Lista de `MidiNoteEvent` a encolar para el bloque de audio actual.

---

## 6. Roadmap de implementación

1. **Infraestructura MIDI en el modelo**
   - Extender `PortDescriptor` con `PortSignalKind`.
   - Añadir flags `produces_midi_` / `consumes_midi_` a `AudioModule`.
   - Adaptar `CanConnectTo` para considerar audio vs MIDI y mantener `is_global_controller()` sin conexiones.
   - Definir `MidiNoteEvent` y tipos auxiliares en un header común.

2. **Estado del SequencerModule**
   - Completar `SequencerModule` con:
     - `SequencerMode mode_`.
     - `int currentPreset_` (0–5).
     - `std::array<SequencerPreset, kNumPresets>`.
   - Añadir API para lectura/escritura de steps y presets desde la UI y el loader `.rtp`.

3. **Runtime Monophonic**
   - Implementar un runtime (en el `AudioEngine` o módulo auxiliar) que:
     - Lea BPM y resolución a partir del `TempoModule`.
     - Mantenga la fase en beats y determine el step actual.
     - Genere `MidiNoteEvent` según los steps del preset activo.

4. **Enrutado hacia Sampleplay y Oscillator**
   - A partir de `Scene::connections()`, construir rutas MIDI.
   - Para cada evento:
     - Si el destino es `SampleplayModule`, llamar a `triggerSampleplayNote` con los parámetros correctos.
     - Si el destino es `OscillatorModule`, ajustar frecuencia y nivel en una voz libre.

5. **Extensión a Polyphonic y Random**
   - Ampliar `SequencerStep` para soportar múltiples pitches por step.
   - Implementar runtimes específicos para Polyphonic y Random, compartiendo el mismo estado y APIs públicas.
   - Actualizar UI y loader `.rtp` para mapear los modos `subtype="tenori"` y el modo aleatorio.
