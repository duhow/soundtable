# Plan de buffers por conexión y grafo de audio

Este documento describe cómo evolucionar desde el diseño actual
basado en buffers de waveform por voz en `AudioEngine` hacia un
modelo donde **cada conexión de audio** (dinámica o hardlink) tiene
su propio buffer de visualización, alineado con un grafo de audio
explícito. El objetivo es soportar más tipos de módulos (no sólo
filtros) sin depender de heurísticas ad‑hoc en la UI.

## 1. Contexto actual

### 1.1. AudioEngine simplificado

En el estado actual del proyecto (`core/src/AudioEngine.*`):

- El motor de audio no conoce el `Scene` ni las `Connection`.
- Gestiona hasta `kMaxVoices` osciladores con:
  - Frecuencia, nivel y forma de onda por voz.
  - Un filtro `StateVariableTPTFilter` por voz (modo low/band/high).
- Mantiene varios **buffers circulares** para visualización:
  - `waveformBuffer_`: mix mono global (osc + Sampleplay).
  - `voicePreFilterWaveformBuffer_`: señal por voz **antes** del filtro.
  - `voicePostFilterWaveformBuffer_`: señal por voz **después** del filtro.
  - `sampleplayWaveformBuffer_`: salida mono de Sampleplay antes de mezclar.
- Expone API de snapshots:
  - `getWaveformSnapshot(...)` → mix global.
  - `getVoiceWaveformSnapshot(...)` → pre‑filtro por voz.
  - `getVoiceFilteredWaveformSnapshot(...)` → post‑filtro por voz.
  - `getSampleplayWaveformSnapshot(...)` → Sampleplay mono.

### 1.2. UI y visualización

En `MainComponent_Paint.cpp`:

- Se piden snapshots por voz (pre/post) y de Sampleplay.
- Se normalizan y se calcula RMS para escalar la altura de la onda.
- Cada línea de audio se dibuja usando estas fuentes:
  - Líneas radiales módulo → centro: usan post‑filtro (voz o Sampleplay).
  - Conexiones módulo → módulo:
    - Si el origen es `FilterModule` → usa post‑filtro.
    - Si el origen es un generador (Osc) u otro módulo → usa pre‑filtro.

La selección de **qué buffer usar para cada conexión** está embebida en
la lógica de pintado, basada en el tipo de módulo origen.

### 1.3. Limitaciones

Este enfoque funciona para el caso simple Osc → Filter → Output, pero:

- Escala mal cuando haya más módulos de proceso (delay, waveshaper,
  moduladores, FX en cadena, etc.).
- Obliga a codificar lógica del ruteo de audio en la UI, en lugar de
  derivarla de un grafo de audio real.
- No hay un concepto explícito de **buffer por conexión**: reutilizamos
  buffers por voz y aplicamos heurísticas según el tipo de módulo.

## 2. Objetivos del nuevo diseño

1. **Buffers por conexión** (nivel conceptual y, a medio plazo,
   físico en el motor):
   - Cada `Connection` de `Scene` que transporte audio tiene asociado
     un buffer de waveform para visualización.
   - La UI deja de decidir “a ojo” si usa pre o post filtro; sólo
     consulta "la onda de esta conexión".

2. **Alineación con un grafo de audio explícito**:
   - El core/motor debe poder construir un grafo de procesado donde
     los módulos son nodos y las conexiones de audio son aristas.
   - Los taps de visualización se enganchan en esas aristas.

3. **Escalabilidad a nuevos módulos**:
   - Añadir un nuevo módulo de audio debería requerir, como máximo,
     registrar su nodo de audio y (opcionalmente) qué conexiones
     queremos monitorizar; no tocar código de pintado genérico.

4. **Respetar tiempo real**:
   - El hilo de audio escribe en buffers por conexión sin locks.
   - La UI sólo lee snapshots (downsampleados) desde el hilo de
     mensaje, como ya hace hoy.

5. **Migración incremental**:
   - No romper el MVP actual.
   - Permitir fases donde parte del ruteo siga siendo "simplificado"
     mientras se introduce el grafo real.

## 3. Diseño por fases

### Fase 0 – Estado actual (documentado)

Referenciar el comportamiento actual descrito en la sección 1 como
baseline. No requiere cambios, sólo sirve de punto de partida.

### Fase 1 – Abstracción de fuente de waveform por conexión (UI)

Objetivo: que la **UI** deje de decidir ad‑hoc qué buffer por voz usar
para cada conexión, y pase a consultar una abstracción intermedia
configurada desde la lógica de audio/scene.

Cambios principales:

1. **Nuevo descriptor de fuente visual por conexión** (sólo UI):

   - En `MainComponent` definir algo tipo:

     - `struct ConnectionVisualSource {`
       - `enum class Kind { None, VoicePre, VoicePost, Sampleplay };`
       - `Kind kind;`
       - `int voiceIndex;`  // válido para VoicePre/VoicePost
       - `// espacio para extensiones futuras (por ejemplo FX buses)`
       - `};`

   - Mapa: `std::unordered_map<ConnectionKey, ConnectionVisualSource>`.
     - `ConnectionKey` ya existe en helpers (`makeConnectionKey`).

2. **Asignación de fuentes por conexión**:

   - En el código que ya relaciona módulos con voces
     (`moduleVoiceIndex_` en `MainComponent` / `MainComponent_Audio.cpp`):
     - Para cada `Connection` de tipo audio:
       - Si el módulo origen es `OscillatorModule` → `Kind::VoicePre`.
       - Si el módulo origen es `FilterModule` u otro procesador que
         use una voz → `Kind::VoicePost`.
       - Si el origen/destino es `SampleplayModule` → `Kind::Sampleplay`.
   - Esta lógica queda centralizada en una función tipo
     `updateConnectionVisualSourcesFromScene()`.

3. **Uso en pintado**:

   - En `MainComponent_Paint.cpp`, en lugar de:
     - `if (fromIsFilter) usar voiceWaveformsPost; else voiceWaveformsPre;`
   - Se pasa a:
     - Buscar `ConnectionVisualSource` por clave de conexión.
     - Según `Kind`, seleccionar el buffer adecuado:
       - `VoicePre` → `voiceWaveformsPre[voiceIndex]`.
       - `VoicePost` → `voiceWaveformsPost[voiceIndex]`.
       - `Sampleplay` → `sampleplayWaveform`.
   - De cara a RMS y normalización se reutilizan los valores ya
     calculados (por voz o Sampleplay).

Resultado de Fase 1:

- Semánticamente, **cada conexión tiene asociada una fuente de
  waveform** bien definida.
- El código de pintado deja de depender directamente de tipos de
  módulo; sólo entiende `ConnectionVisualSource`.
- No se han creado todavía buffers físicos por conexión en el motor,
  pero la UI ya está preparada para ello.

### Fase 2 – Introducir grafo de audio lógico

Objetivo: acercar el motor a la arquitectura descrita en
`architecture_concepts.md` y `tech_stack.md`, definiendo un grafo de
audio explícito en el core.

Cambios previstos (alto nivel):

1. **Nuevo módulo "AudioGraph" en el core**:

   - Ficheros nuevos sugeridos:
     - `core/src/core/AudioGraph.h`
     - `core/src/core/AudioGraph.cpp`

   - Responsabilidades:
     - Representar nodos de audio (oscilador, filtro, volumen, etc.).
     - Representar conexiones de audio entre nodos.
     - Mantener un mapeo 1:1 (o N:1 donde aplique) entre
       `Scene::modules()` / `Scene::connections()` y el grafo.

2. **Integración con `AudioEngine`**:

   - `AudioEngine` se convierte en el "host" del grafo:
     - Mantiene un `AudioGraph` interno.
     - Implementa la lógica de procesado por bloque recorriendo el
       grafo o delegando en un `juce::AudioProcessorGraph`.
   - La instancia actual de osciladores + filtros por voz se puede:
     - Considerar como implementación inicial de algunos tipos de
       nodos (Osc/Filter), o
     - Mantener como backend simplificado mientras se implementa el
       grafo progresivamente.

3. **API de ruteo desde Scene**:

   - Una capa de coordinación (por ejemplo en `MainComponent_Audio`)
     que:
     - Observa cambios en `Scene` (módulos/conexiones).
     - Llama a métodos tipo `audioEngine_.rebuildGraphFromScene(scene_)`.

Resultado de Fase 2:

- El ruteo real de audio se corresponde con el grafo lógico de
  `Scene`.
- Es posible enganchar taps de visualización en cada arista del grafo.

### Fase 3 – Buffers físicos por conexión en el motor

Objetivo: que **cada conexión de audio** tenga su propio buffer
histórico en el lado del motor de audio, con escritura desde el hilo
de audio y lectura segura desde la UI.

Cambios propuestos:

1. **Estructura de buffer por conexión**:

   - Tipo base:

     - `struct WaveformRingBuffer {`
       - `static constexpr int kHistorySize = 4096;`
       - `float data[kHistorySize];`
       - `std::atomic<int> writeIndex;`
       - `// Opcionalmente, algún campo de estado mínimo.`
       - `void clear();`
       - `// Método thread-safe para copiar un snapshot downsampleado.`
       - `void getSnapshot(float* dst, int numPoints, double windowSeconds) const;`
       - `};`

   - Sin locks; el hilo de audio sólo hace `writeIndex += numSamples` y
     escribe en posiciones correspondientes.

2. **Taps por conexión en el grafo**:

   - Cada arista de audio relevante en `AudioGraph` mantiene una
     instancia de `WaveformRingBuffer`.
   - En el procesado de audio, justo después de calcular la señal en
     la conexión, se escribe en el buffer correspondiente.

3. **API pública en `AudioEngine`**:

   - Nuevos métodos:

     - `void getConnectionWaveformSnapshot(ConnectionId id, float* dst, int numPoints, double windowSeconds);`

   - Donde `ConnectionId` sea un handle opaco que la UI obtiene al
     pedir a `AudioEngine` que sincronice el grafo con `Scene`.

4. **Integración con la UI**:

   - `MainComponent` ya dispone, desde Fase 1, de
     `ConnectionVisualSource`. Podemos extenderlo para que:
     - `Kind::ConnectionTap` incluya un `ConnectionId`.
   - En este punto, la UI puede dejar de usar buffers por voz
     (`voicePreFilterWaveformBuffer_` y `voicePostFilterWaveformBuffer_`)
     para las conexiones, y pasar a usar exclusivamente
     `getConnectionWaveformSnapshot`.

Resultado de Fase 3:

- Cada conexión de audio tiene un buffer físico propio en el motor.
- El pintado de ondas se desacopla totalmente de detalles internos de
  implementación de módulos concretos (filtros, delays, etc.).

### Fase 4 – Limpieza y consolidación

Objetivo: eliminar caminos antiguos y simplificar la base de código.

Posibles tareas:

- Deprecar o simplificar buffers por voz si ya no son necesarios para
  visualización (pueden mantenerse sólo si son útiles para debug).
- Revisar que todos los módulos de audio usen el grafo y los taps por
  conexión.
- Actualizar tests y documentación para reflejar la nueva fuente de
  verdad (grafo + buffers por conexión).

## 4. Estrategia de implementación inicial

Para minimizar riesgo en el MVP, la propuesta es:

1. Implementar **Fase 1** primero:
   - Añadir `ConnectionVisualSource` y el mapa en `MainComponent`.
   - Extraer la lógica de asignación desde `Scene` a una función clara.
   - Refactorizar `MainComponent_Paint.cpp` para que consuma esta
     abstracción en lugar de ramas especiales por tipo de módulo.

2. Validar visualmente y con tests existentes que el comportamiento
   (especialmente Osc → Filter → Output) sigue siendo correcto.

3. Documentar en `research/progress.md` los cambios y decisiones.

4. Más adelante, abordar Fases 2 y 3 cuando el motor necesite soportar
   más módulos y ruteos complejos.

Este enfoque permite que, desde ya, el código de la UI piense en
**"buffers por conexión"** aunque en la implementación física todavía
se apoye en buffers por voz. Cuando el grafo y los taps estén
implementados, la transición en la UI debería ser principalmente
un cambio de fuente (`VoicePre/VoicePost` → `ConnectionTap`).
