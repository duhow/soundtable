# Plan de refactor de soundwaves y "Voces"

Este documento define un plan por fases para simplificar y unificar la generación de **ondas de sonido visibles** (soundwaves) en RectaiTable, basándose en un modelo de **Voces por módulo**.

El objetivo es sustituir el sistema actual, híbrido y con lógica repartida entre `AudioEngine` y `MainComponent`, por una arquitectura más clara donde:

- Cada **módulo que genera audio** tiene su **propia voz visual** con un buffer exclusivo.
- Las **conexiones** muestran la onda que realmente circula por ellas, diferenciando la onda original del generador y la onda transformada tras filtros/FX.
- Toda la lógica de actualización de buffers de soundwave se concentra en una **clase dedicada (Voces)**, evitando código duplicado y condicionales dispersos.

---

## 1. Situación actual (resumen)

### 1.1. Generación de soundwaves en `AudioEngine`

Actualmente `AudioEngine` ya mantiene varios historiales de waveform para visualización ([core/src/AudioEngine.h](core/src/AudioEngine.h)):

- Mezcla global mono (buffer `waveformBuffer_` + `getWaveformSnapshot(...)`).
- Buffers por voz de oscilador:
  - `voicePreFilterWaveformBuffer_` → señal **pre‑filtro** por voz
  - `voicePostFilterWaveformBuffer_` → señal **post‑filtro** por voz
  - Acceso vía `getVoiceWaveformSnapshot(...)` y `getVoiceFilteredWaveformSnapshot(...)`.
- Buffers por conexión:
  - `connectionTaps_` + `connectionWaveformBuffers_` + `getConnectionWaveformSnapshot(...)`.
- Buffers por LoopModule:
  - `loopWaveformBuffers_` + `loopModuleToWaveformIndex_` + `getLoopModuleWaveformSnapshot(...)`.

La escritura en estos buffers se hace directamente desde `audioDeviceIOCallbackWithContext(...)`, mezclando responsabilidades de síntesis y visualización.

### 1.2. Lógica visual en `MainComponent`

En el lado UI, la información de waveforms se usa principalmente en:

- [core/src/MainComponent_Paint.cpp](core/src/MainComponent_Paint.cpp):
  - `AudioFrameState` acumula snapshots por voz (pre/post), métricas y punteros a `connectionVisualSources_`.
  - Se usan varios caminos para obtener ondas:
    - Voz pre/post (para conexiones generador→filtro, filtros hacia master, etc.).
    - Buffers de conexión (`getConnectionWaveformSnapshot`) cuando hay taps configurados.
    - Buffers de Loop (`getLoopModuleWaveformSnapshot`) para radiales y conexiones relacionadas.
  - Existen ramas específicas para Sampleplay, Loop, conexiones compuestas, split de líneas con hold‑mute, agregados de entrada, etc.

- [core/src/MainComponent_Audio.cpp](core/src/MainComponent_Audio.cpp):
  - `modulesWithActiveAudio_` y `moduleVoiceIndex_` deciden qué módulos “tienen audio”.
  - `updateConnectionVisualSources()` mapea cada conexión de audio a un `ConnectionVisualSource`:
    - `Kind::kVoicePre` / `kVoicePost` + `voiceIndex` para módulos basados en voces.
    - `Kind::kSampleplay` para Sampleplay.
  - Se llama a `audioEngine_.configureConnectionWaveformTap(...)` según ese mapa y el `AudioGraph` para tener buffers por conexión.

En paralelo existe documentación de transición hacia **buffers por conexión** y un `AudioGraph` explícito ([research/audio_graph_connection_buffers.md](research/audio_graph_connection_buffers.md)), que ya se ha empezado a implementar (taps por conexión, Loop waveforms), pero el resultado es un sistema híbrido:

- Parte de la UI sigue razonando en términos de **voces** (`moduleVoiceIndex_`).
- Parte razonando en términos de **conexiones** (`connectionWaveformBuffers_`).
- Parte usa **buffers especiales por tipo de módulo** (Loop, Sampleplay).

Esto genera:

- Código complejo en `MainComponent_Paint.cpp` y `MainComponent_Audio.cpp` con muchas ramas por tipo de módulo.
- Dudas sobre qué onda mostrar en cada tramo de una línea (radial, conexión filtrada, etc.).
- Dificultad para extender a nuevos módulos (Delay, Modulator, Waveshaper…) sin añadir más casos especiales.

---

## 2. Modelo objetivo: Voces por módulo y flujo de conexiones

El modelo que se quiere alcanzar, alineado con tu descripción, es:

1. **Cada módulo que puede generar sonido tiene una Voz visual propia**:
   - Oscillator, Loop, Sampleplay, Filter (cuando genera salida propia), Delay, Modulator, Waveshaper, etc.
   - Esa Voz mantiene un **buffer circular mono** con la onda que representa la **salida de audio del módulo** (o del puerto de salida principal).
   - Dos módulos Loop distintos nunca comparten buffer; lo mismo aplica a dos Oscillator, dos filtros, etc.

2. **Las conexiones muestran la onda que circula por ellas**:
   - Tramo desde módulo generador hasta el siguiente nodo: muestra la **onda de salida del módulo origen** (Voz del módulo origen → “onda original”).
   - Tras pasar por filtros/FX/moduladores, la salida unificada genera una **onda distinta** (Voz del módulo de proceso), y esa es la que viaja por sus conexiones salientes.
   - Visualmente se ve una historia coherente: cada vez que la señal pasa por un bloque de proceso, la forma de onda cambia.

3. **Clase dedicada a soundwaves: "Voces"**

Se introduce una capa explícita, por ejemplo:

- Clase interna al core (nombre tentativo): `Voices` o `VisualVoices`.
- Responsabilidades:
  - Gestionar el **registro de Voces** (una por módulo productor de audio).
  - Mantener los **buffers circulares mono** por Voz.
  - Proveer una API uniforme para que el hilo de audio escriba muestras y la UI lea snapshots.
  - Mapear **conexiones → Voz fuente** de forma determinista, apoyándose en `Scene` + `AudioGraph`.
- Esta clase debe estar claramente separada de la lógica de síntesis: `AudioEngine` produce audio, `Voices` sólo observa/copía la señal necesaria para visualización.

4. **Relación con `AudioGraph`**

- `AudioGraph` (ya existente) mantiene nodos (módulos) y aristas (conexiones tipadas).
- Para cada nodo de audio se asocia una **Voz visual**.
- Para cada conexión de audio, la UI puede consultar:
  - Qué Voz corresponde al módulo origen (onda que sale por esa conexión).
  - Opcionalmente, qué Voz corresponde al módulo destino (onda tras procesar), para casos donde interese mostrar entrada/salida en diferentes segmentos.

---

## 3. Fases de refactor

### Fase 0 – Alinear requisitos y acotar alcance

Objetivo: congelar una visión común de “qué debe ocurrir visualmente” antes de tocar código.

Tareas:

- Validar este documento frente a los requisitos de usuario:
  - Onda original visible en la conexión que sale del generador.
  - Onda procesada visible tras filtros/FX, en conexiones siguientes.
  - Un buffer exclusivo por módulo productor de audio.
- Decidir el **nivel de fidelidad** deseado para Sampleplay:
  - Opción A: una única Voz global (como ahora), aceptando que varios Sampleplay compartan onda.
  - Opción B (**GANADOR**): una Voz por SampleplayModule, aunque la síntesis subyacente sea global; la Voz sería una aproximación visual basada en el routing (preferible si la UI necesita distinguir módulos Sampleplay).

Salida de Fase 0:

- Aprobación de este plan (posiblemente con pequeños ajustes de alcance).
- Decisión fijada para Sampleplay (**Opción B**): cada `SampleplayModule`
  tendrá su propia Voz visual dedicada, incluso aunque el DSP siga
  usando un único `SampleplaySynth` compartido. Las Voces de Sampleplay
  se considerarán aproximaciones visuales basadas en routing y ganancia
  efectiva, priorizando la coherencia visual entre módulos frente a la
  fidelidad exacta de mezcla cuando haya varios Sampleplay sonando a la vez.

---

### Fase 1 – Inventario y encapsulación del código actual de soundwaves

Objetivo: **aislar** todo el código que genera y consume soundwaves, sin cambiar todavía el comportamiento observable.

Tareas:

1. **Inventario detallado de puntos de escritura de waveforms en `AudioEngine`**:
   - Dónde se escribe en:
     - `waveformBuffer_` (mezcla global).
     - `voicePreFilterWaveformBuffer_` / `voicePostFilterWaveformBuffer_`.
     - `connectionWaveformBuffers_` (taps).
     - `loopWaveformBuffers_`.
   - Documentar para cada uno:
     - Qué señal se está copiando (pre/post filtro, antes/después de gain, antes/después de mezcla).
     - En qué orden se aplica respecto al resto del pipeline.

2. **Inventario de puntos de lectura en `MainComponent`**:
   - Todas las llamadas a:
     - `getWaveformSnapshot`.
     - `getVoiceWaveformSnapshot` / `getVoiceFilteredWaveformSnapshot`.
     - `getConnectionWaveformSnapshot`.
     - `getLoopModuleWaveformSnapshot`.
   - Para cada uso, anotar:
     - Qué elemento visual está pintando (radial, conexión módulo→módulo, entrada agregada de un filtro, etc.).
     - Qué tipo de módulo y tramo lógico representa.

3. **Encapsulación suave en helpers internos**:
   - Extraer pequeñas funciones/helpers que describan la intención:
     - Ejemplo: `fetchWaveformForRadial(...)`, `fetchWaveformForAudioEdge(...)`, etc.
   - Sin cambiar aún las fuentes de datos, pero reduciendo la dispersión de lógica.

Salida de Fase 1:

- Mapa claro de “quién escribe y quién lee” cada tipo de buffer de waveform.
- Código algo más estructurado, listo para redirigir las lecturas hacia Voces.

Estado actual (22-12-2025):

- Escrituras en `AudioEngine` (`audioDeviceIOCallbackWithContext`):
  - `waveformBuffer_`: recibe la mezcla mono final (canal L) tras sumar
    voces de oscilador post‑filtro + Sampleplay filtrado + Loop filtrado.
  - `voicePreFilterWaveformBuffer_` / `voicePostFilterWaveformBuffer_`:
    se rellenan por voz con `voiceSamplePre` (señal pre‑filtro, ya con
    envolvente y nivel aplicados) y `voiceSamplePost` (post‑filtro), y
    son además la fuente de los taps de conexión `kVoicePre`/`kVoicePost`.
  - `connectionWaveformBuffers_`: se actualiza por tap de conexión, bien
    con `voiceSamplePre`/`voiceSamplePost` (según `ConnectionTapSourceKind`)
    o con la señal Sampleplay mono cruda (`rawSampleplayL`) para taps
    `kSampleplay`.
  - `loopWaveformBuffers_`: por Loop, se escribe el `monoDry` (loop mono
    antes de filtros de bus, pero ya con ganancia/envolvente de Loop)
    usando el índice `visualWaveformIndex` asociado a cada `LoopInstance`.

- Lecturas en `MainComponent`:
  - `computeAudioFrameState` en [core/src/MainComponent_Paint.cpp](core/src/MainComponent_Paint.cpp)
    usa `getVoiceWaveformSnapshot` / `getVoiceFilteredWaveformSnapshot`
    para rellenar `AudioFrameState` (arrays por voz + métricas de normalización
    y RMS por voz).
  - Radiales y segmentos de Loop/Sampleplay en `paint` combinan:
    - `getConnectionWaveformSnapshot` para Sampleplay (radiales y
      conexiones Sampleplay→X cuando hay tap configurado).
    - `getLoopModuleWaveformSnapshot` para radiales y conexiones de Loop
      (Loop→Master, Loop→Filter, etc.).
    - Per‑voice snapshots de `AudioFrameState` para generadores basados en
      voces (Oscillator y cadenas con filtro).
  - Lambdas auxiliares como `fetchConnectionWaveformOrAggregate` y helpers
    internos agregan múltiples conexiones hacia un módulo y calculan
    normalización/RMS para entradas agregadas de filtros.

- Encapsulación inicial:
  - Se ha introducido un helper reutilizable `fetchNormalisedLoopWaveform`
    en [core/src/MainComponent_Paint.cpp](core/src/MainComponent_Paint.cpp) que centraliza la
    lectura y normalización pico‑a‑uno de `getLoopModuleWaveformSnapshot`
    para radiales y segmentos de Loop. Esto elimina duplicación local y
    deja preparado el código para redirigir estas lecturas a Voces en
    fases posteriores.

---

### Fase 2 – Definición de la clase "Voces" y API mínima

Objetivo: introducir una **abstracción explícita de Voces** que pueda convivir temporalmente con el sistema existente.

Diseño propuesto (alto nivel):

- Nueva clase en el core, sin dependencias de JUCE (comentarios en inglés en la implementación):
  - Ficheros tentativos: `core/src/core/Voices.h` y `core/src/core/Voices.cpp`.
- Conceptos principales:

1. **VoiceId / ModuleVoiceId**
   - Identificador estable de la Voz asociada a un módulo productor de audio.
   - Mapeo: `moduleId` → `VoiceId`.

2. **Buffer por Voz**
   - Cada Voz mantiene un buffer circular mono de longitud fija (similar a `kWaveformHistorySize`).
   - API (esbozo conceptual):
     - `void writeSamples(VoiceId id, const float* mono, int numSamples);`
     - `void getSnapshot(VoiceId id, float* dst, int numPoints, double windowSeconds) const;`

3. **Integración con `Scene` / `AudioGraph`**
   - Capa de coordinación (probablemente en `MainComponent` / `MainComponent_Audio.cpp`):
     - Construye el mapa `moduleId → VoiceId` en función de los módulos presentes en la escena.
     - Para conexiones, resuelve qué Voz usar como fuente:
       - Conexión `A → B` → Voz de `A` para la onda original en ese tramo.
       - Para salidas agregadas de filtros/FX, se usa la Voz del módulo de proceso.

4. **Ubicación del código de escritura**
   - Aunque la clase `Voices` viva en `core/src/core`, la escritura real se hace desde el hilo de audio en `AudioEngine`:
     - `AudioEngine` recibe referencias/punteros a `Voices` (o un sub-objeto interno equivalente) y, en los puntos donde ya dispone de la señal mono por módulo, llama a `writeSamples(...)`.
   - El objetivo es **no mezclar** más lógica de ring buffer dispersa en `AudioEngine`: todo el manejo de índices, wrap y snapshots vive en `Voices`.

Salida de Fase 2:

- Clase `Voices` (o equivalente) implementada y probada de forma aislada.
- Sin cambios funcionales en la UI: todavía se usan los buffers actuales, pero ya existe la API nueva.

Estado actual (22-12-2025):

- Se ha introducido la clase `soundtable::Voices` en
  [core/src/core/Voices.h](core/src/core/Voices.h) y
  [core/src/core/Voices.cpp](core/src/core/Voices.cpp):
  - Sin dependencias de JUCE ni de `Scene`/`AudioGraph`.
  - API mínima:
    - `Voices(int maxVoices, int historySize)` para crear el contenedor de
      Voces visuales.
    - `setSampleRate(double)` para que el hilo de audio indique el sample
      rate usado al interpretar `windowSeconds`.
    - `reset()` para limpiar historial y write index.
    - `writeSamples(VoiceId, const float* mono, int numSamples)` para que
      el hilo de audio escriba bloques mono por voz.
    - `getSnapshot(VoiceId, float* dst, int numPoints, double windowSeconds)`
      para que la UI lea ventanas de historial downsampleadas.
  - Implementación basada en un índice de escritura global atómico y un
    buffer plano `[voice][sample]` sin locks, pensada para un escritor
    (audio) y múltiples lectores (UI).

- Integración: `Voices` forma parte de `soundtable-core-lib` vía el glob
  `src/core/*.cpp` en [core/CMakeLists.txt](core/CMakeLists.txt), pero aún no está
  conectada a `AudioEngine` ni consumida por `MainComponent`. El sistema
  existente de waveforms (buffers en `AudioEngine` + taps por conexión)
  sigue siendo la única fuente usada en la UI, cumpliendo el objetivo de
  "cero cambios funcionales" en esta fase.

---

### Fase 3 – Asignar una Voz visual exclusiva a cada módulo productor

Objetivo: que **cada módulo que genera sonido tenga un buffer exclusivo** en Voces, independientemente de cómo esté implementado el DSP por debajo.

Tareas por tipo de módulo:

1. **OscillatorModule**
   - Actualmente: varias instancias de Oscillator se multiplexan sobre `kMaxVoices` en `AudioEngine`, más un mapa `moduleVoiceIndex_`.
   - Propuesta:
     - Mantener la asignación módulo→voz de audio (si el límite de voces es suficiente para los escenarios previstos).
     - En el punto del callback donde ya se calcula la señal pre/post filtro por voz, añadir un paso:
       - Determinar qué módulos están asociados a esa voz en este bloque.
       - Llamar a `Voices::writeSamples(voiceIdDeEseModulo, señalMono, numSamples)`.
     - Asumir en primera instancia **un módulo generador por voz** (documentando la limitación); si más adelante se necesita multiplexar, se refinará la estrategia.

2. **LoopModule**
   - Actualmente ya existe un buffer por Loop (`loopWaveformBuffers_` mapeado por `loopModuleToWaveformIndex_`).
   - Propuesta:
     - Reemplazar progresivamente ese mecanismo por Voces:
       - Al configurar `setLoopModuleParams` / al crear `LoopInstance`, registrar una Voz por módulo.
       - En el punto donde se obtiene la señal mono de Loop antes de mezclar, llamar a `Voices::writeSamples` en lugar de escribir en `loopWaveformBuffers_`.

3. **SampleplayModule**
   - DSP subyacente compartido (`SampleplaySynth`), pero visualmente se quieren distinguir módulos.
   - Propuesta inicial (aproximada, visual):
     - Mantener una Voz global de Sampleplay para la onda “cruda”.
     - Adicionalmente, permitir **Voces lógicas por módulo Sampleplay** que escalen/copian la forma global según la ganancia/routing efectivo del módulo (por ejemplo, copiando la onda global a cada módulo activo con diferente escala de amplitud).
     - Documentar claramente que, a efectos de visual, dos SampleplayModule pueden mostrar la misma forma con distinta ganancia/fase.

4. **Filter, Delay, Modulator, Waveshaper y otros FX**
   - Estos módulos **no generan audio de la nada**, pero sí tienen una salida de audio propia (señal procesada).
   - Propuesta:
     - Asignar una Voz por módulo FX que represente su salida agregada.
     - En el pipeline de audio (cuando exista un nodo explícito en `AudioGraph` o se aproximen como bus), tomar la señal de salida post‑proceso y escribirla en la Voz correspondiente.

Salida de Fase 3:

- Para todos los módulos productores/transformadores de audio relevantes, existe una Voz registrada y actualizada en cada bloque de audio.
- Los buffers antiguos (`loopWaveformBuffers_`, parte de `connectionWaveformBuffers_`, etc.) pueden empezar a marcarse como “en transición” hacia desuso.

Estado actual (22-12-2025):

- **OscillatorModule**
  - Cada voz de oscilador en `AudioEngine` (índices `[0,kMaxVoices)`) escribe ahora su señal post‑filtro, ya con envolvente y nivel aplicados, en la instancia interna `visualVoices_`:
    - En `audioDeviceIOCallbackWithContext`, tras calcular `voiceSamplePost` y almacenarlo en `voicePostFilterWaveformBuffer_`, se llama a `visualVoices_.writeSamples(v, &voiceSamplePost, 1)`.
    - Esto reserva el rango de ids `[0, kMaxVoices)` de `Voices` para voces de Oscillator, de forma que cada voz de audio tiene ya un buffer circular propio en la estructura unificada.
  - La UI sigue usando por ahora `getVoiceWaveformSnapshot` / `getVoiceFilteredWaveformSnapshot`; `visualVoices_` se expone vía `AudioEngine::visualVoices()` pero todavía no se consulta desde `MainComponent`.

- **LoopModule**
  - Se ha extendido la escritura de `loopWaveformBuffers_` para que, además de rellenar el historial por módulo con `monoDry` (señal mono del loop antes del filtro de bus), se escriba en `visualVoices_`:
    - Para cada `LoopInstance` con `visualWaveformIndex` válido, en el callback de audio se llama a
      `visualVoices_.writeSamples(kMaxVoices + wfIndex, &monoDry, 1)`.
    - El rango de ids `[kMaxVoices, kMaxVoices + kMaxLoopWaveforms)` queda reservado para LoopModules, usando el mismo `wfIndex` que ya se emplea para mapear módulos a `loopWaveformBuffers_`.
  - Esto garantiza que cada LoopModule con waveform visual propia dispone ya de una Voz dedicada en `Voices`, actualizada bloque a bloque desde el hilo de audio.

- **SampleplayModule y FX (Filter/Delay/Modulator/Waveshaper, etc.)**
  - Se ha introducido un id de Voz dedicado para la ruta global de Sampleplay (mono pre‑filtro) y otro para la salida del bus de Loop post‑filtro en `visualVoices_` dentro de `AudioEngine`:
    - `kVisualVoiceIdSampleplayRaw = kMaxVoices + kMaxLoopWaveforms` representa la señal mono global de Sampleplay antes del filtro de bus; en el callback se escribe con `rawSampleplayL` en ambas ramas (con y sin voces de Oscillator activas).
    - `kVisualVoiceIdLoopBus = kVisualVoiceIdSampleplayRaw + 1` representa la suma de todos los Loop tras el filtro de bus (`loopFilteredL`), en ambas ramas del callback.
  - Estas Voces son **de nivel bus** (no por módulo) y sirven como fuente común para visuales de Sampleplay y cadenas Loop→Filter/FX; en fases posteriores la UI mapeará cada `SampleplayModule` y cada módulo FX relevante a estas Voces compartidas (o combinaciones de ellas), aceptando que la granularidad es global mientras el DSP siga modelando Sampleplay y Loop como rutas únicas.

- Los buffers legacy (`voicePreFilterWaveformBuffer_`, `voicePostFilterWaveformBuffer_`, `loopWaveformBuffers_`, `connectionWaveformBuffers_`) se mantienen sin cambios y siguen siendo la única fuente consultada por la UI, pero ya se consideran “en transición” para Oscillator y Loop en favor de `Voices`.

---

### Fase 4 – Reescribir la lógica visual de conexiones apoyándose en Voces

Objetivo: que **todas las soundwaves que se dibujan en líneas y radiales** provengan de Voces, eliminando la lógica específica por tipo de buffer.

Tareas:

1. **Nuevo contrato UI para conexiones**
   - Definir una estructura de metadatos visual por conexión, desligada de `voicePre`/`voicePost` y de taps específicos:
     - Ejemplo conceptual:
       - `struct ConnectionWaveformDescriptor {`  
         `  ModuleVoiceId sourceVoice;`  
         `  ModuleVoiceId processedVoice; // opcional`  
         `  // flags para elegir si la UI usa source vs processed`  
         `};`
   - Esta estructura se construye a partir de `Scene` + `AudioGraph` + routing actual.

2. **Simplificar `connectionVisualSources_`**
   - Evolucionar `ConnectionVisualSource` para que deje de hablar en términos de `kVoicePre/kVoicePost` y pase a expresar directamente qué Voz (o Voces) se deben usar.
   - Mantener una fase de transición donde el tipo antiguo coexista con el nuevo mientras se migra la lógica de pintado.

3. **Refactor de `MainComponent_Paint.cpp`**
   - Reemplazar progresivamente:
     - Llamadas a `getVoiceWaveformSnapshot` / `getVoiceFilteredWaveformSnapshot`.
     - Llamadas a `getConnectionWaveformSnapshot`.
     - Llamadas a `getLoopModuleWaveformSnapshot`.
   - Por llamadas genéricas a la clase Voces (o a wrappers que ya delegan en Voces desde `AudioEngine`, como el caso de Loop y las voces de Oscillator a partir de esta fase):
     - `voices.getSnapshot(sourceVoice, ...)` para conexiones de salida de generadores.
     - `voices.getSnapshot(processedVoice, ...)` para salidas de filtros/FX.
   - Unificar el tratamiento de:
     - Radiales módulo→master.
     - Conexiones módulo→módulo.
     - Segmentos con hold‑mute (misma onda pero con opacidad/distancia distinta, no con fuentes distintas).

4. **Eliminar heurísticas duplicadas**
   - Retirar ramas que deciden “si el módulo origen es un Filter entonces usa post‑filtro; si es generador, usa pre‑filtro…”.
   - Estas decisiones pasan a estar encapsuladas en la construcción de `ConnectionWaveformDescriptor` y en la asignación de Voces.

Salida de Fase 4:

- Las soundwaves visibles se basan **exclusivamente en Voces por módulo**.
- Las estructuras legacy de waveforms en `AudioEngine` siguen existiendo por compatibilidad, pero ya no son consultadas por la UI para Oscillator, Loop ni Sampleplay; las conexiones y radiales usan ahora snapshots derivados de `visualVoices_` (per‑voice, Loop por módulo y bus global de Sampleplay).

---

### Fase 5 – Eliminación de código legacy y consolidación

Objetivo: limpiar el código eliminando buffers y APIs ya obsoletos, sin cambiar el comportamiento introducido en las fases anteriores.

Tareas:

1. **Deprecación y eliminación controlada de APIs en `AudioEngine`**
   - Eliminar gradualmente:
     - `getWaveformSnapshot` (si ya no se usa en UI).
     - `getVoiceWaveformSnapshot` / `getVoiceFilteredWaveformSnapshot`.
     - `getConnectionWaveformSnapshot` y el sistema de `ConnectionTap`.
     - `getLoopModuleWaveformSnapshot` y `loopWaveformBuffers_`.
   - Eliminar los buffers `waveformBuffer_`, `voicePreFilterWaveformBuffer_`, `voicePostFilterWaveformBuffer_`, `connectionWaveformBuffers_`, `loopWaveformBuffers_` cuando ya no haya referencia a ellos.

2. **Simplificación en `MainComponent`**
   - Podar código de pintado que sólo tenía sentido en el modelo antiguo.
   - Unificar helpers y dejar una única vía de obtención de ondas para la UI.

3. **Tests y regresiones**
   - Ejecutar `ctest --output-on-failure` tras cada bloque de limpieza significativo.
   - Validación manual de escenas típicas:
     - Osc → Filter → Master.
     - Cadena con Delay/Modulator/Waveshaper.
     - Loops múltiples + Sampleplay.

Salida de Fase 5:

- Sistema de soundwaves basado únicamente en Voces.
- Menos código, menos condicionales y puntos claros para añadir nuevos tipos de módulo.

Estado actual (22-12-2025):

- Se ha eliminado la infraestructura de taps por conexión en `AudioEngine` (`ConnectionTapSourceKind`, `clearAllConnectionWaveformTaps`, `configureConnectionWaveformTap`, `getConnectionWaveformSnapshot`, `connectionWaveformBuffers_` y el contador/tabla de taps), de modo que el motor sólo mantiene historiales de waveform a través de `visualVoices_`.
- También se han retirado los buffers legacy de mezcla global (`waveformBuffer_`/`getWaveformSnapshot`) y el ring buffer por Loop (`loopWaveformBuffers_`), conservando únicamente `getVoiceWaveformSnapshot`, `getVoiceFilteredWaveformSnapshot`, `getLoopModuleWaveformSnapshot` y `getSampleplayBusWaveformSnapshot` como wrappers finos que delegan en Voces.
- En `MainComponent_Audio.cpp` se ha podado la sección que reconstruía taps de conexión en cada tick usando `connectionVisualSources_`; esta estructura sigue existiendo pero ahora sólo se usa en `MainComponent_Paint.cpp` para decidir qué Voz (o combinación de Voces) se samplea al pintar radiales y conexiones.

---

## 4. Consideraciones y riesgos

- **Límite de voces vs número de módulos**:
  - Hoy `kMaxVoices` limita cuántos Oscillator pueden sonar en paralelo.
  - Al asociar una Voz visual por módulo, hay que documentar y/o adaptar este límite (p. ej. garantizando que en escenas típicas no se supera, o ampliándolo si es razonable).

- **Desfase entre audio real y visual**:
  - La clase Voces seguirá usando buffers circulares con snapshots downsampleados, como ahora.
  - Es importante mantener la escritura desde el hilo de audio y lecturas lock‑free desde UI.

- **Sampleplay compartido**:
  - Si finalmente se decide dar una Voz por módulo Sampleplay, habrá que aceptar que la precisión no será perfecta cuando varios módulos disparen notas solapadas.
  - La prioridad es la **coherencia visual** más que la fidelidad DSP exacta en este caso.

- **Impacto en rendimiento**:
  - Habrá más buffers (uno por módulo), pero menos lógica dispersa y menos taps por conexión.
  - El plan debe cuidar que las escrituras adicionales sean lineales y cache‑friendly.

---

## 5. Próximos pasos

Para comenzar la implementación:

1. Completar el inventario detallado de Fase 1 (documentando exactamente qué se escribe/lee en cada buffer actual).
2. Diseñar la interfaz concreta de `Voices` (nombres de métodos, estructuras, ownership) y crear el esqueleto en `core/src/core/Voices.h/.cpp`.
3. Introducir la asignación módulo→Voz para Oscillator y Loop, validando que la UI puede empezar a consumir snapshots desde Voces sin cambiar todavía el aspecto visual.

Una vez que estas piezas estén estables, se podrá abordar la migración completa de la lógica de conexiones y la eliminación progresiva del código legacy.
