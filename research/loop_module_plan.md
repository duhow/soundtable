# Plan de implementación: LoopModule

Este documento describe el plan para implementar el **Loop Module** en RectaiTable, incluyendo carga de samples, reproducción de loops, sincronía con BPM y la integración UI básica solicitada.

## 1. Objetivos funcionales

- Cada `LoopModule` puede gestionar **hasta 4 samples de audio**.
- Los samples se cargan desde `com.reactable/Samples` (recursivo), usando los nombres indicados en las entradas `<loop>` del `.rtp` cuando estén disponibles.
- Formatos de entrada: **WAV, FLAC, Ogg Vorbis y Opus**, usando el soporte de formatos de audio de JUCE (sin añadir dependencias nuevas fuera de JUCE).
- Los loops se reproducen **en bucle continuo**; mutear la línea de audio (conexión a Master) **no pausa el loop**, solo silencia su salida hacia el bus master.
- Opción de **sincronizar la velocidad de reproducción con el BPM global** (stretch en tiempo, cambiando pitch de forma simple en esta fase).
- UI: la barra de *freq* del Loop se representa como una barra segmentada en 4, con un triángulo selector controlado por la rotación OSC/Tracker, y posibilidad de cambiar de sample haciendo click en el segmento.
- Al cambiar de sample activo, mostrar el **nombre de archivo** a la derecha del módulo durante ~5 segundos con un fade-out suave.

## 2. Diseño de datos y modelo

### 2.1. `LoopModule` (modelo de dominio)

- Mantener la estructura actual en `core/src/core/AudioModules.{h,cpp}`:
  - `Envelope envelope_`.
  - `std::vector<LoopDefinition> loops_` con `beats`, `filename`, `order`.
- No introducir tipos dependientes de JUCE en la librería de dominio.
- Añadir, si es necesario, **helpers ligeros**:
  - Métodos para recuperar hasta 4 loops normalizados por `order` (0..3).
  - Helper para mapear el parámetro normalizado `"sample"` (0..1) a un índice entero `[0,3]`.

### 2.2. Representación runtime de loops en `AudioEngine`

- En `core/src/AudioEngine.{h,cpp}` añadir una estructura interna para los samples de loop:

  - `LoopSample` (interno a `AudioEngine`):
    - `std::vector<float> left, right` (buffer estéreo ya decodificado).
    - `int numSamples`.
    - `int sourceSampleRate`.
    - `int beats` (copiado de `LoopDefinition`).
    - `double lengthSeconds`.
  - `LoopInstance` (por módulo):
    - `std::string moduleId`.
    - Hasta 4 `LoopSample`.
    - `std::atomic<int> selectedIndex` (0..3).
    - `std::atomic<float> gain` (lineal 0..1, incluye master/gain del módulo).
    - `std::array<double,4> readPosSamples` (solo hilo de audio).

- Mantener una colección de instancias:
  - `std::unordered_map<std::string, std::shared_ptr<LoopInstance>> loopModules_`.
  - Un `std::shared_ptr<const std::vector<std::shared_ptr<LoopInstance>>> loopSnapshot_` accesible lock-free desde el callback de audio (double buffering simple).

## 3. APIs nuevas en `AudioEngine`

Añadir métodos públicos (solo usados desde `MainComponent`):

1. `bool loadLoopSampleFromFile(const std::string& moduleId, int slotIndex,
                                const std::string& path, int beats,
                                std::string* error = nullptr);`
   - Usa `juce::AudioFormatManager` para abrir el fichero (WAV, FLAC, Ogg, Opus).
   - Lee y convierte el audio a estéreo float (si es mono, duplicar a L/R; si tiene más canales, usar los dos primeros).
   - Rellena/actualiza la `LoopInstance` correspondiente a `moduleId`.
   - Calcula `lengthSeconds` y almacena `beats`.

2. `void setLoopModuleParams(const std::string& moduleId,
                             int selectedIndex,
                             float linearGain);`
   - Actualiza `selectedIndex` y `gain` de la `LoopInstance` asociada.
   - No resetea `readPosSamples`, de modo que el loop sigue su fase aunque la salida esté muteada.

3. `void setLoopGlobalTempo(double bpm);`
   - Guarda el BPM global en un `std::atomic<double> loopGlobalBpm_`.

### 3.1. Integración en el callback de audio

En `audioDeviceIOCallbackWithContext`:

- Tras mezclar osciladores y Sampleplay, sumar la contribución de todos los loops activos:
  - Obtener `auto snapshot = loopSnapshot_` una vez por callback.
  - Para cada `LoopInstance` en `snapshot`:
    - Leer `selectedIndex` y `gain` (atomics, relajado).
    - Si `gain <= 0` pero el módulo tiene loops cargados, **seguir avanzando `readPosSamples`** sin sumar al output.
    - Para cada frame:
      - Calcular `step` (incremento por muestra) usando:
        - `sourceSampleRate`, `sampleRate_` (dispositivo) y `lengthSeconds`.
        - BPM global: `loopGlobalBpm_` y `beats`.
        - Fórmula aproximada (time-stretch sencillo, con cambio de pitch):

          $$ step = \frac{sourceSampleRate}{sampleRate_} \times
                   \frac{lengthSeconds \times bpm}{beats \times 60} $$

        - Si `beats <= 0` o `bpm <= 0`, usar `step = sourceSampleRate / sampleRate_` (sincronía desactivada).
      - Leer sample con interpolación lineal en `readPosSamples[slot]`.
      - Hacer wrap en `readPosSamples` cuando alcance `numSamples`.
      - Sumar la muestra escalada por `gain` al bus estéreo (después de Sampleplay, antes del clip final).

- Evitar locks en el callback:
  - `loopSnapshot_` se actualiza solo desde el hilo de UI creando una copia nueva cuando cambia la topología de loops.

## 4. Integración con `MainComponent` (carga y audio)

### 4.1. Carga de samples de Loop

En el constructor de `MainComponent` (tras `loadDefaultPatch()` y `refreshInsideMusicAreaFlags()`):

- Añadir un helper `loadLoopSamples()` similar a `loadSampleplaySoundfonts()`:
  - Iterar `scene_.modules()`.
  - Para cada `LoopModule`:
    - Tomar hasta 4 entradas de `loop->loops()` ordenadas por `order` ascendente.
    - Construir la ruta absoluta usando `rectai::ui::loadFile("Samples/" + filename)`.
    - Si el fichero existe, llamar a `audioEngine_.loadLoopSampleFromFile(module->id(), slotIndex, path, beats, &error)`.
  - Manejar el caso `filename` vacío:
    - O bien ignorarlo (slot sin audio), o bien asignar desde un pool genérico (`com.reactable/Samples` escaneado recursivamente). Para el MVP, basta con ignorar para no complicar el descubrimiento global.

- Invocar `loadLoopSamples()` después de `loadSampleplaySoundfonts()`.

### 4.2. Mapeo de Scene → parámetros de Loop en `timerCallback`

En `MainComponent::timerCallback` (después de configurar Sampleplay):

- Introducir un bloque específico para módulos `LoopModule`:
  - Iterar objetos y módulos.
  - Para cada `LoopModule` dentro del área musical:
    - Determinar si su ruta efectiva hacia el master está muteada:
      - Buscar la conexión auto-generada `Loop → Output(-1)` en `audioEdges`.
      - Consultar `mutedConnections_` como en Sampleplay.
    - Calcular el `amp` efectivo del módulo (`GetParameterOrDefault("amp", 1.0)`), aplicando la misma curva de volumen global que para Sampleplay/Oscillator (master Volume incluido).
    - Derivar un `linearGain` final (0..1) para el loop.
      - Si el radial está muteado hacia master, usar `linearGain = 0` pero seguir reportando el módulo a `AudioEngine` para que su fase avance.
    - Calcular el índice de sample activo:
      - Leer `sampleParam = GetParameterOrDefault("sample", 0.0F)`.
      - Mapear a `[0,3]`: `index = juce::jlimit(0,3, int(std::floor(sampleParam * 4.0F + 1e-4F)))`.
    - Llamar a `audioEngine_.setLoopModuleParams(module->id(), index, linearGain)`.

- Mantener `modulesWithActiveAudio_` actualizado:
  - Insertar el `id` del `LoopModule` cuando `linearGain > 0` y tenga al menos un slot cargado.

- Actualizar `AudioEngine::setLoopGlobalTempo(bpm_)` una vez por tick.

## 5. UI del LoopModule

### 5.1. Barra de freq segmentada y triángulo selector

- En `MainComponent_Paint.cpp`, dentro del bloque de dibujo de la barra de frecuencia (control izquierdo):
  - Detectar `moduleForObject->is<rectai::LoopModule>()`.
  - En lugar de pintar un arco continuo:
    - Dividir la barra en 4 segmentos verticales (mismo arco base) separados por un pequeño espacio (gap).
    - Para cada segmento `i` (0..3):
      - Pintar el fondo del segmento con una opacidad suave.
      - Si `i == selectedIndex` (derivado del parámetro `"sample"`), usar un color de primer plano más intenso.
  - Dibujar un **triángulo** pequeño en el extremo izquierdo del arco (coordenadas locales del módulo), posicionado verticalmente sobre el segmento seleccionado.
    - El triángulo se calcula en el sistema de coordenadas ya rotado del módulo (ya está el `AffineTransform` aplicado).

### 5.2. Interacción por click (debug)

- En `MainComponent_Input.cpp`, en el bloque de interacción con la barra de freq/gain:
  - Cuando el módulo es `LoopModule` y el click cae en el área de la barra izquierda:
    - Determinar el segmento (`0..3`) en el que ha caído el click (por Y dentro del arco segmentado).
    - Asignar el parámetro `"sample"` del módulo: `scene_.SetModuleParameter(id, "sample", segmentIndex / 4.0F)`. 
    - Llamar a un nuevo helper `markLoopSampleLabelActive(module->id())`.
    - Forzar `repaint()`.

### 5.3. Selección por rotación OSC/Tracker

- En `MainComponent::rotationTrackingUpdate(...)`:
  - Extender la lógica que actualmente ajusta `freq`/`gain` en función de la rotación para que, cuando el módulo sea `LoopModule`, la rotación afecte principalmente al parámetro `"sample"`:
    - Mapear el ángulo normalizado (0..2π) a 4 sectores.
    - Actualizar `"sample"` de forma discreta (snap por sector) evitando jitter.
    - Cada cambio de sector llama a `markLoopSampleLabelActive(moduleId)`.

### 5.4. Etiqueta con nombre de fichero y fade-out

- En `MainComponent.h`:
  - Añadir `std::unordered_map<std::string, double> loopLabelLastChangeSeconds_;`.
  - Añadir método privado `void markLoopSampleLabelActive(const std::string& moduleId);`.

- En `MainComponent.cpp`:
  - Implementar `markLoopSampleLabelActive` de forma análoga a `markSampleplayInstrumentLabelActive`.

- En `MainComponent_Paint.cpp`, tras el dibujo del nodo (similar a la etiqueta de Sampleplay):
  - Para módulos `LoopModule`:
    - Determinar el loop seleccionado (índice) y su `filename` (desde `LoopModule::loops()`).
    - Extraer solo el nombre base (sin ruta).
    - Dibujar una etiqueta a la derecha del nodo con el nombre del fichero.
    - Usar `loopLabelLastChangeSeconds_` para controlar la visibilidad:
      - Visible al 100% durante 5s.
      - Fade lineal rápido (p.ej. 0.5s) hasta 0.

## 6. Sincronía con BPM y mute

- Reglas:
  - Los loops **siempre avanzan** en fase, independientemente del estado de mute de la conexión a master.
  - El mute de la línea radial (Loop → Output) solo afecta a la **ganancia de salida** hacia el master.
  - El cálculo de `linearGain` en `timerCallback` aplica:
    - `amp` del módulo (`LoopModule::amp`).
    - Curva de master Volume.
    - Mute de la conexión Loop → Output(-1).
  - El BPM se obtiene de `bpm_` (ya sincronizado con `TempoModule` cuando existe) y se pasa a `AudioEngine` vía `setLoopGlobalTempo`.

## 7. Tests y validación

- Verificar compilación y ejecución local:
  - Carga de `research/Loopdemo.rtp` vía `default.rtp` o escena equivalente.
  - Confirmar en runtime (debug logs) que `loadLoopSampleFromFile` abre correctamente `Demoloops/*.wav`.
  - Validar que:
    - Mover un Loop dentro del área musical da audio en bucle.
    - Mutear la línea radial Loop → Master silencia el audio pero, al desmutear, el loop continúa donde estaba.
    - Rotar el tangible cambia de sample (cuando hay ficheros distintos) y la etiqueta de nombre aparece y se desvanece.
    - El cambio de BPM (TempoModule / UI) afecta a la velocidad del loop (aunque implique cambio de pitch).

## 8. Actualización de documentación

- Actualizar `research/progress.md` al finalizar:
  - Describir brevemente:
    - La introducción de reproducción de loops basada en ficheros.
    - La integración con BPM y mute.
    - Los cambios de UI (barra segmentada, triángulo selector, etiqueta con nombre).
