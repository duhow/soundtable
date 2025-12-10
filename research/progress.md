# Progreso de implementación

## 2025-12-09


### Servicio de tracking: modo sintético vs live y esqueleto de engine
- El binario `rectai-tracker` se ha estructurado para soportar dos modos de ejecución diferenciados:
  - Modo `--mode=synthetic` (por defecto), que conserva el comportamiento anterior: envía un único objeto sintético `/rectai/object` con `trackingId=1` y `logicalId="osc1"` moviéndose horizontalmente.
  - Modo `--mode=live`, que inicializa un nuevo `rectai::tracker::TrackerEngine` y pasa cada frame capturado para que devuelva una lista de `TrackedObject` con coordenadas normalizadas y ángulo en radianes.
- Se ha introducido un pequeño modelo de tracking en `tracker/src/TrackerTypes.h` (`TrackedObject` + alias `TrackedObjectList`) y un esqueleto de `TrackerEngine` en `tracker/src/TrackerEngine.{h,cpp}` con API:
  - `bool initialise(int cameraIndex, int requestedWidth, int requestedHeight, std::string& errorMessage)` para preparar parámetros internos.
  - `TrackedObjectList processFrame(const cv::Mat& frame) const` para extraer objetos de cada frame (por ahora implementación placeholder sin detección real).
- `tracker/src/main.cpp` ahora:
  - Parsea `--mode=synthetic` / `--mode=live` desde línea de comandos.
  - En modo live, inicializa `TrackerEngine` con la resolución actual de la cámara y, en cada frame, obtiene una lista de objetos y envía un mensaje `/rectai/object` por cada uno, usando una función `mapLogicalId(int markerId)` para traducir ids numéricos a `logicalId` (por ahora mapeo fijo `1→osc1`, `2→filter1`).
  - Mantiene el cálculo de FPS y los mensajes de log con prefijo `[rectai-tracker]`.
- `tracker/CMakeLists.txt` se ha actualizado para compilar e incluir los nuevos archivos de tracking en el ejecutable `rectai-tracker`.
  - `Connection` y operaciones de alta coherencia sobre la escena.
- Tests sencillos en `tests/scene_tests.cpp` que validan:
  - Inserción única de módulos.
  - Creación y eliminación de conexiones.
  - Semántica de upsert/borrado de `ObjectInstance`.
- UI JUCE mínima (`rectai-core`):
  - `MainComponent` pinta una escena de ejemplo con dos módulos (`osc1`, `filter1`).
  - Dibujo de objetos como círculos y conexiones como líneas entre ellos.
  - Interacción básica de ratón: click-and-drag sobre los círculos actualiza la posición normalizada de los objetos en la `Scene` y repinta.

### Servicio de tracking (skeleton)
- Binario `rectai-tracker` basado en OpenCV (`tracker/src/main.cpp`):
  - Abre la cámara por defecto.
  - Bucle principal que lee frames y calcula FPS aproximados cada segundo.
  - Mensajes de log en inglés con prefijo `[rectai-tracker]`.
  - Emisor OSC mínimo (`OscSender`) que envía mensajes `/rectai/object` sintéticos
    hacia `localhost:3333` para ejercitar el flujo tracking → core → UI incluso
    sin detección real de marcadores.

### Integración inicial de protocolo de tracking (OSC)
- Añadido módulo de JUCE `juce_osc` a la aplicación principal (`core/CMakeLists.txt`).
- Nueva clase `TrackingOscReceiver` en `core/src/TrackingOscReceiver.{h,cpp}`:
  - Usa `juce::OSCReceiver` con callbacks en el message loop.
  - Escucha en el puerto UDP `3333` (alineado con configuraciones típicas de TUIO).
  - Protocolo OSC minimalista para MVP:
    - `/rectai/object` (int32 trackingId, string logicalId, float x, float y, float angleRadians) → `Scene::UpsertObject`.
    - `/rectai/remove` (int32 trackingId) → `Scene::RemoveObject`.
  - Mensajes de log en caso de fallo al bindear el puerto o recibir argumentos inválidos.
- `MainComponent` mantiene ahora un miembro `TrackingOscReceiver trackingOscReceiver_{scene_, 3333};` que conecta la `Scene` con el receptor OSC.

### Motor de audio (esqueleto)
- Añadido `AudioEngine` en `core/src/AudioEngine.{h,cpp}`:
  - Envuelve un `juce::AudioDeviceManager` inicializado con las salidas por defecto (estéreo, sin entradas).
  - Implementa `juce::AudioIODeviceCallback` para generar un tono senoidal de prueba (A4, 440 Hz) a nivel bajo, validando la canalización de audio.
  - Punto de partida para evolucionar hacia un grafo de módulos que mapee `rectai::ModuleType`/`AudioModule` a nodos de procesamiento.
- `RectaiApplication` instancia un `AudioEngine` de larga vida para que el audio se active junto con la aplicación JUCE.

### Diseño de UI rítmica inicial
- Fondo y mesa:
  - Lienzo circular centrado con gradiente azul (más claro en el centro, más oscuro hacia el borde), simulando la "superficie" de la mesa reactiva descrita en `ui-interface.md`.
  - Ligeras viñetas y contorno para reforzar la sensación de proyección de luz.
- Pulso central:
  - Círculo blanco constante en el centro de la mesa.
  - Ondas concéntricas (anillos que se expanden y se desvanecen) sincronizadas con el tempo (BPM), con el primer pulso de cada 4 más grande y grueso (marcando el compás 4/4).
- Objetos/nodos:
  - Renderizados como círculos con un aura luminosa (dos halos concéntricos suaves) bajo cada objeto.
  - Cada nodo muestra un arco de parámetro alrededor (actualmente mapeado a la posición X normalizada) con un punto brillante indicando el valor actual.
  - La paleta sigue la guía de colores: fondo azul oscuro, nodos activos en azul eléctrico y UI en blanco con distintos niveles de transparencia.
- Conexiones:
  - Todos los objetos/instrumentos se conectan visualmente con el centro mediante líneas blancas.
  - Línea continua si el objeto está activo y línea discontinua si el objeto está en estado mute.
  - Click sobre una línea centro→objeto alterna el mute del instrumento, que a su vez controla el nivel del tono generado por `AudioEngine`.

### Limpieza de idioma y comentarios
- Comentarios de `Scene.h` traducidos al inglés para seguir la convención del proyecto.
- Título de la UI en `MainComponent::paint` actualizado a "RectaiTable - Example scene".

## 2025-12-10

### AudioModules y metadata unificada
- `rectai::Scene` ahora almacena instancias polimórficas de `AudioModule`, cada una con:
  - `ModuleType` (SEQUENCER, AUDIO, GENERATOR, FILTER, SETTINGS) para clasificar familias lógicas.
  - Capacidades de audio (`produces_audio`, `consumes_audio`) y políticas de conexión overrideables por módulo.
  - Metadatos UI (`colour_argb`, `label`, `description`, `icon_id`) más flags para reutilizar los sliders de frecuencia y ganancia según convenga.
- Nuevos módulos concretos:
  - `OscillatorModule`: generador con salida audio, sliders de freq/gain activos, color azul y conexión permitida hacia módulos FILTER/AUDIO.
  - `FilterModule`: módulo de paso con entrada/salida audio, sliders de cutoff/gain activos y política de conexión abierta.
- `MainComponent` consume ahora esta metadata para:
  - Pintar colores/aura basados en el color declarado por cada `AudioModule`.
  - Mostrar u ocultar los sliders laterales en función de `uses_frequency_control` / `uses_gain_control`.
  - Renderizar iconos mediante `icon_id` y etiquetar cada nodo con `label (logical_id)`.
  - Consultar valores por defecto de parámetros (`freq`, `gain`, etc.) a través de `AudioModule::default_parameter_value`, de modo que los `0.5F` por defecto viven en `AudioModules.cpp` en lugar de estar duplicados en `MainComponent`.
- `tests/scene_tests.cpp` se actualiza para instanciar módulos concretos (`OscillatorModule`, `FilterModule`) usando `std::unique_ptr`, reflejando la API `Scene::AddModule(std::unique_ptr<AudioModule>)`.
- Nueva cabecera `core/src/core/AudioModules.{h,cpp}` centraliza las implementaciones concretas para que puedan reutilizarse tanto por la UI como por futuros motores de audio.

### Refinado de interfaz visual según `ui-interface.md`
- Fondo y núcleo central:
  - Se mantiene el lienzo circular azul con gradiente y viñeta, actuando como superficie principal de la mesa reactiva.
  - El núcleo central incorpora ahora ondas expansivas más rítmicas y se usa como punto de convergencia visual de las conexiones.
- Nodos/objetos tangibles:
  - Cada `ObjectInstance` toma su color base del metadato `colour_argb()` expuesto por el `AudioModule` asociado (oscillator, filter, effect, sampler, controller), manteniendo la paleta de azules, verdes, púrpuras y naranjas para diferenciar tipos.
  - El aura bajo cada nodo está tintada con el color del módulo y se dibujan dos halos concéntricos suaves, aproximando el efecto de glow/bloom descrito en `ui-interface.md`.
  - Dentro del cuerpo del nodo se renderizan iconos estilizados según el tipo de módulo (onda sinusoidal para osciladores, curva tipo filtro, barras estilo sampler, anillos para controladores, etc.).
- Anillos de parámetros:
  - Se ha sustituido el único arco de parámetro por dos anillos concéntricos: uno interior que representa un valor derivado de la posición X del objeto y otro exterior que actúa como barra de progreso circular ligada a la fase de tempo global.
  - Ambos anillos se orientan usando la rotación (`angle_radians`) del objeto, de forma que los parámetros “siguen” la orientación física del tangible sobre la mesa.
- Conexiones y flujo de señal:
  - Las conexiones entre módulos (`Scene::connections`) se dibujan como curvas de Bézier suaves.
  - Se añade un pequeño pulso luminoso que recorre cada conexión con un desfase por índice, simulando el flujo de audio entre módulos además de las líneas centro→objeto que ya existían.
- Widgets especiales:
  - El widget de secuenciador (grid de puntos) se posiciona ahora utilizando la rotación del objeto cuya `logical_id` empieza por `"seq"`, proyectando la cuadrícula hacia la dirección del cubo físico.
  - Se mantiene el menú radial de parámetros alrededor del objeto seleccionado, alineado con la estética de menús radiales flotantes descrita en `ui-interface.md`.

### Flujo de señal por sectores y mute por instrumento
- Conexiones espaciales entre instrumentos por sectores:
  - La heurística `isConnectionGeometricallyActive` ahora divide el área musical en cuatro sectores angulares alrededor del centro (90º cada uno en coordenadas normalizadas de la mesa).
  - Dos instrumentos se consideran conectados (p.ej. `osc1 → filter1`) solo si ambos caen en el mismo sector; en ese caso se dibuja la línea curvada entre ellos con su pulso animado.
- Rutas hacia Master y líneas individuales:
  - Cada instrumento mantiene siempre su línea directa `instrumento → Master`, independientemente de si está conectado a otro. Esto permite que cada fiducial tenga un control de mute propio, visible y clickable en todo momento.
- Mute independiente pero con efecto en la cadena de audio:
  - Cada instrumento gestiona su propio estado de mute (click sobre su línea al centro), de forma independiente de las conexiones activas.
  - En el caso `osc1`/`filter1`, cuando hay conexión activa en el mismo sector, el audio que se oye sigue siendo la cadena oscilador→filtro→Master, pero si cualquiera de los dos instrumentos está en estado mute, la salida conjunta se silencia.
  - Cuando no hay conexión activa por sector, solo el mute del oscilador afecta al audio generado (el filtro no forma parte de la ruta audible en ese caso, aunque su estado de mute se conserve para cuando vuelva a entrar en cadena).

### Ajuste de conexiones según área de música
- Las conexiones visuales `instrumento → instrumento` (curvas con pulso) solo se consideran activas y se dibujan cuando **ambos** instrumentos están dentro del círculo de música y el destino cae dentro de un cono desde el centro en dirección al origen (inicialmente 90º, ampliado posteriormente a 120º).
- Si cualquiera de los dos objetos sale fuera del área de música, la conexión desaparece visualmente y deja de participar en el routing lógico:
  - En el caso `osc1 → filter1`, si el filtro queda fuera del círculo, la cadena osc→filtro se considera no válida y el audio se silencia para evitar una ruta incoherente hacia el Master.
- El hit-test para mutear conexiones entre instrumentos también respeta esta regla: solo se puede mutear una conexión clicando sobre ella si ambos instrumentos están dentro del área de música.

### Ajuste del ángulo del cono de conexión
- Se ha ampliado el cono geométrico de conexión entre instrumentos de 90º a 120º alrededor del origen, de modo que `isConnectionGeometricallyActive` considera ahora activas más configuraciones espaciales entre módulos `from` y `to`, manteniendo la misma lógica de sectorización pero con mayor tolerancia angular.

### Alineación del área de música con la UI
- La función de utilidad `isInsideMusicArea` se ha movido a `MainComponent` y ahora calcula el área musical en coordenadas de píxel usando exactamente el mismo centro y radio que el círculo de la mesa renderizado en `paint`.
- Esto garantiza que un instrumento se desactive justo cuando su nodo sale visualmente del círculo azul (zona con fondo negro), independientemente de la relación de aspecto de la ventana.

### Próximos pasos sugeridos (UI)
- Introducir un efecto de glow/bloom real mediante un pipeline de renderizado con OpenGL o similar, en lugar de aproximarlo solo con halos y transparencias.
- Experimentar con blending aditivo global para que las intersecciones de líneas y halos incrementen el brillo de forma más marcada.
- Hacer que algunos parámetros visuales (arcos, intensidad de auras, velocidad de pulsos) respondan a métricas de audio reales en `AudioEngine` (niveles RMS, envolventes, etc.).

## Próximos pasos sugeridos (Roadmap corto)
- **Tracking/protocolo**:
  - Extender `rectai-tracker` para enviar mensajes OSC reales compatibles con el protocolo `/rectai/object` y `/rectai/remove` (más allá de los datos sintéticos actuales).
  - Evaluar si se quiere aproximar al formato TUIO estándar o mantener este canal OSC simplificado como modo de debug.
- **Audio**:
  - Evolucionar `AudioEngine` desde el tono único actual hacia varias voces/módulos (p.ej., una voz por `ObjectInstance` asociado a un `OscillatorModule`).
  - A medio plazo, introducir un `AudioProcessorGraph` o equivalente que mapee `AudioModuleType` a nodos de procesamiento reutilizables.
- **UI/Escenas**:
  - Completar la deserialización (carga) de escenas usando el formato line-based `rectai_scene_v1` de `SceneSerialization` o migrar a un formato JSON cuando se introduzca una librería dedicada.
  - Añadir comandos básicos a la UI para guardar la `Scene` actual en disco y recargarla (presets simples).

### Corrección visual de mute en cadenas Oscilador → Filtro
- Ajustada la lógica de dibujo de líneas centro→objeto en `MainComponent` para que los instrumentos silenciados mantengan siempre visible su línea discontinua hacia el centro, incluso cuando están alimentando a otro módulo mediante una conexión activa.
- Ahora, si un oscilador está en mute pero conectado a un filtro, su línea hacia el Master sigue mostrándose como discontinua, evitando que la “línea silenciada” desaparezca cuando se configura la cadena `osc1 → filter1`.

### Cambio de mapeo de frecuencia del oscilador
- La frecuencia del oscilador deja de depender de la posición X del objeto en la mesa y pasa a controlarse únicamente mediante el parámetro `freq` del módulo (`slider` lateral en la UI).
- En `MainComponent::timerCallback`, la frecuencia se calcula ahora como un rango fijo (aprox. 200–1000 Hz) derivado solo del valor del slider, manteniendo la interacción tangible para mute/posición dentro del área musical pero sin afectar al pitch.

### Visibilidad de líneas y rutas activas hacia Master
- Ajustadas las conexiones centro→objeto en `MainComponent` para que **todas** las líneas de instrumentos permanezcan siempre visibles mientras el objeto esté dentro del área musical; cada instrumento conserva su propia línea hacia el centro.
- Cuando un instrumento alimenta a otro mediante una conexión espacial activa, su línea se dibuja ligeramente atenuada y sin pulso animado, de modo que solo el instrumento aguas abajo (p.ej. el filtro) muestra el pulso hacia Master, reflejando mejor la ruta de audio efectiva sin perder el control visual de mute por instrumento.

### Enrutado visual y mute de conexión Oscilador → Filtro
- Cuando el oscilador (`osc1`) tiene una conexión espacialmente activa hacia el filtro (`filter1`), deja de dibujarse la línea directa oscilador→Master (centro); solo el filtro mantiene la línea activa hacia el núcleo central, reforzando visualmente que la ruta efectiva pasa por el filtro.
- Se ha introducido un estado de mute por conexión (`mutedConnections_` en `MainComponent`) indexado por un identificador estable de `rectai::Connection`.
- Las conexiones entre módulos se dibujan como:
  - Curvas con pulso animado cuando están activas.
  - Líneas rectas punteadas (sin pulso) cuando la conexión está silenciada, manteniendo la legibilidad topológica de la escena.
- El click sobre la línea entre instrumentos ya no mutea el objeto de origen, sino que conmuta el estado de mute de la conexión correspondiente (por ahora especialmente útil en la cadena `osc1 → filter1`).
- La lógica de audio en `timerCallback` tiene en cuenta el mute de conexión: si la conexión `osc1→filter1` está silenciada, la cadena completa osc→filtro→Master se considera muda (el oscilador no vuelve a alimentarse directamente al Master mientras exista esa conexión), respetando siempre los estados de mute por objeto.

### Esquema del formato Reactable `.rtp`
- Documentado el archivo `research/reactable_rtp_schema.md` describiendo la estructura XML de Reactable (`<reactablepatch>`, `<background>`, `<tangibles>/<tangible>`, `<author>`, `<patch>`), con detalle por tipo de tangible (`Output`, `Tonalizer`, `Volume`, `Tempo`, `Accelerometer`, `LFO`, `Sequencer`, `Filter`, `Delay`, `Modulator`, `WaveShaper`, `Input`, `Loop`, `Oscillator`, `Sampleplay`).
- Se definen los subelementos compartidos (`<envelope>`, `<loop>`, `<hardlink>`, `<tone>`, `<sequence>`, `<instrument>`) y su significado, así como los atributos relevantes de cada uno.
- El documento establece el mapeo conceptual entre cada `<tangible>` y el modelo actual de Rectai: cada tangible se descompone en un `ObjectInstance` (instancia física sobre la mesa) y un `AudioModule` concreto (módulo lógico de audio/control) con parámetros y estructuras auxiliares.
- Se identifican los tipos de módulos y estructuras que faltan en el proyecto (por ejemplo, `TonalizerModule`, `SequencerModule`, `LoopModule`, `SampleplayModule`, estructuras `Envelope`, `SequenceTrack`, `LoopDefinition`, `ToneDefinition`, `SampleInstrument`, etc.) para que el futuro loader pueda reconstruir escenas completas a partir de archivos `.rtp` originales.
 
### Nuevos módulos y estructuras derivadas del esquema `.rtp`
- Añadidas estructuras de datos en `core/src/core/AudioModules.h` para representar elementos del formato Reactable: `Envelope`, `LoopDefinition`, `ToneDefinition`, `SequenceTrack` (incluyendo capas `tenori0..tenori12`) y `SampleInstrument`.
- Extendidos los módulos existentes `OscillatorModule` y `FilterModule` con soporte interno para un `Envelope`, preparando el terreno para mapear directamente los `<envelope>` del `.rtp`.
- Implementados nuevos módulos concretos alineados con los tipos de `<tangible>` de Reactable: `OutputModule`, `TonalizerModule`, `VolumeModule`, `TempoModule`, `AccelerometerModule`, `LfoModule`, `SequencerModule`, `DelayModule`, `ModulatorModule`, `WaveShaperModule`, `InputModule`, `LoopModule` y `SampleplayModule`, todos heredando de `AudioModule` y configurando metadatos de UI, puertos y parámetros por defecto inspirados en los atributos del `.rtp`.
- Estos módulos todavía no se usan en la UI ni en los tests, pero proporcionan una capa de modelo coherente para implementar a continuación el loader de archivos `.rtp` y poblar `rectai::Scene` con instancias que reflejen fielmente los patches originales de Reactable.

### Loader de patches Reactable `.rtp` y hardlinks
- Añadido `core/src/core/ReactableRtpLoader.{h,cpp}` con una API `LoadReactablePatchFromString` / `LoadReactablePatchFromFile` que parsea directamente archivos `.rtp` XML (sin cambiar su estructura) y construye un `rectai::Scene` con módulos concretos (`OutputModule`, `DelayModule`, `OscillatorModule`, `SequencerModule`, `LoopModule`, etc.) y `ObjectInstance` para cada `<tangible>`.
- El loader rellena también estructuras auxiliares (`Envelope`, `ToneDefinition`, `SequenceTrack`, `LoopDefinition`, `SampleInstrument`) a partir de los subelementos correspondientes (`<envelope>`, `<tone>`, `<sequence>`, `<loop>`, `<instrument>`), y extrae metadatos de `<author>`/`<patch>` en `ReactablePatchMetadata`.
- Implementada la traducción de `<hardlink to="..." />` dentro de tangibles `Delay` a conexiones reales del modelo: para cada hardlink, se crea un `rectai::Connection` que une el puerto `out` del módulo delay con el puerto `in` del módulo destino (por ejemplo, `Output` con id `-1`). Estas conexiones se marcan ahora explícitamente en el modelo con `is_hardlink = true`.
- Actualizado `tests/scene_tests.cpp` con pruebas de humo que validan tanto la carga básica de un Oscillator desde un `.rtp` mínimo como la creación de una conexión `Delay -> Output` derivada de un hardlink.

### Módulo Sampleplay y soporte básico de SoundFont
- Extendido `SampleplayModule` en `core/src/core/AudioModules.{h,cpp}` para asociar un único archivo SoundFont (.sf2) por módulo.
- Añadido método `bool LoadSoundfont(const std::string& path, std::string* error_message)` que realiza una validación ligera del encabezado SF2 (RIFF + "sfbk") usando solo C++ estándar y, en caso de éxito, almacena el path en el módulo (`soundfont_path_`) y marca `has_soundfont()`.
- Añadido un test en `tests/scene_tests.cpp` que genera un archivo SF2 sintético mínimo en disco, invoca `SampleplayModule::LoadSoundfont`, verifica que se marca como cargado y elimina el archivo temporal.

### Interacción de hardlinks en la UI
- Ampliada la semántica de `Connection::is_hardlink` para que también se use en la interacción de la UI, no solo en el loader `.rtp`.
- En `MainComponent` se introducen hardlinks dinámicos creados por colisión: cuando dos objetos (módulos) hacen contacto en la interfaz (sus círculos se tocan dentro del área musical), se detecta el evento de choque y se llama a un helper que alterna la conexión hardlink entre ambos módulos siempre que `AudioModule::CanConnectTo` lo permita.
- Cada nuevo choque entre un par de objetos alterna el estado: si no existe conexión, se crea una `Connection` con `is_hardlink = true`; si ya existe un hardlink entre esos módulos, se elimina; si existe una conexión normal, se “promueve” a hardlink.
- Los hardlinks se pintan ahora en rojo (línea y pulso animado) en `MainComponent::paint`, distinguiéndose de las conexiones dinámicas blancas.
- El cálculo de conexiones “activas” y el ruteo de audio en `timerCallback` consideran los hardlinks como siempre activos dentro del área musical, ignorando las restricciones geométricas de cono, pero se eliminan automáticamente si cualquiera de los dos módulos sale fuera del área de música.

### Visualización de forma de onda sobre líneas de audio
- Se ha extendido `MainComponent::paint` para que cada línea que genera o transporta sonido muestre una forma de onda superpuesta sobre la línea blanca base.
- Se añaden helpers internos para:
  - Detectar si una `rectai::Connection` es de audio (consultando los `PortDescriptor` de `AudioModule` y, en su defecto, las flags `produces_audio`/`consumes_audio`).
  - Dibujar una forma de onda senoidal animada sobre un segmento recto (`drawWaveformOnLine`) o sobre una curva cuadrática de Bézier (`drawWaveformOnQuadratic`).
- Las líneas radiales centro→objeto que corresponden a módulos que producen o consumen audio ahora se dibujan como una única línea ondulada, sin duplicar la línea blanca estática. Esta línea se obtiene a partir de una muestra real del audio mezclado en `AudioEngine`.
- Las conexiones entre módulos (`Scene::connections`) marcadas como de audio sustituyen igualmente su trazo base por una única curva ondulada (recta o Bézier según el tipo de conexión) siempre que la conexión no esté muteada, reforzando visualmente qué rutas llevan señal.
- Se ha añadido un pequeño buffer circular de muestras mono en `AudioEngine` (unos ~90 ms de historial) y un método `getWaveformSnapshot` que devuelve una ventana de ~50 ms ya decimada a un número fijo de puntos para que la UI pueda normalizar y dibujar la forma de onda.
- Las conexiones y líneas muteadas mantienen su aspecto atenuado/punteado pero no muestran forma de onda, reflejando que no están transportando audio, y cuando el nivel de audio es prácticamente nulo la línea ondulada converge visualmente a una recta.

### Ajustes de rendimiento y precisión de la onda
- El `MainComponent` ahora usa un timer a 120 Hz (en lugar de 60 Hz) y calcula el `dt` real entre ticks mediante `juce::Time::getMillisecondCounterHiRes`, lo que hace que las animaciones y la visualización de la forma de onda respondan con menor latencia y sin depender de una frecuencia fija.
- Se ha eliminado la animación antigua del “punto” que viajaba desde cada módulo al centro (línea módulo→Master), para reducir ruido visual y dejar que la forma de onda sea el principal indicador de flujo de audio.
- En `MainComponent::timerCallback` se mantiene ahora un conjunto `modulesWithActiveAudio_` con los ids de módulos que realmente están contribuyendo audio al master (generadores y, cuando procede, sus módulos destino aguas abajo).
- El pintado de líneas radiales y conexiones consulta este conjunto, de forma que solo los módulos y conexiones con audio activo muestran una onda: por ejemplo, un filtro colocado en el área musical sin estar conectado a ningún oscilador ya no dibuja onda, y su línea vuelve a ser plana.

### Eliminación de la animación de bola en hardlinks
- Se ha eliminado también el pequeño círculo rojo animado que recorría las conexiones marcadas como `is_hardlink`, de forma que estas rutas se representan únicamente mediante su línea (recta u ondulada si llevan audio) sin elementos adicionales que distraigan de la lectura del grafo de módulos.

### Debug view del tracker con resaltado de fiduciales
- En `tracker/src/main.cpp` se ha extendido el modo `--debug-view` para que, en modo live, dibuje un rectángulo rojo alrededor de cada fiducial estable detectado.
- Se reutiliza la lista de `TrackedObject` estabilizada (tras el filtro de detecciones consecutivas) para que solo se resalten objetos “confiables”; si un fiducial desaparece de la escena deja de aparecer también su rectángulo.
- Las coordenadas normalizadas (`x_norm`, `y_norm`) de cada objeto se convierten a píxeles sobre el frame binarizado que ya se mostraba en la ventana de debug, compensando el `flip` horizontal aplicado para que la posición del rectángulo coincida con la del marcador en la imagen.

### Rango de IDs válidos para fiduciales amoeba
- En `tracker/src/TrackerEngine.cpp` se han introducido constantes para definir el rango de IDs de fiduciales “reconocidos” por el tracker (`kMinRecognisedMarkerId = 1`, `kMaxRecognisedMarkerId = 107`).
- La función `detectAmoebaFiducials` ahora descarta cualquier fiducial cuyo `id` esté fuera de ese intervalo y, además, ignora explícitamente el ID `1` aunque sea el mínimo, de modo que solo los marcadores previstos en el sistema se tienen en cuenta para el tracking y el envío OSC.
 - El umbral de choque para crear/eliminar hardlinks se ha afinado usando un radio virtual de 30px por nodo y exigiendo al menos 4px de “solapamiento” entre esos círculos virtuales (distancia entre centros ≤ 2*30 - 4), de forma que el hardlink solo se activa cuando los módulos se acercan de manera clara, sin romper la restricción de que los nodos no se sobrepongan visualmente.

### Tamaño mínimo de imagen para detección de fiduciales amoeba
- En `tracker/src/TrackerEngine.cpp` se ha introducido la constante `kMinFiducialImageSize = 32` dentro del namespace interno del tracker.
- `detectAmoebaFiducials` ahora comprueba también que la imagen binarizada tenga al menos `32x32` píxeles antes de llamar a `step_segmenter`/`find_fiducialsX`; si el frame es más pequeño, devuelve una lista vacía y `TrackerEngine::processFrame` cae automáticamente al modo de tracking por blobs.
- Este filtro reduce falsos positivos en escenas con fondo blanco o ruido muy pequeño que no corresponde realmente a un patrón amoeba, especialmente cuando se trabaja con resoluciones reducidas o zonas recortadas.

### Validación local del patrón amoeba por contenido negro
- Además del tamaño mínimo global, `detectAmoebaFiducials` realiza ahora una validación local por patrón sobre la imagen binarizada.
- Para cada `FiducialX` detectado se recorta una región alrededor de su centro de `64x64` píxeles (ajustada a los bordes si es necesario) en el frame binario.
- Se calcula el porcentaje de píxeles negros dentro de ese recorte (recordando que el binarizado produce 0 para negro y 255 para blanco) y, si el ratio de negro es inferior al 10%, el fiducial se descarta como falso positivo.
- Este chequeo adicional evita aceptar “marcadores” que en realidad son parches casi blancos con muy poco contenido de patrón, lo que ayuda a filtrar ruido visual o reflejos en fondos claros.

### Carga de `default.rtp` desde `com.reactable`
- La aplicación JUCE principal (`rectai-core`) ahora intenta cargar automáticamente el patch Reactable por defecto ubicado en `com.reactable/Resources/default.rtp` al inicializar `MainComponent`.
- Se reutiliza el loader existente `LoadReactablePatchFromFile` de `core/src/core/ReactableRtpLoader.{h,cpp}` para poblar directamente un `rectai::Scene` con módulos, conexiones y objetos a partir del `.rtp` original.
- La ruta al recurso se resuelve de forma robusta probando varias ubicaciones relativas tanto al directorio de trabajo actual como al ejecutable (por ejemplo, `com.reactable/Resources/default.rtp`, `../com.reactable/Resources/default.rtp`, etc.), de manera que funcione tanto al lanzar desde la raíz del repo como desde el árbol `build/`.
- Si la carga falla (por ausencia de archivo o error de parseo), `MainComponent` recupera el comportamiento anterior creando la pequeña escena de ejemplo hardcodeada (`osc1` → `filter1`) para mantener un fallback visible.

### Iconos de módulos desde atlas Reactable
- Añadido cargador ligero de atlas en `MainComponent` que localiza `atlas_2048.png` y `atlas_2048.xml` dentro de `com.reactable/Resources`, parsea sus `<sprite>` y construye un mapa en memoria `icon_id → rect` (usando el nombre corto, p.ej. `"oscillator"`, `"filter"`).
- La UI de nodos utiliza ahora este atlas para dibujar los iconos originales de Reactable dentro de cada círculo cuando existe sprite para el `icon_id` del módulo; en caso contrario, conserva los iconos vectoriales procedurales que ya existían.
- Cuando un icono del atlas está disponible (por ejemplo, para `OscillatorModule` y `FilterModule`), el texto del módulo deja de renderizarse dentro del nodo y se sustituye visualmente por dicho icono, manteniendo la estética original de Reactable y evitando duplicar label + icono.

### Flag `docked` en `ObjectInstance` y área musical
- `rectai::ObjectInstance` se ha ampliado con un flag booleano `docked`, expuesto mediante `bool docked() const`, que indica si el tangible procede de un elemento `docked="1"` en el `.rtp`.
- El loader de patches Reactable (`ReactableRtpLoader.cpp`) ahora lee el atributo `docked` de cada `<tangible>` y lo propaga al construir el `ObjectInstance` correspondiente, manteniendo el resto de la semántica (tracking_id como `id`, posición `x`,`y` y ángulo en radianes).
- La función `MainComponent::isInsideMusicArea` utiliza ahora este flag: cualquier objeto marcado como `docked` se considera automáticamente fuera del área musical, incluso si sus coordenadas geométricas cayeran dentro del círculo de la mesa.
- Con este cambio, los tangibles acoplados (`docked=1` en `default.rtp`, como Tonalizer, Volume, Tempo, LFO, Loop, Sampleplay, etc.) se tratan como módulos “apartados” que no participan en las conexiones espaciales ni en las heurísticas del área musical, alineando la semántica con el comportamiento esperado de la Reactable original.

### Semántica de `Output` (master) y visualización del nodo central
- El tangible `type="Output"` del patch Reactable se interpreta ahora explícitamente como el **master** de la escena y ya no se representa como un módulo normal en la mesa (no aparece como nodo ni en la barra de dock).
- `ReactablePatchMetadata` se amplía con `master_colour_argb` y `master_muted`, que el loader rellena leyendo los atributos `color` y `muted` del tangible `Output` (admitiendo colores en formato ARGB de 8 dígitos o RGB de 6 dígitos, asumiendo alfa opaco).
- `MainComponent` usa `master_colour_argb` para colorear el punto central y sus ondas de pulso, en lugar de un blanco fijo, de modo que el aspecto del master respeta el patch original (`default.rtp` u otros `.rtp`).
- El atributo `muted` del `Output` se traduce en un flag `masterMuted_` en la UI: cuando está activo, atenúa visualmente el nodo central y fuerza el nivel de mezcla global a cero en `timerCallback`, actuando como un mute maestro sobre todo el sonido generado por los módulos.

### Helpers de UI con namespace explícito en `MainComponent`
- Se ha alineado el uso de los helpers de UI definidos en `MainComponentHelpers.h` (`makeConnectionKey`, `makeObjectPairKey`, `makeModulePairKey`, `isConnectionGeometricallyActive`) para que se invoquen siempre cualificados como `rectai::ui::...` desde `MainComponent.cpp`.
- Esto corrige errores de compilación recientes por símbolos no resueltos y deja más clara la separación entre la lógica de modelo (`rectai`) y las utilidades puramente gráficas/de UI (`rectai::ui`).

### Helper genérico `loadFile` para recursos de Reactable
- Se ha extraído la lógica de búsqueda de recursos (antes duplicada en `MainComponent` y `MainComponent_Atlas`) a un helper `rectai::ui::loadFile` definido en `MainComponentHelpers.{h,cpp}`.
- `loadFile` recibe una ruta relativa (por ejemplo, `"com.reactable/Resources/default.rtp"`) y prueba varias ubicaciones plausibles relativas al directorio actual y al ejecutable (`.`, `..`, `../..`, y el parent del binario), devolviendo el primer `juce::File` existente o un archivo vacío si no encuentra nada.
- `MainComponent` usa ahora `loadFile` para localizar `default.rtp` antes de invocar `LoadReactablePatchFromFile`, eliminando el array local de 8 candidatos.
- `MainComponent_Atlas` también usa `loadFile` para hallar `com.reactable/Resources/atlas_2048.png` y deriva de ahí el `atlas_2048.xml` en el mismo directorio, unificando la estrategia de búsqueda de recursos Reactable y reduciendo duplicación de código.

### Alineación del hit-test de líneas centro→objeto con hardlinks
- Ajustada la lógica de `mouseDown` en `MainComponent_Input.cpp` para que el hit-test sobre las líneas centro→objeto siga exactamente las mismas reglas de visibilidad que el dibujado en `MainComponent_Paint.cpp`.
- Ahora, los módulos generadores (`ModuleType::kGenerator`) que tengan una conexión saliente activa (incluyendo conexiones marcadas como `is_hardlink`) dejan de dibujar su línea directa al Master **y** dejan de ser clicables a través de esa línea, evitando que un “link invisible” al centro pueda mutear el módulo.
- El filtro de hit-test también ignora objetos `docked` y el tangible `Output` (`logical_id == "-1"`), de forma que solo los instrumentos realmente representados con una línea visible hacia el Master pueden togglear su estado de mute mediante click cerca de dicha línea.

### Giro de módulos y modulación de frecuencia por rotación
- El protocolo OSC de tracking (`TrackingOscReceiver`) interpreta ahora el quinto argumento de `/rectai/object` como ángulo en grados dentro del rango `[0, 360]` y lo convierte internamente a radianes antes de poblar `rectai::ObjectInstance`, manteniendo la semántica del modelo (`angle_radians`) coherente con el loader `.rtp`.
- `MainComponent` mantiene un mapa `lastObjectAngleDegrees_` indexado por `tracking_id` que almacena el último ángulo conocido (en grados) de cada `ObjectInstance` presente en la escena.
- En `MainComponent::timerCallback`, antes de calcular la mezcla de frecuencia/nivel hacia `AudioEngine`, se recorre la lista de objetos y para cada uno que:
  - tenga un `AudioModule` asociado,
  - exponga control de frecuencia (`uses_frequency_control()`), y
  - esté dentro del área musical (`isInsideMusicArea`),
  se calcula el delta de rotación entre el ángulo actual y el previo.
- El ángulo almacenado en la escena se convierte de radianes a grados, se obtiene la diferencia y se normaliza al rango `[-180, 180]` para tratar correctamente el cruce 0/360°; a partir de ahí se deriva una variación de frecuencia normalizada `deltaFreq = diff / 360.0f` en el rango aproximado `[-0.5, 0.5]`.
- Esta `deltaFreq` se suma al parámetro `freq` del módulo correspondiente, usando `Scene::SetModuleParameter` y limitando siempre el resultado a `[0.0, 1.0]` mediante `juce::jlimit`, de forma que un giro completo de 360° equivale a un desplazamiento de ±1.0 en el parámetro de frecuencia.
- Los objetos que desaparecen de la escena (se borran vía `/rectai/remove` o por otros caminos) se eliminan también del mapa `lastObjectAngleDegrees_` en cada frame, evitando fugas de memoria y dejando el tracking preparado para nuevas apariciones con un historial limpio.

### Alineación del ángulo OSC entre tracker y core
- `tracker/src/OscSender.{h,cpp}` espera ahora explícitamente el ángulo en **grados** en `sendObject` y lo empaqueta tal cual en el mensaje `/rectai/object` (tipo `float`).
- En `tracker/src/main.cpp`, el modo live convierte `obj.angle_rad` (radianes provenientes de `TrackerEngine`) a grados con `angleDegrees = obj.angle_rad * 180.0f / M_PI` antes de enviarlo por OSC, y escribe un log detallado por fiducial con `angle_deg=...` para poder depurar visualmente la rotación.
- El modo synthetic también usa ya una variable `angleDegrees` coherente, de modo que cualquier futura animación de giro sintético respetará la misma convención de grados/plano `[0,360]` utilizada por el core.
- En el core, `TrackingOscReceiver` sigue recibiendo el quinto argumento como grados y lo transforma a radianes al construir `ObjectInstance`, de forma que `MainComponent` y el resto del modelo continúan trabajando internamente siempre en radianes.
- La lógica de modulación por rotación en `MainComponent::timerCallback` se ha relajado para aplicar el delta de giro a cualquier objeto **no docked**, independientemente de que esté o no estrictamente dentro del círculo musical, evitando casos en los que la barra de `freq` no se actualizaba al girar un fiducial situado muy cerca del borde.

### Orientación radial de módulos respecto a su línea de audio
- Los módulos que se encuentran dentro del área musical se renderizan ahora con una orientación radial alineada con su propia línea de audio (línea blanca) hacia el centro: antes todos los nodos tenían una orientación fija en pantalla, independientemente de su posición.
- En `MainComponent::paint`, al dibujar cada nodo dentro del círculo musical se calcula el ángulo polar entre el centro de la mesa y la posición del objeto y se aplica una `AffineTransform::rotation` alrededor del centro del nodo; con esto, el cuerpo del módulo, sus barras laterales de `freq`/`gain` y el icono interno giran como un bloque al mover el tangible por la superficie.
- El widget de secuenciador asociado a objetos cuyo `logical_id` empieza por `"seq"` adopta la misma orientación radial cuando están en el área musical: en lugar de usar únicamente `angle_radians()` del `ObjectInstance`, el ángulo efectivo se recalcula respecto al centro de la mesa, de modo que la cuadrícula proyecta siempre “hacia fuera” siguiendo la misma dirección que la línea de audio.

### Hardlinks rectos y controles laterales con hit-test rotado
- Las conexiones marcadas como `is_hardlink` en el modelo se dibujan ahora como líneas rectas rojas entre módulos, con un pulso animado que se desplaza linealmente de origen a destino; las conexiones dinámicas siguen renderizándose como curvas de Bézier blancas con su propio pulso siguiendo la trayectoria curva.
- La lógica de hit-test de los controles laterales de `freq` y `gain` en `MainComponent::mouseDown` y el arrastre correspondiente en `mouseDrag` transforman ahora el punto de ratón al sistema de coordenadas local del módulo (invirtiendo la misma rotación que se aplica en `paint`), de forma que el área interactiva de los sliders gira junto con las barras y permanece coherente con su aspecto visual aunque el nodo esté rotado en la mesa.

### Unicidad de objetos por `logical_id` en la Scene
- `rectai::Scene::UpsertObject` garantiza ahora que solo exista un `ObjectInstance` por cada `logical_id` en la escena: antes de insertar/actualizar el nuevo objeto, recorre el mapa interno `objects_` y elimina cualquier entrada previa cuyo `logical_id()` coincida pero cuyo `tracking_id()` sea distinto.
- Esto resuelve situaciones en las que un mismo módulo podía aparecer dos veces en la superficie musical (por ejemplo, una posición “manual” definida en el `.rtp` y otra proveniente de tracking OSC) manteniendo **solo** la posición más reciente, que típicamente es la enviada por el tracker.
- Con esta política, cuando el tracker envía `/rectai/object` para un módulo que ya tenía un objeto asociado, la ubicación controlada por OSC pasa a ser la única referencia espacial para ese `logical_id`, de modo que las heurísticas de UI y audio (conexiones, área musical, barras de `freq`/`gain`) siempre operan sobre una única instancia coherente por módulo.

### Mapeo de múltiples osciladores a voces independientes
- `AudioEngine` expone ahora públicamente la constante `kMaxVoices` (actualmente 16), que define el número máximo de voces senoidales que puede mezclar en paralelo.
- `MainComponent::timerCallback` deja de colapsar todos los generadores (`ModuleType::kGenerator`, típicamente osciladores) en una sola frecuencia/nivel global; en su lugar recorre los objetos de la `Scene` y, para cada generador activo dentro del área musical y con ruta de conexión válida, calcula una frecuencia absoluta y un nivel efectivo teniendo en cuenta:
  - El parámetro `freq` propio del módulo.
  - El parámetro `gain` tanto del generador como del módulo aguas abajo (por ejemplo, un filtro o volumen) siguiendo las conexiones de `Scene`.
  - Los flags de mute por objeto (`mutedObjects_`), por conexión (`mutedConnections_`) y el estado `masterMuted_`.
- Cada generador audible se asigna a una voz distinta de `AudioEngine` mediante `setVoice(voiceIndex, frequency, level)`, hasta un máximo de `AudioEngine::kMaxVoices`; las voces no utilizadas en ese frame se fuerzan explícitamente a `(frequency=0.0, level=0.0)` para garantizar que osciladores que desaparecen de la escena dejan de sonar inmediatamente.
- El callback de audio en `AudioEngine` sigue mezclando todas las voces en una salida estéreo simple, pero ahora cada oscilador tangible se traduce en una contribución independiente al mix, permitiendo escuchar simultáneamente varios osciladores conectados al Master o encadenados a filtros según la topología definida en la `Scene`.

### Retorno automático al dock cuando desaparece el fiducial
- El manejador `/rectai/remove` en `TrackingOscReceiver` deja de llamar directamente a `Scene::RemoveObject` y, en su lugar, busca el `ObjectInstance` asociado al `trackingId` que llega por OSC.
- Si existe, se crea una nueva instancia con los mismos `tracking_id`, `logical_id`, posición y ángulo, pero con el flag `docked=true`, y se reinyecta en la escena mediante `Scene::UpsertObject`.
- Dado que `isInsideMusicArea` considera automáticamente que cualquier objeto `docked` está fuera del área musical, el módulo desaparece de la mesa central y vuelve a aparecer en la barra de dock de la UI, manteniendo intacto su módulo lógico en la escena.
- Combinado con la unicidad por `logical_id`, esto asegura que cada módulo tiene siempre **una sola** representación espacial: mientras el fiducial está presente, la controla el tracking OSC; cuando desaparece, la representación vuelve al dock sin duplicados.

### Tracker basado en blobs tipo "amoeba" y eliminación de ArUco
- `TrackerEngine` deja de depender de `cv::aruco` y ya no intenta detectar marcadores ArUco ni usar diccionarios predefinidos.
- La ruta de detección se basaba inicialmente únicamente en segmentación de blobs: conversión a escala de grises, `GaussianBlur`, `adaptiveThreshold` y `findContours` para localizar regiones orgánicas de un tamaño mínimo configurable.
- Para cada contorno suficientemente grande se calculaba el centroide mediante momentos y la orientación aproximada con `cv::fitEllipse`, generando un `TrackedObject` con coordenadas normalizadas y ángulo en radianes.
- Se mantenía un pequeño estado interno (`lastObjects_` + `nextId_`) que asociaba cada nueva detección con el objeto previo más cercano en coordenadas normalizadas, asignando así IDs numéricos estables entre frames sin depender de IDs codificados en el marcador.
- `tracker/CMakeLists.txt` y `tests/CMakeLists.txt` se han simplificado para pedir solo los módulos de OpenCV necesarios (`core`, `imgproc`, `videoio`, `highgui`) eliminando `aruco` e `imgcodecs` del flujo de compilación.

### Detección real de fiduciales "amoeba" usando libfidtrack
- El ejecutable `rectai-tracker` ahora enlaza directamente con el código original de reacTIVision (`reacTIVision/ext/libfidtrack`), añadiendo a su target los archivos `segment.c`, `fidtrackX.c`, `topologysearch.c` y `treeidmap.cpp`, e incluyendo su cabecera pública.
- `TrackerEngine` inicializa en `initialise` las estructuras de libfidtrack (`TreeIdMap`, `Segmenter`, `FidtrackerX`) usando el set de símbolos `"default"` de amoeba incluido en `default_trees.h`, alineado con el comportamiento por defecto de reacTIVision.
- En `processFrame`, tras convertir el frame a escala de grises y aplicar un `GaussianBlur` + `adaptiveThreshold` con `THRESH_BINARY` (0 negro, 255 blanco), se pasa la imagen binaria a `step_segmenter` y se invoca `find_fiducialsX` para obtener un array de `FiducialX` con `id`, posición y ángulo.
- Cada `FiducialX` con `id >= 0` se normaliza a coordenadas [0,1] usando el ancho/alto configurados en `initialise`, y se traduce a un `TrackedObject` cuyo `id` coincide con el ID amoeba real del marcador físico; el ángulo se preserva en radianes desde libfidtrack.
- Si en un frame no se detecta ningún fiducial válido (por ausencia de marcadores amoeba o por condiciones de iluminación), el engine mantiene un **fallback** basado en blobs: se ejecuta la ruta anterior de `findContours` + `fitEllipse` y se siguen asignando IDs estables mediante proximidad (`lastObjects_` + `nextId_`), de manera que los tests sintéticos y escenarios sin marcadores físicos siguen funcionando.
- El destructor de `TrackerEngine` libera correctamente los recursos de libfidtrack (`terminate_segmenter`, `terminate_fidtrackerX`, `terminate_treeidmap`), evitando fugas de memoria en ejecuciones prolongadas del servicio de tracking.

### Target Earthly para AppImage
- Añadido un nuevo target `appimage` en el `Earthfile` que reutiliza la imagen base `the-base`, compila los binarios en modo Release y prepara un `AppDir` con `RectaiTable` y `rectai-tracker` en `usr/bin`.
- El target descarga `linuxdeploy` y `appimagetool` como AppImages y los usa para recolectar todas las dependencias compartidas (incluyendo `curl`, OpenCV y librerías del sistema necesarias para JUCE) y empaquetarlas en un único `rectai-table-x86_64.AppImage`.
- Se genera un archivo `.desktop` mínimo para `RectaiTable` mediante un comando `printf` (en lugar de heredocs), evitando problemas de sintaxis en Earthly/Docker debidos a la indentación del marcador `EOF`.
- El AppImage resultante se exporta como artefacto local en `build/rectai-table-x86_64.AppImage` al ejecutar `earthly -P +appimage` desde la raíz del repositorio.

### Tests del tracker con PNGs de fiduciales amoeba
- `tests/tracker_tests.cpp` utiliza ahora explícitamente los recursos `fiducial_30.png` y `fiducial_55.png` ubicados en `tests/`, cargándolos directamente desde disco mediante `cv::imread` (en modo escala de grises) sin necesidad de copiarlos al directorio de binarios.
- El test inicializa `TrackerEngine` con las dimensiones de la primera imagen y procesa cada PNG, verificando que entre los `TrackedObject` devueltos se encuentra al menos uno cuyo `id` coincide exactamente con el número codificado en el nombre del archivo (30 y 55 respectivamente). Esto endurece la aserción anterior (que solo comprobaba que hubiera algún ID positivo) y valida de forma más directa la decodificación de fiduciales amoeba vía libfidtrack.

### Vista de debug del tracker alineada con `processFrame`
- `TrackerEngine` expone ahora una sobrecarga `TrackedObjectList processFrame(const cv::Mat& frame, cv::Mat& debugFrame)` que, además de devolver la lista de `TrackedObject`, rellena `debugFrame` con la imagen binarizada (`adaptiveThreshold`) que se utiliza internamente para la detección de fiduciales amoeba y el fallback de blobs.
- La implementación pública sin imagen de debug (`processFrame(const cv::Mat& frame)`) delega en un helper privado `processFrameInternal(const cv::Mat& frame, cv::Mat* debugFrame)`, garantizando que ambos caminos comparten exactamente el mismo pipeline de procesamiento.
- En `tracker/src/main.cpp`, cuando se ejecuta en modo `--mode=live` y con `--debug-view`, el bucle principal llama a la nueva variante de `processFrame` con `debugFrame`, y la ventana OpenCV de debug muestra ese `debugFrame` (espejado en horizontal), que es una imagen de un solo canal (grises/0-255) exactamente igual a la que consume libfidtrack.
- En modo sintético (o si no hay `debugFrame` disponible por cualquier motivo), el debug view ya no enseña el frame de cámara en color, sino que convierte el frame a escala de grises antes de mostrarlo, de forma que la vista de debug se parezca siempre al input que espera reacTIVision.

### Vista de depuración de cámara en `rectai-tracker`
- Añadido flag `--debug-view` al ejecutable `rectai-tracker`.
- Cuando se pasa este flag (en cualquier modo), el proceso abre una ventana de OpenCV titulada `rectai-tracker debug` donde muestra el frame de la webcam espejado horizontalmente (`cv::flip`), permitiendo verificar que la cámara está capturando correctamente y que el área de trabajo está bien encuadrada.
- Mientras la vista de depuración está activa, pulsar `Esc`, `q` o `Q` cierra la ventana y termina el tracker con un mensaje de log `[rectai-tracker] Debug view exit requested by user`.

### Estabilización de fiduciales por frames consecutivos en `rectai-tracker`
- En el modo `--mode=live`, el bucle principal de `rectai-tracker` mantiene ahora un contador por ID de fiducial con el número de frames consecutivos en los que se ha detectado cada marcador.
- Solo se consideran "objetos estables" aquellos fiduciales que se han visto al menos en un umbral configurable de **N frames consecutivos** (actualmente 10); únicamente estos objetos se usan para actualizar `TrackerState` y para enviar mensajes OSC `/rectai/object` al core JUCE.
- Además del número de frames, se comprueba que la posición normalizada del fiducial no cambie bruscamente entre frames: si la distancia entre posiciones sucesivas supera ~10% del tamaño de la mesa, se resetea el contador a 1 para ese ID.
- Si un fiducial deja de aparecer en un frame, su contador y su última posición se eliminan de las tablas internas, de modo que detecciones espurias que solo duran uno o dos frames (o que saltan de sitio) nunca llegan a producir logs ni eventos OSC.
- Las eliminaciones (`/rectai/remove`) siguen basándose en `TrackerState::collectRemovals`, pero este estado solo se alimenta con los objetos estables, por lo que también se reducen los mensajes de "fiducial X removed" causados por falsos positivos de la cámara.

### Control de Tempo por rotación del módulo Tempo
- El core JUCE (`MainComponent`) trata ahora explícitamente el módulo `TempoModule` como controlador del **BPM global de la sesión**, inicializando `bpm_` a partir del parámetro `tempo` del patch Reactable cargado (por ejemplo, el `tempo="128"` de `default.rtp`), limitado al rango `[40, 400]` BPM.
- En `MainComponent::timerCallback` se calcula para cada `ObjectInstance` un delta de rotación por frame en grados, normalizado a `[-180, 180]` y almacenado en un mapa `rotationDeltaDegrees` compartido tanto por la modulación de frecuencia de osciladores como por el control de tempo.
- Para objetos asociados a `TempoModule`, cada variación de **5º** de rotación incrementa o decrementa el tempo en **1 BPM**, aplicando la misma convención de sentido (clockwise/counter-clockwise) que ya se utiliza para el control de frecuencia; el valor resultante se almacena en `bpm_` como `double` y se limita siempre a `[40, 400]` BPM.
- Cada vez que se actualiza el BPM global se sincroniza también el parámetro lógico `tempo` del propio `TempoModule` vía `Scene::SetModuleParameter`, de forma que futuras serializaciones o inspecciones del modelo vean el tempo actualizado.
- La UI de nodos (`MainComponent_Paint.cpp`) dibuja ahora un pequeño texto con el valor de BPM **entero** (sin decimales) en la esquina superior izquierda del círculo del módulo Tempo, tanto cuando está en la superficie musical como cuando aparece acoplado en la barra de dock, reflejando siempre el valor de `bpm_` que alimenta las ondas concéntricas y la fase del secuenciador.
