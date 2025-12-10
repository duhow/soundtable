# Progreso de implementación

## 2025-12-09

### Modelo y UI básica
- Modelo de dominio inicial implementado en `rectai::Scene` (`Scene.h/.cpp`), con:
  - `ObjectInstance` (id de tracking, id lógico, posición normalizada, ángulo).
  - `AudioModule` + `ModuleType` y descriptores de puertos.
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
 - El umbral de choque para crear/eliminar hardlinks se ha afinado usando un radio virtual de 30px por nodo y exigiendo al menos 4px de “solapamiento” entre esos círculos virtuales (distancia entre centros ≤ 2*30 - 4), de forma que el hardlink solo se activa cuando los módulos se acercan de manera clara, sin romper la restricción de que los nodos no se sobrepongan visualmente.

### Carga de `default.rtp` desde `com.reactable`
- La aplicación JUCE principal (`rectai-core`) ahora intenta cargar automáticamente el patch Reactable por defecto ubicado en `com.reactable/Resources/default.rtp` al inicializar `MainComponent`.
- Se reutiliza el loader existente `LoadReactablePatchFromFile` de `core/src/core/ReactableRtpLoader.{h,cpp}` para poblar directamente un `rectai::Scene` con módulos, conexiones y objetos a partir del `.rtp` original.
- La ruta al recurso se resuelve de forma robusta probando varias ubicaciones relativas tanto al directorio de trabajo actual como al ejecutable (por ejemplo, `com.reactable/Resources/default.rtp`, `../com.reactable/Resources/default.rtp`, etc.), de manera que funcione tanto al lanzar desde la raíz del repo como desde el árbol `build/`.
- Si la carga falla (por ausencia de archivo o error de parseo), `MainComponent` recupera el comportamiento anterior creando la pequeña escena de ejemplo hardcodeada (`osc1` → `filter1`) para mantener un fallback visible.

### Flag `docked` en `ObjectInstance` y área musical
- `rectai::ObjectInstance` se ha ampliado con un flag booleano `docked`, expuesto mediante `bool docked() const`, que indica si el tangible procede de un elemento `docked="1"` en el `.rtp`.
- El loader de patches Reactable (`ReactableRtpLoader.cpp`) ahora lee el atributo `docked` de cada `<tangible>` y lo propaga al construir el `ObjectInstance` correspondiente, manteniendo el resto de la semántica (tracking_id como `id`, posición `x`,`y` y ángulo en radianes).
- La función `MainComponent::isInsideMusicArea` utiliza ahora este flag: cualquier objeto marcado como `docked` se considera automáticamente fuera del área musical, incluso si sus coordenadas geométricas cayeran dentro del círculo de la mesa.
- Con este cambio, los tangibles acoplados (`docked=1` en `default.rtp`, como Tonalizer, Volume, Tempo, LFO, Loop, Sampleplay, etc.) se tratan como módulos “apartados” que no participan en las conexiones espaciales ni en las heurísticas del área musical, alineando la semántica con el comportamiento esperado de la Reactable original.
