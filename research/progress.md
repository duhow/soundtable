# Progreso de implementación

## 2025-12-12

### Mejora de visibilidad de la base circular de los módulos
- En `MainComponent_Paint.cpp` se ha añadido un contorno explícito de alto contraste alrededor del cuerpo circular de cada módulo, tanto en la mesa como en el dock, de forma que la “redonda” bajo cada módulo sea claramente visible incluso cuando el color de relleno está muy próximo al fondo.
- El color del contorno se elige dinámicamente en función del brillo del módulo (blanco sobre módulos oscuros, negro sobre módulos claros), manteniendo la legibilidad de la silueta sin interferir con la paleta de colores cargada desde los patches `.rtp`.
 - Además, el color de relleno del nodo se fuerza a ser completamente opaco (`alpha = 1.0`) en la función `getBodyColourForObject`, de modo que aunque algún color ARGB procedente del `.rtp` lleve un canal alpha bajo, la base circular siempre se renderiza como un disco sólido y coloreado.

### Color por defecto unificado y simplificación de nodos
- En `ReactableRtpLoader.cpp`, cuando un `<tangible>` no declara explícitamente un atributo `color`, el loader ya no conserva el color por defecto definido en el constructor del módulo, sino que sobrescribe el color visual con un valor unificado oscuro `#111111` (`0xFF111111`). Esto garantiza que todos los módulos sin color definido en el `.rtp` comparten una base visual coherente y claramente diferenciable de los módulos coloreados (loops, sampleplays, oscillators, etc.).
- En `MainComponent_Paint.cpp` se han eliminado los dos círculos adicionales de “aura” bajo cada módulo, tanto en la mesa como en el dock; ahora solo se dibuja el círculo principal del módulo (más su contorno), simplificando la lectura visual y acercando el diseño a la representación más plana que estamos buscando para depurar la paleta de colores.

### Corrección de conversión ARGB → `juce::Colour`
- Corregida la función `colourFromArgb` en `MainComponentHelpers.cpp`: antes construía `juce::Colour` pasando los cuatro componentes como si el constructor fuera `(alpha, red, green, blue)`, cuando en realidad el constructor de 4 parámetros espera `(red, green, blue, alpha)`. Esto provocaba un corrimiento de canales y que muchos colores se vieran rojizos/blanquecinos.
- La función ahora extrae los componentes ARGB y usa explícitamente `juce::Colour::fromARGB(alpha, red, green, blue)`, de forma que los colores procedentes de `colour_argb()` (incluidos los `color="..."` del `.rtp`) se respetan exactamente en la UI.
### Opacidad diferenciada para pulsos de tempo
- Ajustado el render de los anillos de pulso en `MainComponent_Paint.cpp` para que los pulsos secundarios de tempo (beats intermedios) se dibujen con menor opacidad que el pulso que marca el beat principal del compás.
- La estructura `Pulse` sigue marcando los beats fuertes con `strong = true` cada 4 golpes (`beatIndex_`), pero ahora el cálculo de alpha distingue entre ambos: el pulso principal mantiene la opacidad base dependiente de la edad del pulso y del estado de mute del master, mientras que los pulsos secundarios escalan esa opacidad por un factor adicional (≈40%).
- Este cambio hace que el ritmo siga siendo claramente visible, pero refuerza visualmente el beat fuerte del compás como referencia rítmica principal, alineando la jerarquía de brillo con el diseño descrito para la UI rítmica de la mesa.

### Fondo de mesa sólido y borde difuminado
- Actualizado el fondo de la mesa en `MainComponent_Paint.cpp` para que el disco principal se pinte con un color sólido `#001a80` (sin gradiente interno), alineado con la paleta azul oscura descrita para la superficie de la mesa.
- Introducido un anillo exterior alrededor de la mesa que se renderiza como una elipse con gradiente radial desde `#001a80` en el borde de la mesa hasta negro en el radio exterior, creando un difuminado suave entre la superficie y el fondo negro general.
- Eliminada la viñeta anterior basada en un simple trazo negro grueso alrededor del círculo y sustituida por este borde degradado, que integra mejor la mesa con el lienzo negro sin añadir contornos duros.

### Separación de gestos: sliders de módulo vs corte de líneas
- Introducido un flag explícito `isCutModeActive_` en `MainComponent` que distingue entre el modo de **interacción** (cursor blanco) y el modo de **corte de sonido** (cursor rojo con trail).
- `MainComponent::mouseDown` ahora solo activa `isCutModeActive_ = true` cuando el click comienza en espacio vacío del área musical (dentro del círculo de música, fuera del dock y sin haber capturado ni un módulo, ni un slider lateral, ni una línea para mute hold); en cualquier otra interacción el gesto permanece en modo blanco.
- `MainComponent::mouseDrag` condiciona la detección de cortes de líneas (`touchCutObjects_` y `touchCutConnections_`) a que `isCutModeActive_` sea verdadero y, además, a que no se esté arrastrando un módulo (`draggedObjectId_ == 0`), ajustando sliders (`sideControlKind_ == kNone`) ni manteniendo una línea en mute temporal (`!activeConnectionHold_`).
- `MainComponent::paint` usa ahora este flag para dibujar el cursor en blanco en todos los gestos de interacción (ajuste de frecuencia/ganancia, click-and-hold sobre líneas) y en rojo solo en gestos de corte iniciados en espacio vacío. Al arrastrar módulos sobre la mesa el cursor se oculta; en el caso de módulos sacados desde el dock, el cursor solo es visible mientras el puntero permanece dentro del propio dock.
- El hit-test de las barras laterales de frecuencia/ganancia se ha ampliado: ahora un click en **cualquier punto de la barra** mueve inmediatamente el valor del parámetro hasta esa posición y comienza un gesto de drag desde ahí, en lugar de requerir que el usuario acierte específicamente sobre el handle.

### Drop seguro con CONTROL para Loop/Oscillator/Sampleplay
- En `MainComponent_Input.cpp` se ha añadido la función auxiliar privada `applyControlDropMuteIfNeeded`, ahora invocada desde `mouseDrag` justo después de actualizar la posición del objeto arrastrado.
- Cuando, durante un drag con la tecla CONTROL pulsada, el objeto está sobre la mesa (posición actual en el área musical) y el módulo asociado es de tipo `OscillatorModule`, `LoopModule` o `SampleplayModule`, la función fuerza el parámetro de nivel correspondiente a 0: `gain = 0.0` para osciladores y `amp = 0.0` para loops y sampleplays.
- De este modo, el silencio se aplica en cuanto el módulo entra o se mueve dentro del área musical mientras se mantiene CONTROL, sin esperar al `mouseUp`, proporcionando una forma rápida de “colocar en silencio” estos módulos al situarlos en la mesa y permitiendo luego subir el nivel desde los controles laterales de ganancia.

### Mapeo de ganancia con verdadero 0% de volumen
- En `MainComponent_Audio.cpp` se ha ajustado el cálculo de `calculatedLevel` para que un parámetro de ganancia normalizado igual a `0.0` produzca un nivel efectivo de `0.0` (silencio real), en lugar de mantener siempre un mínimo basado en `base_level_`.
- El mapeo pasa de `calculatedLevel = base_level + level_range * gainParam` a una versión que devuelve 0 cuando `gainParam <= 0.0` y solo aplica el offset `base_level` para valores de ganancia mayores que 0, de forma que poner el slider de ganancia al mínimo realmente apaga el oscilador en audio (aunque se siga generando la forma de onda interna para visualización).

## 2025-12-11

### Corrección de estiramiento de formas de onda según distancia
- Ajustado el render de formas de onda en `MainComponent_Paint.cpp` para que la muestra de audio utilizada en las líneas dependa de la **distancia en píxeles** y no de la longitud normalizada de la línea.
- `drawWaveformOnLine` ahora mapea un desplazamiento a lo largo de la línea (en píxeles) a la ventana de samples usando un factor fijo de "muestras por píxel" (con wrapping en el buffer), manteniendo así un patrón visual consistente (por ejemplo, el diente de sierra) aunque cambie la distancia entre un módulo y el Master o entre dos módulos conectados.
- `drawWaveformOnQuadratic` sigue el mismo principio utilizando una longitud aproximada de la curva (distancia entre extremos) para definir el avance por el buffer, de modo que las conexiones curvadas también conservan el mismo patrón de onda al mover los módulos.
- Este cambio elimina el efecto observado de formas de onda "comprimidas" cerca del centro y "estiradas" cuando los nodos se alejan, ya que la frecuencia espacial de la onda en pantalla pasa a ser estable e independiente de la longitud de la conexión.
 - Se ha incrementado la resolución del snapshot usado para visualización de cada voz de `128` a `512` samples (`kWaveformPoints`), mejorando la definición del patrón (especialmente en saw) a la vez que se sigue usando solo un ciclo estimado para repetir la forma a lo largo de líneas largas.

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
- Se ha reajustado el cono geométrico de conexión entre instrumentos para que `isConnectionGeometricallyActive` utilice ahora un ángulo total de 120º alrededor del origen (60º de semicono). Esto mantiene la idea de sectores angulares alrededor del centro y ofrece una tolerancia algo más amplia, de modo que cuando dos módulos están razonablemente alineados dentro de ese cono se considera que la conexión dinámica está activa.

### Forma de las conexiones dinámicas en la UI
- Las conexiones entre módulos (`Scene::connections`) que no son hardlink (conexiones dinámicas) se renderizan ahora como segmentos rectos en lugar de curvas de Bézier. El pulso animado que recorre la conexión se mueve linealmente desde el módulo de origen al de destino, y la visualización de waveform también se dibuja sobre la línea recta (`drawWaveformOnLine`), lo que hace que la geometría de las conexiones dinámicas coincida mejor con la expectativa visual de "línea directa" entre módulos cercanos.

### Creación automática de conexiones dinámicas por disposición espacial
- En `MainComponent::timerCallback` se ha añadido una pasada de mantenimiento que recorre los pares de objetos dentro del área musical y, para cada par de módulos compatibles según `AudioModule::CanConnectTo`, crea automáticamente una conexión dinámica (`Connection` con `is_hardlink = false`) desde el módulo origen al destino cuando el objeto destino cae dentro del cono geométrico de 105º definido por `isConnectionGeometricallyActive`.
- Estas conexiones automáticas utilizan siempre los puertos estándar `out → in` y solo se crean cuando no existe ya ninguna conexión entre ese par de módulos, de modo que no interfieren con hardlinks existentes ni con conexiones explícitas que se puedan definir en el futuro.

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

### Controladores globales sin conexiones explícitas (Volume, Tempo, Tonalizer)
- Se ha introducido en `AudioModule` el método virtual `is_global_controller()` para distinguir módulos que actúan como controladores de sesión (p.ej. `VolumeModule`, `TempoModule`, `TonalizerModule`) del resto del grafo de audio.
- `VolumeModule`, `TempoModule` y `TonalizerModule` sobreescriben `is_global_controller()` devolviendo `true`, marcándolos como Global Controllers.
- `Scene::AddConnection` y `AudioModule::CanConnectTo` rechazan ahora cualquier conexión (dinámica o hardlink) en la que participe un Global Controller, de forma que estos módulos nunca aparecen en `Scene::connections` ni se encadenan con otros módulos ni con `Output`.
- En la UI (`MainComponent_Paint.cpp`), los módulos marcados como Global Controller ya no dibujan la línea radial blanca hacia el nodo central, manteniendo el rol de estos tangibles como controladores globales de sesión sin routing explícito.
- Se han añadido tests en `tests/scene_tests.cpp` que verifican que `VolumeModule`, `TempoModule` y `TonalizerModule` no pueden crear conexiones en `Scene` ni ser destino de ellas.

### Alineación de hit-test y pintado en el Dock de módulos
- Se ha corregido la geometría del Dock en `MainComponent::mouseDown` y `MainComponent::mouseDrag` para que utilicen la misma área de contenido que el pintado en `MainComponent_Paint.cpp`, descontando explícitamente la altura del título del Dock antes de calcular alturas disponibles, offsets y centros (`cy`) de cada módulo dockeado.
- Antes del cambio, el cálculo de `cy` para el hit-test de los módulos del Dock ignoraba la franja de título superior, lo que desplazaba hacia arriba el centro lógico de los círculos respecto a su posición real en pantalla; como consecuencia, solo era posible seleccionar/arrastrar un módulo haciendo click en la parte alta del círculo, por encima del icono.
- Con la corrección, el círculo dibujado y el área de hit-test vuelven a coincidir en el Dock: ahora se puede clicar y arrastrar un módulo desde cualquier punto del círculo o de su icono para sacarlo hacia el área musical, manteniendo el mismo comportamiento de scroll vertical del Dock.

### Oscillator con subtipos y cambio por click derecho
- `OscillatorModule` incorpora ahora un enum `Waveform` con cuatro subtipos alineados con la Reactable original: `sine`, `saw`, `square` y `noise`.
- El constructor de `OscillatorModule` inicializa la forma de onda a `sine` y asigna automáticamente el `icon_id` correspondiente del atlas (`oscillator_sine`).
- Se exponen helpers en el modelo:
  - `set_waveform(Waveform)` y `waveform()` para gestionar el estado interno.
  - `set_waveform_from_subtype(const std::string&)` para mapear directamente el atributo `subtype` de los tangibles `type="Oscillator"` en los patches `.rtp`.
  - `cycle_waveform()` para avanzar cíclicamente por los cuatro subtipos y actualizar el `icon_id` a `oscillator_sine`, `oscillator_saw`, `oscillator_square` u `oscillator_noise`.
  - `subtype_string()` como representación textual del subtipo actual (útil de cara a futura serialización).
- El loader de patches `ReactableRtpLoader` ahora lee el atributo `subtype` de los tangibles `Oscillator` y llama a `set_waveform_from_subtype`, de modo que un `<tangible type="Oscillator" ... subtype="sine" ...>` se traduce a un módulo con icono `oscillator_sine` y estado de forma de onda coherente.
- En la UI (`MainComponent_Input`), un click derecho sobre el círculo de un Oscillator (en la superficie musical) invoca `cycle_waveform()` sobre el módulo asociado, cambiando inmediatamente el icono según el subtipo seleccionado.
- El pintado de iconos (`MainComponent_Paint`) se ha ajustado para que los `icon_id` que comienzan por `"oscillator"` (`oscillator_sine`, `oscillator_saw`, etc.) sigan usando el mismo fallback vectorial que el icono genérico `oscillator` cuando el atlas de sprites no está disponible, conservando la legibilidad en modo degradado.
- Se han añadido tests en `tests/scene_tests.cpp` que validan:
  - El ciclo completo de formas de onda e iconos (`sine → saw → square → noise → sine`).
  - Que un patch `.rtp` mínimo con un Oscillator de subtipo `sine` se carga como `OscillatorModule` y expone el icono `oscillator_sine` en el modelo.

### Filtro con cambio de modo por click derecho e iconos específicos
- `FilterModule::Mode` se aprovecha ahora no solo para la lógica interna del filtro sino también para la selección de icono: `set_mode(Mode)` actualiza siempre tanto el estado `mode_` como el `icon_id` asociado del módulo.
- Los tres modos (`kLowPass`, `kBandPass`, `kHighPass`) se mapean directamente a los sprites del atlas `atlas_2048.xml`: `filter_lowpass`, `filter_bandpass` y `filter_hipass` respectivamente.
- El constructor de `FilterModule` inicializa el módulo llamando a `set_mode(Mode::kBandPass)`, de modo que el icono por defecto queda alineado con el modo efectivo (band-pass) sin duplicar lógica de selección de iconos.
- Se añade el helper `cycle_mode()` en `FilterModule`, que recorre cíclicamente los tres modos (`lowpass → bandpass → highpass → lowpass`) reutilizando `set_mode` para mantener `mode_` e `icon_id` siempre sincronizados.
- `set_mode_from_subtype(const std::string&)` continúa leyendo el atributo `subtype` de los tangibles de tipo filtro en los patches `.rtp`, pero ahora delega toda la actualización de estado e icono en `set_mode`, garantizando que cualquier cambio de modo (ya sea por loader o por interacción en la UI) use el mismo código.
- En la UI (`MainComponent_Input`), el bloque de click derecho que antes solo afectaba a los Oscillator se amplía: un click derecho sobre un tangible asociado a `FilterModule` invoca `cycle_mode()`, cambiando el modo del filtro y actualizando inmediatamente el icono renderizado en la mesa.

### Módulo de filtro con modos y resonancia
- `FilterModule` se ha ampliado para soportar tres modos explícitos (`lowpass`, `bandpass`, `highpass`) mediante un enum interno y un helper `set_mode_from_subtype` que inicializa el modo a partir del atributo `subtype` del `.rtp` (por defecto `bandpass` cuando no se reconoce el valor).
- La rotación física del tangible de filtro sigue mapeándose al parámetro normalizado `freq` del módulo, que en adelante se interpreta como frecuencia de corte (low/high-pass) o frecuencia central (band-pass), delegando en el futuro motor de audio la traducción a Hz según el modo.
- La barra lateral derecha del filtro deja de controlar el volumen de la cadena y pasa a representar el parámetro de calidad/resonancia `q`; la UI utiliza ahora `q` para pintar y editar ese control, mientras que el motor de audio ignora dicho valor como ganancia global.
- En el cálculo de nivel por voz en `MainComponent::timerCallback`, los módulos de filtro ya no modifican la ganancia efectiva de la cadena `osc → filtro → master`; el nivel se deriva del generador (u otros módulos aguas abajo como `VolumeModule`), preservando la intención de que el control del filtro afecte solo a su carácter espectral (resonancia) y no al volumen general.
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

### Colores base de módulos desde `default.rtp`
- Alineado el color base de `OscillatorModule` en `AudioModules.cpp` con el valor definido en el patch Reactable por defecto `com.reactable/Resources/default.rtp`, usando el color azul `#2366e1` especificado para los tangibles `type="Oscillator"` de subtipo `sine`.
- Revisados los colores de otros módulos (`Output`, `Input`, `Loop`, `Sampleplay`, controladores globales) para confirmar que ya coincidían con los valores de `default.rtp` o, cuando no había color explícito en el XML, mantenían la paleta gris definida previamente.
 - Añadido un setter público `AudioModule::OverrideColour(uint32_t)` que permite a loaders externos sobreescribir el color visual de un módulo a partir de datos de escena sin exponer directamente `set_colour`.
 - Actualizado `ReactableRtpLoader.cpp` para que, al crear cada módulo desde un `<tangible>`, lea el atributo `color` (cuando existe) y llame a `OverrideColour`, de modo que los colores de `Output`, `Input`, `Loop`, `Sampleplay` y los distintos `Oscillator` (por ejemplo `sine` azul y `saw` rojo) reflejen exactamente los valores del `default.rtp` y de cualquier otro patch Reactable cargado.
 - El color del tangible `type="Output"` se sincroniza además con `ReactablePatchMetadata::master_colour_argb`, garantizando que el nodo central y sus pulsos compartan el mismo color definido en el archivo `.rtp`.

### Overlay de IDs de módulo en modo debug
- Añadido un overlay de texto en `MainComponent_Paint.cpp` que, solo en compilaciones de depuración (`#if !defined(NDEBUG)`), dibuja a la derecha de cada nodo (tanto en la mesa como en el dock) el `logical_id` asociado al `ObjectInstance`/`AudioModule`.
- Este overlay facilita depurar problemas de mapeo de colores y rutas (por ejemplo, distinguir rápidamente los cuatro `Loop` sampler `24/29/34/39` y su color de `default.rtp`) sin contaminar la apariencia de los builds de Release.

### Color de fondo del dock
- Actualizado el color de fondo del panel de dock en `MainComponent_Paint.cpp` para que use un gris sólido `#404040` en lugar de un negro semitransparente, mejorando el contraste visual de los módulos dockeados (especialmente los que tienen colores oscuros o negros por defecto) frente al fondo general.
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

### Control de Volume global por rotación y slider
- El `VolumeModule` se inicializa ahora explícitamente con el parámetro lógico `volume` a `0.9F`, de forma que el volumen global arranca al **90%** por defecto, alineado con el comportamiento esperado de la mesa original.
- El slider lateral derecho asociado al módulo Volume, que en la UI se renderiza como un único control visible, deja de mapearse al parámetro genérico `gain` y pasa a leer/escribir directamente el parámetro `volume` del módulo, manteniendo el resto de parámetros de dinámica/FX (`compression_level`, `reverb_level`, `delay_fb`, etc.) fuera de esta interacción.
- En `MainComponent::mouseDrag`, cuando el usuario arrastra el control lateral derecho de un tangible asociado a `VolumeModule`, el valor normalizado calculado se aplica únicamente a `Scene::SetModuleParameter(..., "volume", value)`, dejando el mapeo a `gain` intacto para el resto de módulos y usando aún `q` para el control derecho de los filtros.
- `MainComponent::timerCallback` reutiliza el mapa `rotationDeltaDegrees` para aplicar también la rotación de los tangibles `Volume` al parámetro `volume`: cada vuelta completa (`±360º`) suma o resta aproximadamente `±1.0` al valor normalizado de volumen (clampado siempre a `[0.0, 1.0]`), con el mismo sentido de giro que se usa para frecuencia y tempo.
- El valor actual de `volume` se interpreta como **volumen global de Master**: antes de mapear los generadores a voces en `AudioEngine`, `MainComponent::timerCallback` busca el primer `VolumeModule` presente en la escena, lee su parámetro `volume` (por defecto `0.9F`) y lo usa como multiplicador global sobre el nivel de cada voz (`level * globalVolume`), aplicado después de la lógica de mute por objeto/conexión y del flag `masterMuted_`.
- Gracias a este cambio, cada módulo sigue teniendo su propio control de nivel local (`gain` en osciladores y otros módulos de audio), mientras que el módulo Volume actúa como un control maestro que escala toda la salida del sistema por encima del módulo `Output`, cumpliendo la separación "volumen por módulo" vs "volumen global de la mezcla".
 - Se ha alineado también la detección de clicks del slider lateral derecho del módulo Volume en `MainComponent::mouseDown` para que el handle seleccionado use el mismo parámetro lógico `volume` que se pinta en la UI; antes, el hit-test seguía leyendo el parámetro genérico `gain`, lo que desplazaba la zona de click hacia el 0% aunque el círculo visible estuviera cerca del 90%.

### Filtro de audio estable con JUCE DSP
- El motor de audio (`AudioEngine`) ha dejado de usar un biquad RBJ implementado a mano por voz y ahora emplea `juce::dsp::StateVariableTPTFilter<float>` para aplicar el filtro por voz de manera estable.
- En `audioDeviceAboutToStart` se inicializa un `StateVariableTPTFilter` por cada voz con un `ProcessSpec` mono (1 canal) y el tamaño máximo de bloque reportado por el dispositivo de audio; cada filtro se resetea tanto al iniciar como al detener el dispositivo.
- El método `setVoiceFilter` configura ahora directamente el tipo (`lowpass`, `bandpass`, `highpass`) con `setType`, y ajusta frecuencia de corte y resonancia con `setCutoffFrequency` y `setResonance`, tras limitar la frecuencia al rango físico válido por debajo de Nyquist.
- En `audioDeviceIOCallbackWithContext`, cuando una voz tiene modo de filtro distinto de cero, la muestra senoidal generada para esa voz se pasa por `filters_[v].processSample(0, raw)`, y solo entonces se acumula en la mezcla y en el buffer de forma de onda, eliminando los artefactos de “crispado” que producía la implementación biquad anterior.

### Evitar reseteos continuos del filtro por voz
- `AudioEngine::setVoiceFilter` ya no resetea incondicionalmente el estado interno del filtro en cada tick del `timerCallback`; en su lugar, actualiza solo el tipo (`lowpass`, `bandpass`, `highpass`), la frecuencia de corte (clampada a < Nyquist) y la resonancia (`Q`) en el filtro JUCE sin borrar su memoria interna.
- Si el modo de filtro es 0 (bypass) o la frecuencia de corte resulta ≤ 0 tras el clamping, el filtro se deja efectivamente en bypass (no se aplica procesamiento adicional), pero sin reseteos periódicos que pudieran introducir clicks.
- Esta eliminación de reseteos innecesarios reduce los chasquidos y el ruido "crisp" que se apreciaban al escuchar un oscilador pasando por un filtro estático, manteniendo a la vez un flujo de audio continuo incluso cuando los parámetros del filtro se mantienen fijos.

## 2025-12-11

### Configuración de envelope ADSR en FilterModule
- El módulo `FilterModule` incluye ahora valores por defecto para los parámetros de envelope ADSR (Attack, Decay, Duration, Release), inicializados en milisegundos:
  - `attack = 500.0F` ms
  - `decay = 500.0F` ms
  - `duration = 1000.0F` ms
  - `release = 500.0F` ms
- Estos valores se almacenan tanto en la estructura `Envelope` interna del módulo (`envelope_`) como en los parámetros lógicos del `AudioModule` mediante `SetParameter`, garantizando que estén disponibles tanto para el procesamiento futuro como para la UI y serialización.
- Se han añadido métodos públicos para configurar cada componente del envelope de forma independiente:
  - `set_envelope_attack(float attack_ms)`
  - `set_envelope_decay(float decay_ms)`
  - `set_envelope_duration(float duration_ms)`
  - `set_envelope_release(float release_ms)`
- Cada uno de estos métodos actualiza tanto el miembro `envelope_` como el parámetro correspondiente en el mapa de parámetros del módulo, manteniendo ambos sincronizados.
- El método `default_parameter_value` de `FilterModule` se ha extendido para devolver los valores configurados del envelope cuando se soliciten los parámetros `"attack"`, `"decay"`, `"duration"` o `"release"`, permitiendo que la UI y otros componentes lean estos valores mediante `GetParameterOrDefault`.
- Se han añadido tests en `tests/scene_tests.cpp` que verifican:
  - Los valores por defecto del envelope (500/500/1000/500 ms).
  - Que los parámetros pueden leerse correctamente mediante `GetParameterOrDefault`.
  - Que los setters actualizan tanto el envelope interno como los parámetros lógicos del módulo de forma coherente.
- Esta implementación prepara el terreno para que el motor de audio utilice estos parámetros de envelope en el procesamiento del filtro y para que la UI pueda exponer controles que permitan al usuario ajustar el envelope de forma dinámica.

### Touch interface visual feedback
- `MainComponent` incorpora ahora un sistema de feedback visual para simular interacciones táctiles sobre la interfaz:
  - **Cursor circular**: al hacer click/touch en la ventana, aparece un círculo con borde vacío (12px de radio, grosor de borde 8px) que representa la posición del dedo:
    - **Color rojo** (#FF0000 con opacidad 0.298, equivalente visual a #4C0000 sobre negro) cuando el touch se inicia en el área principal de la ventana (excluida la barra dock).
    - **Color gris claro** (0xFFCCCCCC) cuando el touch se inicia desde la barra dock.
  - **Rastro de movimiento con fade-out**: mientras el cursor se mantiene presionado (hold) en el área de la ventana (excluyendo el dock), se dibuja un rastro rojo (#FF0000) fino (1.5px) que sigue la trayectoria del movimiento.
    - El rastro implementa un sistema de desvanecimiento (fade-out) temporal: cada punto del rastro tiene un timestamp asociado y su opacidad disminuye linealmente durante 400ms hasta desaparecer completamente.
    - Los puntos se dibujan segmento por segmento con transparencia calculada en función de su edad: `alpha = 1.0 - (edad / 0.4s)`.
    - Los puntos que han superado los 400ms se eliminan automáticamente del vector durante `mouseDrag`, manteniendo el consumo de memoria bajo control.
    - El sistema de fade-out está controlado por la constante `kEnableTrailFade` (por defecto `true`), permitiendo desactivarlo por código si fuera necesario sin cambios estructurales.
  - **Limitación de memoria**: el rastro mantiene un máximo de 500 puntos (`kMaxTrailPoints`) como límite absoluto para prevenir consumo excesivo de memoria en movimientos largos.
- Variables de estado añadidas a `MainComponent.h`:
  - `isTouchActive_`: indica si hay un touch activo (mouseDown).
  - `isTouchHeld_`: indica si el touch está siendo mantenido (mouseDrag).
  - `touchStartedInDock_`: registra si el touch inicial fue dentro de la zona del dock, determinando el color del círculo.
  - `currentTouchPosition_`: posición actual del cursor/toque.
  - `touchTrail_`: vector de `TrailPoint` (estructura con `position` y `timestamp`) que acumula la trayectoria con información temporal para el fade-out.
  - `kEnableTrailFade`: constante booleana (true) que controla el sistema de desvanecimiento del rastro.
  - `kTrailFadeDurationSeconds`: constante que define la duración del fade-out (0.4 segundos).
- Actualizaciones en `MainComponent_Input.cpp`:
  - `mouseDown`: inicializa el estado del touch, determina si comenzó en el dock mediante el cálculo del área del dock con `calculateDockWidth`, limpia el trail previo y dispara `repaint()`.
  - `mouseDrag`: marca `isTouchHeld_` como true, actualiza la posición del cursor, añade puntos al trail con timestamp actual (usando `juce::Time::getMillisecondCounterHiRes()`), elimina puntos obsoletos (edad > 400ms) si el fade está activo, y dispara `repaint()`.
  - `mouseUp`: resetea todo el estado del touch (`isTouchActive_`, `isTouchHeld_`, `touchStartedInDock_`), limpia el trail y dispara `repaint()`.
- Renderizado en `MainComponent_Paint.cpp`:
  - El rastro se dibuja segmento por segmento (línea entre puntos consecutivos), calculando la transparencia de cada segmento según la edad del punto destino.
  - Luego se dibuja el círculo del cursor con `drawEllipse` usando el color correspondiente (#4C0000 o gris claro) y un grosor de 2px.
- El estado `isTouchHeld_` queda almacenado para poder extenderlo en futuras features que requieran conocer si el usuario está manteniendo presionado el cursor.

### Touch line cutting para alternar mute
- Implementado sistema de "corte" de líneas de audio mediante gestos táctiles para alternar el estado de silencio (mute/unmute) de forma intuitiva:
  - **Detección de intersección**: mientras el cursor está en hold (arrastre), el sistema detecta cuando el trail cruza líneas de audio:
    - **Líneas objeto-a-centro** (blancas): representan el output de sonido de cada módulo hacia el master.
    - **Hardlinks módulo-a-módulo** (rojas): conexiones fijas entre módulos.
    - **Conexiones dinámicas** (blancas curvas): conexiones que dependen del cono geométrico de 120º.
  - **Marcado visual con toggle**: cada vez que el cursor cruza una línea, ésta se marca o desmarca (toggle) y cambia a **color amarillo con grosor aumentado (3px)**, proporcionando feedback visual inmediato de que está marcada para alternar su mute.
  - **Aplicación al soltar**: cuando se suelta el ratón (`mouseUp`), todas las líneas marcadas alternan su estado de mute:
    - Líneas mutadas pasan a activas.
    - Líneas activas pasan a mutadas.
  - **Algoritmo de intersección**: nueva función helper `lineSegmentsIntersect` en `MainComponentHelpers` que calcula la distancia mínima entre dos segmentos de línea y detecta intersección con un threshold de 15px.
  - **Sistema de transiciones enter/exit**: mantiene sets de intersección actual (`touchCurrentlyIntersectingConnections_`, `touchCurrentlyIntersectingObjects_`) que detectan cuando el cursor **entra** en la zona de intersección (intersects && !wasIntersecting) para hacer toggle, y cuando **sale** (!intersects && wasIntersecting) para actualizar el estado, permitiendo cruzar repetidamente la misma línea con toggle alternado en cada cruce:
    - 1ª pasada: marca para mute (amarillo).
    - 2ª pasada: desmarca (vuelve a color original).
    - 3ª pasada: vuelve a marcar (amarillo), etc.
  - **Exclusión de drag de módulos**: no detecta líneas cuando se está moviendo un módulo (`draggedObjectId_ != 0`), evitando marcados accidentales al reposicionar objetos en la mesa.
  - **Estructuras de tracking**:
    - `touchCutConnections_`: set de claves de conexión marcadas para toggle al soltar.
    - `touchCutObjects_`: set de IDs de objetos cuyas líneas al centro están marcadas para toggle.
    - `touchCurrentlyIntersectingConnections_`/`touchCurrentlyIntersectingObjects_`: sets temporales para tracking de transiciones.
- El sistema respeta la lógica existente de visibilidad de líneas (área musical, conexiones activas, controladores globales).
- Código actualizado:
  - `MainComponent.h`: añadidos 4 sets de tracking (cut + currentlyIntersecting).
  - `MainComponent_Input.cpp`: detección de transiciones en `mouseDrag` con condición `draggedObjectId_ == 0`, aplicación de toggle en `mouseUp`, limpieza en `mouseDown`.
  - `MainComponent_Paint.cpp`: renderizado condicional de líneas en amarillo con grosor aumentado.
  - `MainComponentHelpers.{h,cpp}`: función `lineSegmentsIntersect` para geometría de intersección.

### Click-and-hold temporary mute con split rendering en líneas de audio
- Implementado nuevo comportamiento para la interacción con líneas de audio: al hacer click en una línea, el módulo se silencia temporalmente mientras se mantiene presionado el botón, con visualización parcial de la waveform:
  - **Mute temporal durante hold**: al hacer click en una línea de audio (tanto object-to-center como module-to-module), el sistema:
    - Calcula la posición normalizada (0-1) del punto de click a lo largo de la línea.
    - Aplica mute inmediato al módulo/conexión correspondiente.
    - Almacena el estado de mute previo: si la línea ya estaba silenciada antes del click, se recuerda para no desmutar al soltar.
    - Al soltar el click (`mouseUp`), se desmutea automáticamente solo si la línea no estaba previamente muteada.
  - **Split rendering visual**: durante el hold, la línea se renderiza en dos segmentos:
    - **Segmento activo** (source → split point): muestra la waveform de audio normalmente si hay señal activa, demostrando que hasta ese punto la señal sigue siendo audible.
    - **Segmento silenciado** (split point → destination): se dibuja con línea punteada (dashed) con transparencia reducida (0.6F), indicando visualmente que desde ese punto en adelante la señal está muted.
  - **Cursor blanco sin trail**: mientras se mantiene el click en una línea de audio:
    - El cursor cambia a **color blanco** (`juce::Colours::white.withAlpha(0.8F)`), diferenciándose del rojo normal para indicar modo de interacción especial.
    - El **trail rojo desaparece** (no se dibuja), eliminando el rastro de movimiento y proporcionando feedback visual claro de que estamos en modo "hold mute" y no en modo "line cutting".
  - **Estructuras de estado**:
    - Nuevo `ConnectionHoldState` en `MainComponent.h` que almacena:
      - `connection_key` o `object_id`: identifica la línea afectada.
      - `is_object_line`: distingue entre línea object-to-center (true) o module-to-module (false).
      - `split_point`: posición normalizada (0-1) donde se hizo click.
      - `was_previously_muted`: estado de mute anterior al click para restauración correcta.
    - Variable `std::optional<ConnectionHoldState> activeConnectionHold_` que marca si hay una línea siendo held.
  - **Implementación**:
    - `MainComponent_Input.cpp`:
      - `mouseDown`: al detectar click en línea (segment hit-test), calcula el `splitPoint` mediante proyección vectorial del punto de click sobre el segmento (producto punto normalizado), almacena el estado previo de mute, activa el mute inmediato y guarda todo en `activeConnectionHold_`.
      - `mouseUp`: si hay `activeConnectionHold_` activo, desmutea la línea solo si `!was_previously_muted`, luego resetea `activeConnectionHold_`.
    - `MainComponent_Paint.cpp`:
      - Render de líneas object-to-center: si `activeConnectionHold_` está activo y coincide con la línea, calcula `splitPoint` interpolado y renderiza:
        - Primer segmento con waveform (si audio activo) desde center hasta split.
        - Segundo segmento dashed desde split hasta object.
      - Render de conexiones module-to-module: similar split rendering aplicado a hardlinks y dynamic connections, usando líneas rectas desde p1 a splitPoint con waveform, y dashed desde splitPoint a p2.
      - Cursor: condición `activeConnectionHold_.has_value()` cambia color a blanco y desactiva el dibujado del trail rojo.
  - **Compatibilidad con line cutting**: el sistema convive con el mecanismo de line cutting previo (drag con trail para toggle permanente de mute), diferenciándose por el cursor blanco vs rojo y la ausencia de trail durante hold.
- Este comportamiento proporciona control temporal fino sobre el audio, permitiendo escuchar cómo suena la señal hasta un punto específico de la cadena, útil para debugging y performance en vivo.
- Código actualizado:
  - `MainComponent.h`: añadidos `#include <optional>`, estructura `ConnectionHoldState` y variable `activeConnectionHold_`.
  - `MainComponent_Input.cpp`: cálculo de split point en hit-test de líneas, almacenamiento de estado hold, lógica de unmute condicional en `mouseUp`.
  - `MainComponent_Paint.cpp`: render condicional de split en líneas object-to-center y module-to-module, cambio de cursor a blanco y supresión de trail durante hold.

#### Correcciones de bugs en click-and-hold mute
- **Separación de interacciones**: se ha corregido la lógica para prevenir que el sistema de "line cutting" (cursor rojo con trail) se active durante el modo "hold mute" (cursor blanco sin trail):
  - Añadida condición `!activeConnectionHold_.has_value()` en la detección de intersecciones de líneas durante `mouseDrag`.
  - Esto asegura que ambos modos de interacción sean mutuamente excluyentes y no interfieran entre sí.
- **Corrección de dirección de split rendering**: se ha invertido la dirección de los segmentos visuales para reflejar correctamente el flujo de audio:
  - **Segmento con waveform**: ahora va desde el **módulo (origen)** hasta el **punto de click**, mostrando que la señal de audio se genera en el módulo y viaja hasta donde el usuario presiona.
  - **Segmento punteado/silenciado**: ahora va desde el **punto de click** hasta la **salida (centro/master o módulo destino)**, indicando que desde ese punto hacia adelante la señal está cortada/muteada.
  - En líneas object-to-center: waveform de `{cx, cy}` (objeto) a `splitPoint`, dashed de `splitPoint` a `centre` (master).
  - En líneas module-to-module: waveform de `p1` (from/source) a `splitPoint`, dashed de `splitPoint` a `p2` (to/destination).
- **Visualización de waveform durante mute**: se ha corregido para que la waveform se muestre **siempre** en el segmento activo (origen → split) cuando hay señal de audio, **incluso si el módulo está temporalmente muteado** durante el hold:
  - El código de render durante `isBeingHeld` ya no filtra por estado de mute para decidir si dibuja la waveform.
  - Esto permite al usuario ver visualmente que el módulo está generando audio hasta el punto de corte, proporcionando feedback más intuitivo.
- **Unmute incondicional al soltar**: se ha corregido la lógica de `mouseUp` para que **siempre desmutee** la línea al soltar el click, independientemente de si estaba muteada previamente o no:
  - Eliminado el campo `was_previously_muted` de `ConnectionHoldState`.
  - Eliminadas las condiciones que chequeaban el estado previo antes de desmutar.
  - Ahora `mouseUp` simplemente llama a `erase()` en `mutedObjects_` o `mutedConnections_` sin condiciones, garantizando que al soltar el click la línea siempre quede activa (no muted).
- Estas correcciones mejoran la coherencia visual y funcional del sistema de control temporal de mute, alineando el comportamiento con las expectativas del usuario sobre el flujo direccional de la señal de audio.
- **Corrección crítica - generación de waveform en AudioEngine con level=0**: se ha corregido el problema fundamental donde la waveform no se mostraba durante el hold mute (mostrando solo línea blanca):
  - **Problema raíz en tres capas**:
    1. En `MainComponent_Audio.cpp`: cuando un módulo se muteaba, `level = 0` y no entraba en `if (level > 0)`, no asignando voice index.
    2. Tras corregir eso para procesar con `isBeingHeld`, el voice se configuraba con `outputLevel = 0` en el AudioEngine.
    3. **Problema final**: En `AudioEngine.cpp`, la condición `if (level > 0.0F && freq > 0.0)` impedía la generación del oscillador cuando `level = 0`, por lo que `voiceWaveformBuffer_` quedaba en 0 y no había datos que visualizar.
  - **Solución implementada en tres capas**:
    1. **AudioEngine** (`AudioEngine.cpp`): Modificada la lógica de generación de waveform para que funcione **independientemente del level**:
       - Cambiada condición de `if (level > 0.0F && freq > 0.0)` a solo `if (freq > 0.0)`.
       - Los oscilladores (sine, saw, square, noise) ahora generan waveform a **amplitud completa** (sin multiplicar por level).
       - La waveform sin escalar se almacena en `voiceWaveformBuffer_[v][bufIndex]` para visualización.
       - El `level` se aplica **después**, solo para el output mezclado: `scaledOutput = s * level`.
       - Esto permite capturar la forma de onda real del oscillador incluso cuando `level = 0`.
    2. **Motor de audio** (`MainComponent_Audio.cpp`): Lógica para procesar voices durante `isBeingHeld`:
       - Añadida detección de `isBeingHeld` verificando si objeto/conexión está en hold.
       - Cambiada condición a `if ((calculatedLevel > 0.0F || isBeingHeld))` para forzar procesamiento.
       - El `outputLevel` es 0 durante hold, silenciando la salida pero permitiendo generación interna.
    3. **Capa de renderizado** (`MainComponent_Paint.cpp`): Búsqueda directa de `voiceIndex` sin dependencias de flags de mute durante `isBeingHeld`.
  - **Resultado**: La waveform ahora se visualiza correctamente en el segmento activo (módulo → split point) durante el hold mute, mostrando la forma de onda real del audio generado internamente aunque la salida esté completamente silenciada.
  - **Corrección adicional - consistencia de waveform**: se ha corregido un problema donde la forma de onda mostrada cambiaba completamente al hacer click en diferentes posiciones de la línea:
    - **Problema**: El número de segmentos usado para dibujar la waveform variaba según `splitT` (posición del click), causando diferente muestreo del buffer `voiceWaveforms[]` en cada click.
    - **Solución**: Mantener siempre el mismo número de segmentos fijo (72 para object-to-center, 64 para module-to-module), independientemente de la longitud de la línea dibujada.
    - Esto garantiza que se muestree siempre la misma porción del buffer de waveform, mostrando un patrón consistente de la forma de onda sin importar dónde se haga click en la línea.
