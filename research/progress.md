# Progreso de implementación

## 2025-12-09

### Modelo y UI básica
- Modelo de dominio inicial implementado en `rectai::Scene` (`Scene.h/.cpp`), con:
  - `ObjectInstance` (id de tracking, id lógico, posición normalizada, ángulo).
  - `Module` + `ModuleKind` y descriptores de puertos.
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
  - Punto de partida para evolucionar hacia un grafo de módulos que mapee `rectai::ModuleKind` a nodos de procesamiento.
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

### Refinado de interfaz visual según `ui-interface.md`
- Fondo y núcleo central:
  - Se mantiene el lienzo circular azul con gradiente y viñeta, actuando como superficie principal de la mesa reactiva.
  - El núcleo central incorpora ahora ondas expansivas más rítmicas y se usa como punto de convergencia visual de las conexiones.
- Nodos/objetos tangibles:
  - Cada `ObjectInstance` toma su color base a partir del `ModuleKind` asociado (`oscillator`, `filter`, `effect`, `sampler`, `controller`), con variantes azules, verdes, púrpuras y naranjas para diferenciar tipos.
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

### Flujo de señal basado en proximidad y mute en cadena
- Conexiones espaciales entre instrumentos:
  - Se introduce una heurística geométrica `isConnectionGeometricallyActive` que considera distancia (radio máximo en coordenadas normalizadas) y orientación (el segundo objeto debe estar aproximadamente “delante” del primero según su ángulo) para decidir si una conexión `Scene::connections` está activa visualmente.
  - Cuando una conexión módulo→módulo está activa (por ejemplo, `osc1 → filter1`), solo se dibuja la línea curvada entre los instrumentos y la línea directa desde el módulo fuente al Master desaparece. Visualmente, el sonido “pasa” primero por el filtro y luego al centro.
- Rutas hacia Master y líneas mutables:
  - Las líneas centro→objeto solo se dibujan para instrumentos que no estén alimentando a otro módulo a través de una conexión activa. El último módulo de una cadena (p. ej. `filter1`) es el que mantiene la conexión visible hacia el Master.
  - El click de mute sobre una línea centro→objeto solo es válido para estos módulos “terminales”; si un instrumento está alimentando a otro, su línea directa a Master se considera inactiva y no se puede mutear directamente.
- Propagación de mute en la cadena de audio:
  - En `MainComponent::timerCallback` el oscilador (`osc1`) se enruta dinámicamente:
    - Si no hay conexión activa `osc1 → filter1`, el oscilador se conecta directamente al Master y el mute depende solo de su propia línea al centro.
    - Si hay conexión activa hacia el filtro, el oscilador se considera alimentando a `filter1` y la línea de `filter1` hacia el Master controla el silencio de toda la cadena. Mutear esa línea silencia también al oscilador.
  - El parámetro `Gain` del oscilador controla el nivel cuando va directo a Master, mientras que cuando pasa por el filtro se utiliza el `Gain` del filtro como control principal de loudness.

### Próximos pasos sugeridos (UI)
- Introducir un efecto de glow/bloom real mediante un pipeline de renderizado con OpenGL o similar, en lugar de aproximarlo solo con halos y transparencias.
- Experimentar con blending aditivo global para que las intersecciones de líneas y halos incrementen el brillo de forma más marcada.
- Hacer que algunos parámetros visuales (arcos, intensidad de auras, velocidad de pulsos) respondan a métricas de audio reales en `AudioEngine` (niveles RMS, envolventes, etc.).

## Próximos pasos sugeridos (Roadmap corto)
- **Tracking/protocolo**:
  - Extender `rectai-tracker` para enviar mensajes OSC reales compatibles con el protocolo `/rectai/object` y `/rectai/remove` (más allá de los datos sintéticos actuales).
  - Evaluar si se quiere aproximar al formato TUIO estándar o mantener este canal OSC simplificado como modo de debug.
- **Audio**:
  - Evolucionar `AudioEngine` desde el tono único actual hacia varias voces/módulos (p.ej., una voz por `ObjectInstance` asociado a `ModuleKind::kOscillator`).
  - A medio plazo, introducir un `AudioProcessorGraph` o equivalente que mapee `ModuleKind` a nodos de procesamiento reutilizables.
- **UI/Escenas**:
  - Completar la deserialización (carga) de escenas usando el formato line-based `rectai_scene_v1` de `SceneSerialization` o migrar a un formato JSON cuando se introduzca una librería dedicada.
  - Añadir comandos básicos a la UI para guardar la `Scene` actual en disco y recargarla (presets simples).
