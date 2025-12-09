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

### Limpieza de idioma y comentarios
- Comentarios de `Scene.h` traducidos al inglés para seguir la convención del proyecto.
- Título de la UI en `MainComponent::paint` actualizado a "RectaiTable - Example scene".

## Próximos pasos sugeridos (Roadmap corto)
- **Tracking/protocolo**:
  - Extender `rectai-tracker` para enviar mensajes OSC reales compatibles con el protocolo `/rectai/object` y `/rectai/remove`.
  - Evaluar si se quiere aproximar al formato TUIO estándar o mantener este canal OSC simplificado como modo de debug.
- **Audio**:
  - Introducir un esqueleto de motor de audio en JUCE (por ejemplo, `AudioAppComponent` o `AudioDeviceManager` + `AudioProcessorGraph`).
  - Mapear módulos lógicos (`ModuleKind`) a nodos de audio concretos en el grafo.
- **UI/Escenas**:
  - Definir un formato de serialización simple (probablemente JSON) para guardar/cargar escenas desde `rectai::Scene`.
  - Añadir comandos básicos a la UI para cargar/guardar presets de escena.
