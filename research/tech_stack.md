# Decisión de stack tecnológico

Este documento consolida la decisión sobre el lenguaje, frameworks y organización técnica para el proyecto Reactable-like.

## 1. Objetivos que condicionan el stack

- **Rendimiento y eficiencia**:
  - Procesamiento de audio en tiempo real con baja latencia.
  - Rendering gráfico fluido para la "mesa" interactiva.
  - Evitar runtimes pesados tipo Electron.
- **Multiplataforma con prioridad Linux**:
  - Soporte sólido en Linux (ALSA/JACK/PipeWire, cámaras habituales, drivers comunes).
  - Soporte también en Windows (audio, UI y cámaras), sin necesidad de tocar demasiado el core.
- **Arquitectura modular**:
  - Separar tracking, core musical, motor de audio y UI.
  - Posibilidad de ejecutar el servicio de tracking como proceso independiente (estilo reacTIVision).
- **Extensibilidad y mantenibilidad**:
  - Facilidad para añadir nuevos módulos de audio, nuevas formas de interacción y nuevos frontends.

## 2. Decisión principal

### 2.1. Lenguaje base del core y UI principal

- **Lenguaje**: `C++20` (o estándar reciente equivalente disponible en los toolchains objetivo).
- **Framework principal**: **JUCE**.

Justificación:

- JUCE está ampliamente utilizado en la industria del audio profesional para:
  - Sintetizadores.
  - DAWs.
  - Plugins (VST/AU/CLAP, etc.).
- Proporciona en un único framework:
  - Backend de **audio** multiplataforma (ALSA/JACK/PipeWire en Linux; WASAPI/ASIO en Windows).
  - **MIDI**, **OSC**, timers, hilos, manejo de archivos, etc.
  - Un sistema de **UI nativo y ligero**, suficiente para implementar una mesa 2D con objetos interactivos sin depender de un navegador.
- Permite estructurar bien la separación entre:
  - Hilo de audio de alta prioridad.
  - Lógica de interacción / modelo.
  - UI.

Conclusión: **el núcleo de la aplicación musical (core + motor de audio + UI de mesa virtual) se implementará en C++ usando JUCE**.

### 2.2. Servicio de tracking

- **Lenguaje**: `C++`.
- **Librería principal de visión**: **OpenCV**.
- **Librería de marcadores** (a decidir en detalle más adelante):
  - Opciones: ArUco, AprilTag, o implementación propia inspirada en marcadores tipo amoeba.

Rol del servicio de tracking:

- Proceso independiente (similar a reacTIVision) que se encarga de:
  - Capturar vídeo desde cámara(s).
  - Detectar marcadores y estimar posición (x, y) y orientación.
  - Gestionar la persistencia de IDs de sesión.
  - Exportar eventos de tracking a través de la red o IPC.

Comunicación con el core:

- Protocolo de salida:
  - Preferentemente **TUIO/OSC** (por compatibilidad con ecosistema y herramientas existentes).
  - Posibilidad de ofrecer en paralelo un canal alternativo (por ejemplo, mensajes OSC personalizados o WebSocket con JSON) si se considera útil para debugging o frontends web.

Conclusión: **el tracking se implementará como servicio C++/OpenCV separado, que comunica con el core vía TUIO/OSC u otro protocolo ligero similar**.

## 3. Alternativas consideradas

Aunque la decisión principal es C++ + JUCE + OpenCV, se han considerado otras opciones:

### 3.1. Rust + UI nativa (egui/iced)

- Core y audio en Rust (librerías como `cpal`, `kira`/`oddio`, `midir`, `rosc`).
- UI en Rust con `egui` o `iced`.
- Pros:
  - Seguridad de memoria y concurrencia más robusta.
  - Ecosistema moderno.
- Contras:
  - El ecosistema de audio en Rust sigue siendo más joven que JUCE.
  - Faltan herramientas tan específicas y maduras para audio interactivo como en el mundo JUCE.

Se considera una opción atractiva a medio/largo plazo, pero **no es la elección principal** para el arranque del proyecto.

### 3.2. Core en C++/Rust + UI en Qt/QML

- Motor de audio y lógica en C++ (posiblemente con parte de JUCE sólo para audio) o Rust.
- UI en Qt/QML para una experiencia gráfica rica y animada.

Pros:

- Qt/QML es potente para UIs 2D complejas y animaciones.
- Buen soporte multiplataforma.

Contras:

- Combinación de dos frameworks grandes (Qt + JUCE o Qt + motor audio propio) aumenta la complejidad.
- Más superficie de mantenimiento y riesgo de duplicar funcionalidades.

También se descarta como opción principal para el MVP, pero se mantiene como referencia si se necesitan capacidades gráficas específicas que excedan lo cómodo con JUCE.

## 4. Mapeo de arquitectura a tecnologías

A partir del documento de `architecture_concepts.md`, los componentes se mapean así:

1. **Servicio de tracking**
   - Lenguaje: C++.
   - Librerías: OpenCV (+ librería de marcadores), sockets UDP para TUIO/OSC.
   - Proceso independiente, ejecutable tipo `soundtable-tracker`.

2. **Core de interacción y modelo de mundo**
   - Lenguaje: C++.
   - Ubicación: parte de la aplicación principal JUCE.
   - Responsabilidades:
     - Representar la mesa, objetos/módulos y conexiones.
     - Escuchar eventos de tracking (vía OSC/TUIO) y mapearlos a acciones internas.

3. **Motor de audio**
   - Lenguaje: C++.
   - Framework: JUCE (AudioDeviceManager, AudioProcessorGraph, etc.).
   - Diseño:
     - Grafo de módulos (osciladores, filtros, efectos, etc.).
     - Hilo de audio dedicado con alta prioridad.

4. **UI de mesa y herramientas auxiliares**
   - Lenguaje: C++.
   - Framework: JUCE (Component, Graphics, etc.).
   - Funcionalidades:
     - Visualización de la mesa y los objetos (físicos y virtuales).
     - Edición de escenas, presets, parámetros.
     - Herramientas de depuración (vista de estado, monitor de carga, etc.).

5. **Persistencia y configuración**
   - Lenguaje: C++.
   - Formato de ficheros: muy probablemente JSON o YAML (a definir más adelante).
   - Uso tanto en core como en UI para guardar/cargar escenas y configuración.

## 5. Roadmap técnico inicial (orientado a implementación)

1. **Definir modelo de datos y API interna del core**
   - Entidades: Surface/Mesa, ObjectInstance, Module, Connection, Scene, etc.
   - Interfaces para crear/eliminar módulos, conectar, actualizar parámetros.

2. **Crear un MVP de aplicación JUCE sin tracking físico**
   - Mesa virtual con objetos manejados con ratón/trackpad.
   - Motor de audio básico con algunos módulos simples.
   - Guardado/carga de escenas en un formato simple.

3. **Implementar el servicio de tracking C++/OpenCV**
   - Captura de cámara.
   - Detección de marcadores.
   - Emisión de eventos por TUIO/OSC.

4. **Integrar el tracking físico con el core**
   - Módulo receptor de TUIO/OSC en la aplicación JUCE.
   - Mapeo de IDs de marcadores a tipos de módulos.

5. **Refinar UI y modularidad**
   - Mejorar visualización de la mesa y feedback al usuario.
   - Diseñar sistema de plugins de módulos de audio.

Este documento fija el punto de partida tecnológico. Cambios futuros deberán actualizar este archivo para mantener alineada la visión técnica con la implementación real.