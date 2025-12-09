# Conceptos de arquitectura – Sistema Reactable-like

Este documento esboza una posible arquitectura para un sistema tipo Reactable, moderno y multiplataforma, con foco en Linux.

## 1. Visión de alto nivel

Dividimos el sistema en varias **capas** y **servicios** relativamente independientes:

1. **Capa de hardware / IO**
   - Cámaras de vídeo.
   - Dispositivos de audio.
   - Dispositivos MIDI/controladores externos.

2. **Servicio de tracking** (inspirado en reacTIVision)
   - Captura y procesado de vídeo.
   - Detección y tracking de marcadores/objetos.
   - Emisión de eventos de alto nivel (TUIO/OSC/otros).

3. **Core de interacción y modelo de mundo**
   - Representación interna de la **mesa** y de los **módulos**.
   - Gestión del grafo de conexiones de audio y control.
   - Mapeo entre eventos de tracking y acciones sobre el modelo (crear, mover, rotar, conectar módulos).

4. **Motor de audio**
   - Síntesis, efectos, ruteo y mezcla.
   - Sincronización y relojes.
   - Interfaces de entrada/salida hacia el sistema operativo.

5. **Frontends de usuario (UI)**
   - Aplicación de escritorio (mesa virtual + visualización de objetos físicos cuando existan).
   - Herramientas auxiliares (calibración, monitor de debug, editor de módulos/presets).

6. **Persistencia y configuración**
   - Gestión de escenas, presets y configuración de hardware.
   - Ficheros de proyecto.

## 2. Separación en procesos / servicios

Una decisión clave es si todo vive en un único proceso o se divide en varios servicios comunicados por IPC/red.

### Opción A – Monolito modular

- Tracking, core, audio y UI en un solo programa.
- Ventaja: menor complejidad de despliegue, comunicación interna más simple.
- Inconveniente: más difícil aislar fallos, menos flexible para escalado y headless.

### Opción B – Arquitectura orientada a servicios (recomendada a medio plazo)

- **Servicio de tracking** separado (similar a reacTIVision):
  - Expone eventos de posición/rotación de objetos.
  - Puede ejecutarse en otra máquina si hace falta.

- **Core + motor de audio** como proceso principal de la aplicación musical.

- **Frontends** (UI de mesa, herramientas de calibración) que se conectan al core y/o al servicio de tracking.

Para un MVP se puede empezar con algo cercano a la opción A, pero diseñando las interfaces como si fueran servicios separados para facilitar la posterior separación.

## 3. Modelo de datos principal

Entidades clave:

- `Surface` / `Mesa` – define dimensiones, sistema de coordenadas, configuración visual.
- `ObjectInstance` – instancia tangible o virtual presente en la mesa:
  - ID de tracking (si viene del servicio de tracking).
  - ID lógico (tipo de módulo asociado).
  - Posición y orientación.

- `Module` – entidad lógica en el core musical:
  - Tipo (oscilador, filtro, secuenciador, etc.).
  - Parámetros.
  - Puertos de entrada/salida (audio/control).

- `Connection` – relación entre puertos de módulos.
- `Scene` – conjunto de módulos, conexiones y estado asociado a una sesión.

Este modelo debe ser **serializable** (para guardado/carga) y lo bastante abstracto para no depender fuertemente de la fuente de tracking.

## 4. Interfaces entre componentes

### 4.1. Tracking → Core de interacción

- Canal de eventos:
  - `object_added(id, typeHint, x, y, angle)`
  - `object_updated(id, x, y, angle, velocity, accel)`
  - `object_removed(id)`

- Transporte:
  - Inicialmente: TUIO/OSC o mensajes propios vía UDP/TCP/WebSocket.
  - A medio plazo: una API más rica (p.ej., gRPC, IPC local) si hace falta.

### 4.2. Core de interacción → Motor de audio

- Definición de grafo de audio:
  - Crear/eliminar módulos.
  - Conectar/desconectar puertos.
  - Actualización de parámetros.

- API interna de alto rendimiento (en memoria), con hilo de audio separado y comunicación con colas lock-free o similar.

### 4.3. Core de interacción ↔ UI

- Canal bidireccional:
  - La UI observa el estado (escena, módulos, posiciones, conexiones).
  - El usuario actúa a través de la UI (mueve objetos virtuales, cambia parámetros, guarda escenas).

- Transporte:
  - En un principio puede ser llamadas directas en memoria.
  - Compatible con un futuro frontend remoto (UI web) mediante WebSocket/API HTTP.

## 5. Consideraciones tecnológicas (sin cerrar aún)

> Nota: aquí no se fija definitivamente el stack, sólo se anotan criterios.

- **Lenguaje para core/audio**:
  - C++ o Rust son candidatos naturales por rendimiento y ecosistema de audio.
  - Python, etc., podrían intervenir en capas de scripting, pero no en el core de tiempo real.

- **Frameworks de audio** (Linux):
  - JACK, ALSA directamente, o integración con PipeWire.

- **Tracking**:
  - OpenCV para captura y procesamiento básico.
  - Opcionalmente librerías de markers (ArUco, AprilTag) como alternativa a marcadores amoeba.

- **UI de escritorio**:
  - Toolkits multiplataforma (Qt, GTK, JUCE) o frontends basados en web (Electron, Tauri, etc.) según preferencias del proyecto.

## 6. Patrón de concurrencia y tiempo real

- **Hilo de audio dedicado**:
  - Ejecuta el procesamiento DSP con prioridad alta.
  - Recibe cambios de estado (parámetros, conexiones) mediante estructuras lock-free o colas de mensajes.

- **Hilo(s) de tracking**:
  - Procesan frames de cámara y publican eventos.

- **Hilo(s) de UI**:
  - Renderizan visualización y gestionan input de usuario.

La coordinación entre estas partes debe minimizar bloqueos y asegurarse de que la experiencia de audio no sufra glitches.

## 7. Persistencia y configuración

- Formatos de archivo legibles (por ejemplo JSON, YAML o similar) para escenas y presets.
- Distinción entre:
  - Configuración global del sistema (audio, cámara, rutas de recursos).
  - Escenas/proyectos específicos (módulos, conexiones, parámetros).

## 8. Roadmap arquitectónico

1. **Definir modelo de datos y API interna** del core (módulos, conexiones, escenas).
2. **Implementar motor de audio básico** con algunos módulos simples.
3. **Crear UI de mesa virtual** que interactúe con el core (sin tracking físico).
4. **Diseñar e implementar servicio de tracking** (o integrar uno existente) con una interfaz clara hacia el core.
5. **Extraer componentes en procesos/servicios separados** donde tenga sentido (tracking, herramientas, etc.).

Este documento es una primera aproximación y servirá como mapa conceptual para decisiones posteriores de diseño más concretas.