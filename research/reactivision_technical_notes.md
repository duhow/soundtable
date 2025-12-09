# reacTIVision – Notas técnicas

Este documento recopila información técnica sobre **reacTIVision**, el sistema de visión por computador utilizado históricamente por Reactable y otras interfaces tangibles.

> Nota: La información se basa en conocimiento disponible hasta 2024 y en documentación pública del proyecto reacTIVision.

## 1. Propósito de reacTIVision

- Es un **framework de tracking de marcadores fiduciales** en tiempo real, pensado para interfaces tangibles.
- Proporciona la detección robusta de:
  - **Objetos** con IDs únicos (marcadores de tipo amoeba/TUIO).
  - **Cursors** (puntos de contacto) para interacción tipo multitouch.
- Expone la información a través del protocolo **TUIO** (basado en OSC/UDP), permitiendo que distintas aplicaciones consuman los datos de tracking sin acoplarse al código de visión.

## 2. Arquitectura general

1. **Captura de vídeo**
   - Entrada desde cámara (habitualmente USB/FireWire, situada debajo de la superficie).
   - Resolución y framerate adaptados al caso de uso (p.ej. 640x480 a 60 fps, o superior).

2. **Preprocesado**
   - Conversión a escala de grises.
   - Umbralización / binarización para separar marcadores del fondo.
   - Opcionalmente, correcciones geométricas (distorsión de lente, perspectiva).

3. **Detección de marcadores**
   - Búsqueda de regiones candidatas.
   - Análisis de forma (contornos) y patrones internos.
   - Reconstrucción de la orientación a partir de la forma del marcador.

4. **Reconocimiento e identificación**
   - Los marcadores type amoeba codifican un ID en su forma orgánica.
   - El sistema asocia un ID estable y calcula posición (x, y) y ángulo.

5. **Seguimiento (tracking)**
   - Asociación de detecciones frame a frame (tracking temporal).
   - Gestión de aparición/desaparición de objetos, pérdida temporal, etc.

6. **Salida de datos (TUIO)**
   - Generación de mensajes TUIO por UDP.
   - Dos tipos principales de entidades: **obj** (objetos) y **cur** (cursores).
   - Cada mensaje incluye posición normalizada, ángulo, velocidad, aceleración, estado.

## 3. TUIO en pocas palabras

- **TUIO** (Tangible User Interface Object) es un protocolo estándar de facto para interfaces tangibles.
- Basado en **OSC (Open Sound Control)** sobre UDP.
- Envía mensajes con información como:
  - ID de sesión.
  - ID simbólico (por ejemplo, tipo de marcador).
  - Posición normalizada (x, y) en el rango [0, 1].
  - Ángulo de rotación en radianes o grados (según implementación de alto nivel).
  - Velocidad y aceleración.
- Permite que múltiples aplicaciones, escritas en lenguajes distintos, se conecten al mismo servidor reacTIVision para recibir eventos.

## 4. Características técnicas relevantes

- Implementado históricamente en C++ con dependencias de librerías de visión (OpenCV en algunas variantes) y librerías de I/O de cámara específicas.
- Multiplataforma (Linux, macOS, Windows), aunque el grado de soporte y la facilidad de compilación pueden variar con el tiempo.
- Diseñado para ser **headless**, sin lógica de aplicación musical: sólo tracking.
- Optimizado para **baja latencia**, a menudo mediante procesamiento en CPU sin hardware especializado.

## 5. Diseño de marcadores y superficie

- Los marcadores de reacTIVision son de tipo **amoeba**, con formas orgánicas que codifican un ID de forma robusta.
- La superficie puede ser:
  - Una mesa retroproyectada (proyector debajo, pantalla traslúcida arriba).
  - Una pantalla LCD con cámara inferior o superior.
  - Cualquier superficie plana con cámara que la vea completa.
- Los objetos tangibles suelen ser cilindros o piezas con base plana donde se pega el marcador.

## 6. Limitaciones y retos técnicos

- Sensible a condiciones de iluminación extremas (reflejos, muy poca luz o luz muy variable).
- La calidad de la cámara (exposición, lente, velocidad) afecta mucho a la robustez.
- Calibración de cámara y superficie necesaria para conseguir coordenadas precisas.
- Ocultaciones (cuando un objeto tapa a otro) pueden provocar pérdida temporal de tracking.

## 7. Lecciones para nuestro proyecto

Puntos clave que podemos reutilizar o modernizar:

- Mantener una **capa de tracking desacoplada** que emita eventos de alto nivel (posiciones, ángulos, IDs) mediante un protocolo estándar.
- Considerar TUIO/OSC como interfaz de red, pero valorar también alternativas modernas (WebSockets, gRPC, etc.) para integraciones nuevas.
- Permitir que distintos frontends de "mesa" consuman los datos de tracking (por ejemplo, nuestra aplicación Reactable-like, visualizadores, herramientas de depuración).
- Evaluar el uso de librerías de visión actuales (OpenCV, MediaPipe, soluciones de AR) y, eventualmente, aprovechar GPU.

## 8. Opciones de diseño inspiradas en reacTIVision

Para nuestro rediseño moderno podríamos:

- Implementar un **servicio de tracking modular** que:
  - Soporte distintas fuentes de vídeo (cámara física, stream, vídeo de prueba).
  - Permita cambiar el algoritmo de detección (fiduciales clásicos, markers tipo ArUco, etc.).
  - Emita eventos en varios formatos (TUIO, OSC genérico, WebSocket JSON, etc.).

- Separar claramente:
  - **Core de tracking** (headless, sin UI).
  - **Herramienta de calibración** (GUI sencilla para alinear cámara y superficie).
  - **Monitor de debug** (visualización de marcadores, IDs, FPS).

Estas notas servirán como base técnica para el diseño de la capa de tracking en nuestro sistema.