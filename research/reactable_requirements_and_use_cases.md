# Requisitos y casos de uso – Sistema tipo Reactable

Este documento recoge requisitos y escenarios de uso para una nueva aplicación/sistema tipo Reactable, moderna, multiplataforma y con prioridad en Linux.

## 1. Objetivos generales

- Ofrecer un **instrumento musical tangible/modular** inspirado en Reactable.
- Priorizar soporte robusto en **Linux** (primer ciudadano), manteniendo la posibilidad de extender a macOS/Windows.
- Diseñar una arquitectura **modular y extensible**, separando tracking, lógica musical, audio y UI.
- Facilitar experimentación en entornos de **investigación, performances en vivo e instalaciones interactivas**.

## 2. Tipos de usuario principales

1. **Músicos / performers**
   - Quieren crear y manipular patches de sonido de forma intuitiva en directo.
   - Necesitan fiabilidad, baja latencia y facilidad para guardar/cargar escenas.

2. **Artistas interactivos / escenógrafos**
   - Quieren integrar la mesa en instalaciones, mapping de proyección, etc.
   - Necesitan integración con otros sistemas (MIDI, OSC, redes, visuales).

3. **Investigadores / docentes**
   - Interesados en explorar interfaces tangibles, interacción hombre‑máquina, educación musical.
   - Valoran la apertura del sistema, documentación y capacidad de extenderlo.

4. **Desarrolladores / makers**
   - Quieren modificar el sistema, crear nuevos módulos, conectarlo a hardware propio.
   - Necesitan APIs claras y buena estructura de código.

## 3. Casos de uso clave

1. **Performance en directo con mesa física**
   - El usuario dispone de una mesa con cámara y proyector/pantalla.
   - Coloca objetos tangibles y manipula el sonido durante un concierto.
   - El sistema debe ser robusto a cambios de luz y soportar largas sesiones.

2. **Uso de escritorio sin hardware dedicado**
   - El usuario utiliza sólo una pantalla y ratón/trackpad o pantalla táctil.
   - Los "objetos" se representan como elementos gráficos manipulables.
   - Sirve para diseño de patches, pruebas rápidas y educación.

3. **Instalación interactiva en museo o evento**
   - La mesa está accesible al público general.
   - Debe existir un modo "kiosco" con interfaz simplificada y escenas preconfiguradas.

4. **Entorno de laboratorio / investigación**
   - El sistema se usa para validar nuevas técnicas de tracking o de interacción.
   - Se requieren logs detallados, APIs abiertas y herramientas de depuración.

## 4. Requisitos funcionales (alto nivel)

1. **Gestión de objetos/módulos**
   - Detección de objetos físicos (vía tracking) o virtuales (UI).
   - Asignación de un **módulo de audio/lógica** a cada tipo de objeto.
   - Posibilidad de crear, copiar, parametrizar y eliminar módulos.

2. **Construcción de patches**
   - Conexión automática o semiautomática entre módulos basándose en su disposición.
   - Visualización clara del grafo de señal.
   - Soporte para múltiples entradas/salidas por módulo.

3. **Motor de audio**
   - Síntesis básica (osciladores, filtros, envolventes).
   - Procesamiento de efectos (delay, reverb, distorsión, etc.).
   - Reproducción de samples / loops.
   - Sincronización a tempo interno y, opcionalmente, a reloj externo (MIDI clock, Ableton Link u otros).

4. **Control y parametrización**
   - Mapeo de posición/orientación a parámetros de módulo.
   - UI para ajustar parámetros finos (sliders, knobs virtuales, etc.).
   - Posibilidad de automatización simple (LFOs, secuenciadores internos).

5. **Persistencia**
   - Guardado de escenas/patches completos en archivos portables.
   - Exportación/importación de presets de módulos.

6. **Integración externa**
   - Entrada/salida de audio configurable.
   - Soporte de protocolos externos: **MIDI**, **OSC**, idealmente extensible.
   - Posible integración con DAWs (como efecto/instrumento virtual a largo plazo).

7. **Herramientas auxiliares**
   - Monitor de estado del sistema (uso CPU, latencia audio, FPS tracking).
   - Herramienta de calibración de cámara/superficie.
   - Modo demo/atractivo para mostrar el sistema sin intervención del usuario.

## 5. Requisitos no funcionales

1. **Rendimiento y latencia**
   - Latencia audio total (entrada-interacción-salida) percibida < 20 ms deseable.
   - Latencia visual (tracking → UI) muy baja para evitar sensación de retraso.

2. **Robustez y estabilidad**
   - Manejo de pérdida temporal de tracking sin glitches de audio catastróficos.
   - Recuperación automática si se reinicia la cámara o se pierde la conexión.

3. **Portabilidad**
   - Soporte prioritario para Linux (por ejemplo, Ubuntu/Debian, Fedora).
   - Abstracción de APIs de audio y vídeo para facilitar portar a otros sistemas.

4. **Extensibilidad**
   - Arquitectura de **plugins** o módulos cargables.
   - APIs documentadas para crear nuevos tipos de módulos y mappings.

5. **Usabilidad**
   - Interfaz comprensible sin manual extenso para acciones básicas.
   - Sistema de ayudas contextuales y ejemplos de escena.

6. **Mantenibilidad**
   - Código modular, con capas bien separadas.
   - Tests automatizados al menos en componentes críticos (audio y tracking).

## 6. Alcance inicial (MVP)

Para un primer prototipo funcional (MVP) se propone:

- Frontend de escritorio (sin hardware dedicado obligatorio).
- Un conjunto reducido de módulos: 2–3 generadores, 2–3 efectos, 1–2 controladores.
- Motor de audio estable con ruteo básico y sincronización de tempo interna.
- Soporte de escenas guardadas/cargadas.
- Tracking opcional: si no hay cámara configurada, usar modo totalmente virtual.

## 7. Roadmap conceptual

1. **Fase 1 – Prototipo software-only**
   - UI de mesa virtual.
   - Motor de audio interno.
   - Módulos básicos.

2. **Fase 2 – Integración de tracking físico**
   - Servicio de tracking (inspirado en reacTIVision).
   - Calibración básica y mapeo de marcadores a módulos.

3. **Fase 3 – Extensibilidad y protocolos externos**
   - Soporte inicial de MIDI/OSC.
   - Sistema de plugins de módulos.

4. **Fase 4 – Optimización y hardware dedicado**
   - Ajustes finos de rendimiento y latencia.
   - Integración con mesa física dedicada (cámara + proyector/pantalla).

Este documento servirá como referencia para priorizar funcionalidades y validar decisiones de diseño/arquitectura.