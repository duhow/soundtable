# Reactable – Visión general y conceptos clave

Este documento recopila notas sobre el proyecto Reactable original, con el objetivo de entender sus principios de diseño, capacidades y limitaciones para inspirar una nueva implementación moderna, multiplataforma y orientada a Linux.

## 1. Contexto e historia breve

- **Reactable** es un instrumento musical electrónico basado en una **superficie tangible** interactiva.
- Fue desarrollado originalmente por el grupo de investigación **Music Technology Group (MTG)** de la Universitat Pompeu Fabra (Barcelona).
- Saltó a la fama hacia mediados/finales de los 2000 por su uso en actuaciones en directo, instalaciones interactivas y demostraciones de interfaces tangibles.
- Existen varias encarnaciones: la mesa física original, versiones de software (por ejemplo Reactable Mobile / apps) y sistemas personalizados para instalaciones.

## 2. Principios de interacción

- **Interfaz tangible**: el usuario manipula **objetos físicos** sobre una superficie (normalmente redonda) que representan módulos de síntesis o procesamiento de audio.
- **Marcadores fiduciales**: cada objeto tiene un marcador visual único (código) que el sistema reconoce mediante visión por computador.
- **Relaciones espaciales**: la posición, orientación y proximidad entre objetos definen la topología del patch de audio (quién se conecta con quién, intensidad, ruteo, etc.).
- **Feedback visual directo**: la superficie es una pantalla que proyecta visualizaciones alrededor de los objetos:
  - Conexiones (líneas, flujos) entre módulos.
  - Estados internos (niveles, espectros, parámetros).
  - Feedback en tiempo real a las acciones del usuario.
- **Performatividad**: está diseñado para conciertos en vivo, enfatizando la claridad visual de la estructura musical para el público.

## 3. Componentes conceptuales

Podemos descomponer Reactable en varios niveles:

1. **Interfaz física**
   - Superficie (mesa) retroproyectada o con pantalla integrada.
   - Cámara (habitualmente situada debajo) para capturar la posición de los marcadores.
   - Objetos tangibles con marcadores impresos.

2. **Capa de tracking**
   - Detección de marcadores fiduciales en tiempo real.
   - Cálculo de posición (x, y) y orientación (ángulo) de cada objeto.
   - Seguimiento de IDs persistentes.

3. **Capa de interacción y mapeo**
   - Asociación entre cada ID de marcador y un **módulo lógico** (oscilador, filtro, sample player, efectos, controladores, etc.).
   - Reglas de conexión en función de distancias / vecindades / orientaciones.
   - Mapeo de gestos (mover, rotar, añadir, quitar, tocar con el dedo) a cambios de parámetros.

4. **Motor de audio y lógica musical**
   - Motor de síntesis (frecuencias, envolventes, filtros, efectos, etc.).
   - Ruteo de señales entre módulos según el grafo definido por la disposición física.
   - Sincronización, tempo, patrones rítmicos, escalas, etc.

5. **Visualización y UI gráfica**
   - Renderizado de la mesa y los objetos sobre la superficie.
   - Visualización del flujo de señal, espectros, estados de los módulos.
   - Ayudas visuales para orientación del performer (valores numéricos, indicadores discretos).

## 4. Tipos de objetos / módulos habituales

Aunque existen distintas variantes, típicamente se incluyen:

- **Generadores**: osciladores, sintetizadores, samplers, drum machines, fuentes de ruido.
- **Procesadores**: filtros, delays, reverbs, distorsiones, compresores.
- **Moduladores / control**: LFOs, envolventes, secuenciadores, arpegiadores, generadores de patrones.
- **Ruteo**: mezcladores, splitters, sumadores, buses.
- **Interacción avanzada**: objetos que controlan escenas, presets, grabación de loops, etc.

Cada objeto suele tener una representación gráfica alrededor (círculos, símbolos) que indican su tipo y sus parámetros principales.

## 5. Comportamiento interactivo típico

- Colocar un objeto en la mesa **activa** el módulo correspondiente.
- Retirar el objeto **elimina** ese módulo y sus conexiones.
- Mover objetos más cerca o más lejos modifica:
  - Qué módulos se conectan.
  - El nivel de envío/mezcla.
- Rotar un objeto modifica uno o varios parámetros (por ejemplo, frecuencia de corte, ganancia, tempo relativo, etc.).
- Algunos sistemas permiten **gestos multi-touch** sobre la superficie para controlar parámetros continuos adicionales.

## 6. Características técnicas relevantes para nuestro rediseño

- **Tiempo real estricto**: latencia visual y auditiva muy baja (ideal <10–20 ms percibidos).
- **Robustez a condiciones de luz**: el tracking debe funcionar en entornos de escenario, con luces cambiantes.
- **Multiusuario**: más de una persona puede interactuar simultáneamente.
- **Configurabilidad**: posibilidad de definir nuevos tipos de objetos, presets, escenas, mappings.
- **Portabilidad del motor**: el motor de audio y la lógica deberían ser agnósticos de la plataforma de interfaz (mesa física, pantalla táctil, VR, etc.).

## 7. Limitaciones clásicas y oportunidades de mejora

- Dependencia de **hardware específico** (mesa, cámara, proyector) en las versiones originales.
- Curva de aprendizaje: aunque sea intuitivo visualmente, construir patches complejos puede ser difícil para usuarios noveles.
- Flexibilidad limitada en la definición de nuevos objetos por parte de usuarios finales (sin conocimiento técnico).
- Distribución compleja del sistema completo (drivers, dependencias, configuración de cámara, calibración).

Oportunidades para una versión moderna multiplataforma:

- Aprovechar **frameworks de audio modernos** (por ejemplo JACK/PipeWire en Linux, CoreAudio, WASAPI) de forma unificada.
- Integrar control por **MIDI**, **OSC** y protocolos de red para interoperar con otros DAWs e instrumentos.
- Posibilitar distintos frontends: mesa tangible, app de escritorio, app tablet, web.
- Definir un formato abierto para describir **módulos, conexiones y layouts**.

## 8. Ideas clave a conservar en nuestro diseño

- La representación **visual directa** del grafo de audio.
- El uso de **tangibles** (cuando el hardware lo permita) como metáfora principal.
- La modularidad extrema: cualquier objeto debería ser intercambiable y combinable.
- Un modelo de **estado reproducible**: la mesa completa debería poder guardarse y restaurarse como preset/escena.

Estas notas servirán como referencia para alinear el diseño de nuestra aplicación con la filosofía Reactable sin copiarla literalmente, priorizando una implementación moderna, abierta y bien documentada.