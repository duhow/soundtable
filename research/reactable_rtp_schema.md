# Esquema del formato Reactable `.rtp` y mapeo a Rectai

Este documento describe la estructura del archivo `.rtp` de Reactable (formato XML), tomando como ejemplo `Loopdemo.rtp`, y define cómo se mapeará a nuestro modelo actual (`rectai::Scene`, `AudioModule`, `ObjectInstance` y extensiones futuras).

El objetivo es **consumir directamente archivos `.rtp` originales**, sin modificar su estructura, y extraer de ellos toda la información necesaria para reconstruir un estado de la aplicación Rectai.

---

## 1. Estructura general del documento `.rtp`

Raíz del documento:

```xml
<reactablepatch>
  <background ... />
  <tangibles>
    <tangible ...> ... </tangible>
    ...
  </tangibles>
  <author name="..." />
  <patch name="..." />
</reactablepatch>
```

Elementos principales:

- `reactablepatch`: elemento raíz.
- `background`: configuración visual global de la mesa.
- `tangibles`: colección de objetos físicos/virtuales colocados sobre la mesa.
- `author`: metadatos de autoría.
- `patch`: nombre del patch.

En Rectai, inicialmente usaremos estos elementos para:

- Cargar metadatos (`author`, `patch`) como información de preset/escena.
- Tener un punto de extensión para un futuro `BackgroundSettings` (no forma parte de `Scene` de momento).
- Construir el contenido principal de `Scene` a partir de `<tangibles>`.

---

## 2. Elemento `<background>`

Ejemplo del archivo `Loopdemo.rtp`:

```xml
<background
    color="0,0.101961,0.501961"
    texture=""
    alpha="0.5"
    rotation="0"
    revolution="0"
    random="0" />
```

Atributos conocidos:

- `color`: tripleta de flotantes separados por comas (RGB normalizado, 0–1).
- `texture`: ruta o identificador de textura (cadena vacía en el ejemplo).
- `alpha`: opacidad global.
- `rotation`: rotación de fondo.
- `revolution`: parámetro adicional de animación/rotación.
- `random`: nivel de aleatoriedad (por ejemplo, ruido o jitter en el fondo).

### Mapeo a Rectai

A corto plazo, el `<background>` **no se mapea directamente** a `rectai::Scene` (que se centra en módulos/conexiones/objetos). Será la base para definir una futura estructura, por ejemplo:

- `struct BackgroundSettings { ... };`
- Posible integración en un contenedor de escena ampliado (por ejemplo, `ScenePresentation` o similar).

El parser `.rtp` debe, como mínimo, ser capaz de:

- Leer y almacenar estos valores en una estructura interna.
- Permitir que la capa de UI decida cómo representarlos.

---

## 3. Contenedor `<tangibles>` y elementos `<tangible>`

```xml
<tangibles>
  <tangible ...> ... </tangible>
  ...
</tangibles>
```

Cada `<tangible>` representa **una entidad física/virtual en la mesa Reactable**, que en Rectai se descompone conceptualmente en:

- Una **instancia física** (`rectai::ObjectInstance`): posición, ángulo, color, estado de dock/mute, etc.
- Un **módulo lógico de audio/control** (`AudioModule` derivado): tipo de módulo (oscilador, filtro, secuenciador, etc.), parámetros, envelopes, secuencias, loops, etc.

### 3.1. Atributos comunes de `<tangible>`

Todos los ejemplos de `Loopdemo.rtp` comparten un conjunto de atributos básicos:

- `type`: categoría del tangible. Valores observados:
  - `Output`, `Tonalizer`, `Volume`, `Tempo`, `Accelerometer`, `LFO`,
    `Sequencer`, `Filter`, `Delay`, `Modulator`, `WaveShaper`, `Input`,
    `Loop`, `Oscillator`, `Sampleplay`.
- `id`: identificador entero del tangible dentro del patch (positivo o negativo).
- `x`, `y`: posición sobre la mesa (coordenadas flotantes, frecuentemente en un rango alrededor de `[-1, +3]` aproximadamente). Actúan como posición normalizada/relativa.
- `angle`: ángulo en grados.
- `color`: color en formato ARGB hexadecimal de 8 dígitos (ej. `d5d5d5ff`).
- `docked`: `0` o `1`, indica si el tangible está “acoplado” a un borde o zona fija.
- `muted`: `0` o `1`, indica si el módulo asociado está silenciado.
- `point`: entero que parece indicar un rol funcional (entrada/salida o rol específico de patch).

Atributos adicionales frecuentes:

- `subtype`: variante dentro del `type` (ej. `lowpass`, `pingpong`, `feedback`, `ringmod`, `distort`, `loop`, `sine`, `saw`, `drum`, `synth`, `tonalizer`, `sequencer`, etc.).
- Atributos de parámetros específicos de audio/control, que dependen de `type` (detallados por tipo más abajo).

### 3.2. Mapeo general a Rectai

Para cada `<tangible>` se prevé el siguiente mapeo:

1. **Instancia física (`ObjectInstance`)**:
   - `tracking_id` ← valor de `id` (símbolo único por tangible dentro del patch).
   - `logical_id` ← identificador lógico de módulo. Podemos usar una convención como `"module_" + id` o un mapeo 1:1 entre `id` y un `std::string` estable.
   - `x` ← atributo `x` (convertido a coordenadas normalizadas que usemos internamente; por ahora podemos conservar el valor tal cual y delegar la normalización en la capa de UI si es necesario).
   - `y` ← atributo `y`.
   - `angle_radians` ← atributo `angle` convertido de grados a radianes.

   Atributos como `color`, `docked`, `muted`, `point` **no están presentes** en `ObjectInstance` hoy. Se contemplan dos opciones para el futuro:
   - Extender `ObjectInstance` para incluir estos campos de estado de presentación.
   - Guardarlos como parámetros del módulo o en una estructura de presentación independiente.

2. **Módulo lógico (`AudioModule` + derivados)**:
   - `type` / `subtype` → selección de clase concreta (`OscillatorModule`, `FilterModule`, `SequencerModule`, etc.) y configuración de `ModuleType`.
   - Todos los atributos de parámetros adicionales → se mapean a parámetros de módulo (`SetParameter(name, value)`), o a estructuras auxiliares (`Envelope`, `Sequence`, `LoopDefinition`, etc.).
   - Subelementos como `<envelope>`, `<sequence>`, `<loop>`, `<tone>`, `<instrument>`, `<hardlink>` → se traducen a estructuras C++ específicas.

Cada `tangible` generará, por tanto:

- 1 entrada en `Scene::objects()`.
- 1 entrada en `Scene::modules()` (probablemente con `id` basado en el `id` del tangible o en un nombre derivado).

---

## 4. Tipos de `<tangible>` y sus parámetros

A continuación se resumen los tipos de tangibles observados en `Loopdemo.rtp` y sus atributos específicos. Esta sección sirve como guía para definir las futuras clases de módulos y sus estructuras auxiliares.

### 4.1. `type="Output"`

Ejemplo:

```xml
<tangible type="Output" id="-1" x="0" y="0" angle="0"
          color="ffffffff" docked="0" muted="0" point="0" />
```

Atributos específicos:

- Ninguno aparte de los comunes.

Semántica:

- Representa la **salida global** del sistema Reactable.
- En Rectai, será el punto final de la cadena de audio.

Implicación para el modelo:

- Necesitaremos un módulo lógico `OutputModule`, aunque las conexiones explícitas no están codificadas en el `.rtp` mediante elementos `connection`, sino indirectamente (por ejemplo, con `<hardlink to="-1" />` en un `Delay`).

### 4.2. `type="Tonalizer"`

Ejemplo:

```xml
<tangible type="Tonalizer" id="0" x="2.60833" y="0.9" angle="0"
          color="d5d5d5ff" docked="1" muted="0" point="0"
          subtype="tonalizer" current_track="0">
    <tone key="0" scale="1,1,1,1,1,1,1,1,1,1,1,1" />
    ... (más <tone>) ...
</tangible>
```

Atributos adicionales:

- `subtype="tonalizer"`.
- `current_track`: entero (`0` en el ejemplo).

Subelementos:

- Múltiples `<tone>`:

  ```xml
  <tone key="0" scale="1,1,1,1,1,1,1,1,1,1,1,1" />
  ```

  - `key`: índice de tono/base.
  - `scale`: lista de 12 valores (enteros flotantes o enteros) separados por comas.

Implicación para el modelo:

- Necesitaremos una estructura `ToneDefinition` o similar:
  - `int key;`
  - `std::array<float, 12> scale;`
- Y un módulo `TonalizerModule` que almacene un conjunto de estas definiciones.

### 4.3. `type="Volume"`

Ejemplo:

```xml
<tangible type="Volume" id="1" ... subtype="volume"
          volume="90"
          compression_level="0" compression_on="0"
          reverb_level="0" reverb_input="0" reverb_on="0"
          delay_fb="0" delay_time="0.7" />
```

Parámetros adicionales:

- `volume`: nivel principal (probablemente [%] o dB relativo).
- `compression_level`, `compression_on`.
- `reverb_level`, `reverb_input`, `reverb_on`.
- `delay_fb`, `delay_time`.

Implicación para el modelo:

- `VolumeModule` con parámetros mapeados 1:1.
- Posibilidad de exponer estos parámetros como `SetParameter("volume", ...)`, etc., con tipos y rangos razonables.

### 4.4. `type="Tempo"`

Ejemplo:

```xml
<tangible type="Tempo" id="2" ... tempo="128" meter="4" swing="0" />
```

Parámetros:

- `tempo`: BPM.
- `meter`: compás (ej. 4 = 4/4).
- `swing`: grado de swing.

Implicación:

- Módulo global `TempoModule` que controla el reloj de la escena (especialmente relevante para secuenciadores y loops).

### 4.5. `type="Accelerometer"`

Ejemplo:

```xml
<tangible type="Accelerometer" id="3" ...
          amp_mult="1" freq_mult="1" freq="12" duration="0.75" />
```

Parámetros:

- `amp_mult`, `freq_mult`: multiplicadores para amplitud y frecuencia.
- `freq`: frecuencia base de modulación.
- `duration`: duración característica.

Implicación:

- `AccelerometerModule` que produce señales de control basadas en estos parámetros (y, en el futuro, en datos físicos de un sensor real).

### 4.6. `type="LFO"`

Ejemplo:

```xml
<tangible type="LFO" id="4" ... subtype="sine"
          freq="9" mult="0.906058" samplehold="1" />
```

Parámetros:

- `subtype`: forma de onda (`sine`, etc.).
- `freq`: frecuencia.
- `mult`: multiplicador.
- `samplehold`: flag o nivel de efecto sample & hold.

Implicación:

- `LfoModule` con forma de onda seleccionable y parámetros estándar.

### 4.7. `type="Sequencer"`

Ejemplos (docked y libre):

```xml
<tangible type="Sequencer" id="5" ... subtype="sequencer"
          current_track="0" autoseq_on="0" noteedit_on="0"
          duration="0.998737" num_tracks="6" offset="0">
    <sequence ... />
    ...
</tangible>

<tangible type="Sequencer" id="6" ...>
    <sequence ... />
    ...
</tangible>
```

Atributos del `<tangible>`:

- `subtype="sequencer"` (en uno de los casos).
- `current_track`, `autoseq_on`, `noteedit_on`.
- `duration`: duración del patrón.
- `num_tracks`: número de pistas.
- `offset`: desplazamiento temporal.

Subelementos `<sequence>` (uno por pista o patrón):

```xml
<sequence rows="1,1,1,1,1,1,1,1,1,1,1,1,1"
          speed="5" speed_type="binary"
          step_frequencies="0,0,0,0.166667,0,..."
          steps="1,0,0,1,0,0,0,1,0,1,0,0,1,0,0,0"
          tenori0="0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
          ...
          tenori12="0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
          volumes="1,1,1,0.233645,..." />
```

Atributos de `sequence`:

- `rows`: patrón de filas activas.
- `speed`, `speed_type` (p.ej. "binary").
- `step_frequencies`: frecuencia asociada a cada paso.
- `steps`: 0/1 por paso.
- `tenori0`..`tenori12`: patrones 0/1 por capa.
- `volumes`: nivel por paso.

Implicación:

- Necesitaremos una estructura `SequenceTrack` con:
  - Listas de pasos, frecuencias, volúmenes.
  - Vectores (o arrays) para tenori0..tenori12.
- Y un `SequencerModule` que agregue `num_tracks` secuencias.

### 4.8. `type="Filter"`

Ejemplos:

```xml
<tangible type="Filter" id="8" ... subtype="lowpass"
          freq="123.9" q="0">
    <envelope ... />
</tangible>

<tangible type="Filter" id="9" ... subtype="lowpass"
          freq="97.4083" q="0.811374">
    <envelope ... />
</tangible>
```

Parámetros:

- `subtype`: tipo de filtro (ej. `lowpass`).
- `freq`: frecuencia de corte.
- `q`: factor de calidad.

Subelementos:

- `<envelope>` (ver sección 5.1).

Implicación:

- Ya existe `FilterModule` en el código, pero habrá que ampliarlo para soportar estos parámetros y la envolvente.

### 4.9. `type="Delay"`

Ejemplos:

```xml
<tangible type="Delay" id="10" ... subtype="pingpong"
          delay="0.66" fb="0.5" sweep="0">
    <envelope ... />
    <hardlink to="-1" />
</tangible>

<tangible type="Delay" id="11" ... subtype="feedback"
          delay="0.66" fb="0.5" sweep="0">
    <envelope ... />
</tangible>
```

Parámetros:

- `subtype`: variante de delay (`pingpong`, `feedback`, etc.).
- `delay`: tiempo de retardo.
- `fb`: feedback.
- `sweep`: parámetro adicional de modulación de delay.

Subelementos:

- `<envelope>`.
- `<hardlink to="id" />`: conexión fija a otro tangible (por ejemplo, salida global `id=-1`).

Implicación:

- `DelayModule` con soporte para estas variantes y parámetros.
- Un mecanismo para traducir `<hardlink>` a **conexiones en nuestro modelo** (`rectai::Connection`) hacia módulos especiales como el de salida.

### 4.10. `type="Modulator"`

Ejemplo:

```xml
<tangible type="Modulator" id="12" ... subtype="ringmod"
          effect="0.5" drywet="0.5" depth="0.5" min="0.5" fb="0.5">
    <envelope ... />
</tangible>
```

Parámetros:

- `subtype`: tipo de modulación (`ringmod`, etc.).
- `effect`, `drywet`, `depth`, `min`, `fb`.

Implicación:

- `ModulatorModule` con estos parámetros, que actuaría sobre señales de audio/control.

### 4.11. `type="WaveShaper"`

Ejemplo:

```xml
<tangible type="WaveShaper" id="14" ... subtype="distort"
          effect="0.5" drywet="0.5">
    <envelope ... />
</tangible>
```

Parámetros:

- `subtype`: tipo de distorsión (`distort`, etc.).
- `effect`, `drywet`.

Implicación:

- `WaveShaperModule` asociado a distorsiones.

### 4.12. `type="Input"`

Ejemplo:

```xml
<tangible type="Input" id="20" ... amp="0">
    <envelope ... />
</tangible>
```

Parámetros:

- `amp`: ganancia de entrada.

Implicación:

- `InputModule` asociado a la entrada de audio externo.

### 4.13. `type="Loop"`

Ejemplos:

```xml
<tangible type="Loop" id="24" ... subtype="loop"
          amp="0.356205" sample="0" speed="1">
    <envelope ... />
    <loop beats="8" filename="Demoloops/pl_padloop1.wav" order="0" />
    ...
</tangible>

<tangible type="Loop" id="34" ...>
    <envelope ... />
    <loop beats="4" filename="Demoloops/pl_subendbreak1.wav" order="0" />
    ...
</tangible>

<tangible type="Loop" id="29" ...>
    <envelope ... />
    <loop beats="8" filename="" order="0" />
    ...
</tangible>
```

Parámetros del `tangible`:

- `subtype="loop"`.
- `amp`: nivel.
- `sample`: índice de sample.
- `speed`: velocidad de reproducción.

Subelementos `<loop>`:

- `beats`: número de beats.
- `filename`: ruta del archivo de audio.
- `order`: posición dentro de una secuencia de loops.

Implicación:

- `LoopModule` con una lista de `LoopDefinition`:
  - `beats`, `filename`, `order`.

### 4.14. `type="Oscillator"`

Ejemplo:

```xml
<tangible type="Oscillator" id="46" ... subtype="saw"
          amp="0" sweep="0" current_osc="1" second_tonalize="0"
          midifreq="44"
          second_amp1="0.869318" offset1="12" detune1="1.26263" wave1="1"
          second_amp2="0.70202" offset2="7" detune2="0.505049" wave2="1"
          bite="0">
    <envelope ... />
</tangible>
```

Parámetros:

- `subtype`: tipo de oscilador (`saw`, `sine`, etc.).
- `amp`, `sweep`, `bite`.
- `current_osc`, `second_tonalize`.
- `midifreq`: nota base en valor MIDI.
- `second_amp1`, `offset1`, `detune1`, `wave1`.
- `second_amp2`, `offset2`, `detune2`, `wave2`.

Subelementos:

- `<envelope>`.

Implicación:

- `OscillatorModule` ya existe, pero habrá que alinearlo con estos parámetros, al menos a nivel de nomenclatura y rangos.

### 4.15. `type="Sampleplay"`

Ejemplos:

```xml
<tangible type="Sampleplay" id="48" ... subtype="drum"
          amp="1" midifreq="57" filename="default.sf2">
    <instrument name="TR-101 Drumset" />
    <instrument name="Grungy Ramp Bass" />
</tangible>

<tangible type="Sampleplay" id="49" ... subtype="synth"
          amp="1" midifreq="57" filename="default.sf2">
    <instrument name="TR-101 Drumset" />
    <instrument name="Grungy Ramp Bass" />
</tangible>
```

Parámetros:

- `subtype`: rol principal (`drum`, `synth`).
- `amp`: nivel.
- `midifreq`: nota base.
- `filename`: archivo SF2.

Subelementos `<instrument>`:

- `name`: nombre del preset/instrumento.

Implicación:

- `SampleplayModule` con:
  - Sample o banco (SF2).
  - Lista de instrumentos disponibles.

---

## 5. Subelementos compartidos

### 5.1. `<envelope>`

Aparece en múltiples tipos de tangibles (`Filter`, `Delay`, `Modulator`, `WaveShaper`, `Input`, `Loop`, `Oscillator`, etc.). Ejemplo genérico:

```xml
<envelope attack="0" decay="0" duration="25"
          points_x="0,0,0,0.2,1"
          points_y="0,1,1,1,0"
          release="20" />
```

Atributos:

- `attack`, `decay`, `duration`, `release`: tiempos característicos.
- `points_x`: lista de puntos X normalizados (0–1).
- `points_y`: lista de puntos Y (nivel de la envolvente).

Implicación:

- Estructura común `Envelope` con:
  - `float attack, decay, duration, release;`
  - `std::vector<float> points_x, points_y;`

### 5.2. `<loop>`

Ya descrito en 4.13.

### 5.3. `<hardlink>`

Ejemplo:

```xml
<hardlink to="-1" />
```

Atributos:

- `to`: `id` de otro tangible.

Implicación:

- Debe traducirse a una conexión en nuestro modelo (`rectai::Connection`) entre el módulo del tangible actual y el módulo objetivo (`Output` u otros).

### 5.4. `<tone>`

Ya descrito en 4.2.

### 5.5. `<sequence>`

Ya descrito en 4.7.

### 5.6. `<instrument>`

Ya descrito en 4.15.

---

## 6. Metadatos finales: `<author>` y `<patch>`

Al final del documento `.rtp` encontramos:

```xml
<author name="Reactable" />
<patch name="Loopdemo" />
```

Atributos:

- `author/@name`: nombre del autor.
- `patch/@name`: nombre del patch.

Implicación:

- Se pueden mapear a una estructura de metadatos de escena, por ejemplo:

  ```cpp
  struct SceneMetadata {
    std::string author_name;
    std::string patch_name;
  };
  ```

- No forman parte de `rectai::Scene` hoy, pero se pueden mantener junto al `Scene` cargado.

---

## 7. Resumen de lo que falta en el modelo Rectai

A partir del análisis de `Loopdemo.rtp`, se identifican las siguientes necesidades para alinear el modelo de Rectai con el esquema Reactable:

1. **Nuevos tipos de módulos** (derivados de `AudioModule`):
   - `OutputModule`, `TonalizerModule`, `VolumeModule`, `TempoModule`, `AccelerometerModule`,
     `LfoModule`, `SequencerModule`, `DelayModule`, `ModulatorModule`,
     `WaveShaperModule`, `InputModule`, `LoopModule`, `SampleplayModule`.
   - Revisión y ampliación de `OscillatorModule` y `FilterModule` para soportar todos los parámetros observados.

2. **Estructuras auxiliares compartidas**:
   - `Envelope` (para `<envelope>`).
   - `LoopDefinition` (para `<loop>` en `Loop` tangibles).
   - `ToneDefinition` o similar (para `<tone>` en `Tonalizer`).
   - `SequenceTrack` / `SequencePattern` (para `<sequence>` en `Sequencer`).
   - `SampleInstrument` (para `<instrument>` en `Sampleplay`).

3. **Estado de presentación de objetos**:
   - Decidir si `ObjectInstance` se extiende para incluir:
     - `color`, `docked`, `muted`, `point`.
   - O si estos campos se gestionan como parámetros de módulo o en una estructura separada.

4. **Conexiones implícitas por `hardlink`**:
   - Diseñar una traducción de `<hardlink to="id">` a `rectai::Connection`.

5. **Metadatos globales y fondo**:
   - Definir estructuras para `BackgroundSettings` y `SceneMetadata`.

---

## 8. Futuras funciones de carga `.rtp`

Este documento sirve como base para implementar un loader de patches Reactable. A alto nivel, la futura API podría tener la forma:

```cpp
struct ReactablePatch {
  Scene scene;                 // Módulos, conexiones y objetos.
  // Metadata y presentación.
  // BackgroundSettings background;
  // SceneMetadata metadata;
};

bool LoadReactablePatchFromXml(const std::string& xml,
                               ReactablePatch& out_patch,
                               std::string* error_message);
```

Pasos esperados del parser:

1. Parsear el XML raíz `<reactablepatch>`.
2. Leer `<background>`, `<author>`, `<patch>` y guardar en estructuras auxiliares.
3. Recorrer `<tangibles>/<tangible>`:
   - Crear `ObjectInstance` con `id`, `x`, `y`, `angle` → radianes.
   - Crear/instanciar el módulo lógico adecuado según `type`/`subtype`.
   - Mapear atributos adicionales a parámetros del módulo.
   - Procesar subelementos específicos (`<envelope>`, `<sequence>`, `<loop>`, `<tone>`, `<instrument>`, `<hardlink>`).
4. Traducir `<hardlink>` a `Connection` en el modelo de `Scene`.

Con este esquema documentado, disponemos de la base necesaria para:

- Diseñar las nuevas clases de módulo.
- Implementar el parser de `.rtp`.
- Mantener una correspondencia clara con el formato Reactable original sin modificar su estructura.
