# Protocolo TUIO en rectai-table (borrador inicial)

## 1. Objetivo general

Este documento define cómo `rectai-tracker` y `rectai-core` integrarán el protocolo **TUIO** empezando por **TUIO 1.1**, manteniendo compatibilidad con el flujo actual basado en mensajes **OSC personalizados**, y preparando el terreno para soportar **TUIO 2.0** en el futuro.

El foco inmediato es:

- Seguir usando **OSC sobre UDP** como transporte.
- Introducir un pequeño mensaje de anuncio desde el tracker al core (`/tuio/hello`) y un **selector de modo explícito** en el tracker (`--osc`) en lugar de una negociación compleja con ACK.
- Definir cómo representar:
  - Fiduciales (marcadores físicos) con los mismos atributos que hoy o más.
  - Interacciones tipo "finger" (toques) para soportar múltiples toques simultáneos.
- Especificar el comportamiento de **fallback** cuando el core no responda a la negociación.

## 2. Contexto actual (versión mínima)

### 2.1 Transporte

- `rectai-tracker` ya envía datos al core mediante **OSC/UDP** local (detalles exactos a revisar en la siguiente fase del diseño).
- El core asume un conjunto de mensajes OSC concreto para actualizar la escena: posiciones, orientación y presencia de fiduciales.

### 2.2 Requisitos de compatibilidad

- El nuevo soporte TUIO **no debe romper** el flujo actual OSC:
  - Si el core no soporta TUIO 1.1, el tracker debe **seguir enviando los mensajes OSC actuales**.
  - Si la negociación falla o expira, el tracker vuelve automáticamente al modo OSC clásico y lo registra en los logs.

## 3. TUIO 1.1: resumen práctico

> Nota: esta sección se centra en los aspectos concretos que necesitamos en el proyecto; para los detalles formales se usará la especificación oficial TUIO 1.1.

- TUIO 1.x define perfiles sobre OSC para describir objetos tangibles, cursores (fingers) y blobs:
  - `tuio/2Dobj`: para objetos con id simbólico, posición 2D y ángulo.
  - `tuio/2Dcur`: para cursores tipo finger sin orientación.
  - (Opcional) `tuio/2Dblb` para blobs.
- Cada perfil usa típicamente tres tipos de mensajes:
  - `set`: estado de una entidad concreta (objeto, cursor, blob).
  - `alive`: lista de IDs actualmente presentes.
  - `fseq`: número de frame lógico (para sincronizar).

Para `rectai-table` nos interesan principalmente:

- `tuio/2Dobj` para mapear fiduciales existentes a objetos TUIO que instancian/controlan módulos en el modelo `Scene`.
- `tuio/2Dcur` para mapear detecciones de finger a cursores TUIO que el core usa como puntero principal (equivalente al ratón: down/move/up).

## 4. Negociación de versión TUIO 1.1

### 4.1 Motivación

Necesitamos saber si el core entiende TUIO 1.1 antes de enviar eventos con esa semántica. Usaremos el propio canal OSC para una negociación ligera.

### 4.2 Mensaje de saludo unidireccional

Implementación actual (simplificada respecto a la propuesta inicial con ACK):

- Al arrancar `rectai-tracker`, si el socket OSC hacia el core se ha creado correctamente y **no** se pasa `--osc`, el tracker entra en **modo TUIO 1.1** y envía un **mensaje de saludo** indicando la versión que propone:
  - Dirección OSC: `/tuio/hello`.
  - Argumentos:
    - `string` identificador del tracker (por ejemplo `"rectai-tracker"`).
    - `string` versión TUIO propuesta (por ejemplo `"1.1"`).
    - `string` versión interna del tracker (por ejemplo `"0.1.0"`).
- El core no envía ningún ACK; simplemente registra en los logs una línea del tipo:
  - `[rectai-core] TUIO hello from rectai-tracker (client 0.1.0, TUIO 1.1, via localhost:3333)`.

### 4.3 Selección de modo y fallback

- El modo de salida del tracker se elige así:
  - Si se pasa `--osc` en la línea de comandos, el tracker fuerza **modo OSC clásico** y sólo emite `/rectai/object` y `/rectai/remove`.
  - Si no se pasa `--osc` y el socket OSC está disponible, el tracker entra en **modo TUIO 1.1** y emite únicamente mensajes TUIO estándar (`/tuio/2Dobj`).
- No hay timeouts ni reintentos asociados a `/tuio/hello`: es un anuncio best-effort.
- Si el socket OSC no se puede abrir al arrancar, el tracker lo reporta en logs y el core simplemente no recibirá actualizaciones de tracking.

## 5. Mapeo de fiduciales a TUIO 1.1

### 5.1 Atributos mínimos necesarios

Queremos mantener al menos los atributos de fiducial que ya maneja el tracker actual, y aprovechar los campos estándar de TUIO donde tenga sentido.

A nivel conceptual, para cada fiducial tenemos (ya o en breve):

- `session_id`: identificador único de sesión (TUIO lo llama `s_id`).
- `symbol_id` o `class_id`: id simbólico del marcador (TUIO lo llama `i_id`).
- Posición normalizada en la mesa:
  - `x`, `y` en el rango [0,1] o coordenadas centradas que después se normalizan.
- Orientación:
  - `angle` en radianes o grados (TUIO usa `a` en radianes en [0,2π)).
- Velocidades y aceleraciones (opcional, TUIO provee campos `X`, `Y`, `A`, `m`, `r`).

### 5.2 Perfil `tuio/2Dobj`

En TUIO 1.1, un mensaje típico `set` para `tuio/2Dobj` tiene la forma:

- Dirección: `/tuio/2Dobj`.
- Tipo `set` con argumentos:
  - `s_id` (session id).
  - `i_id` (symbol id / fiducial id).
  - `x`, `y`, `a` (posición y ángulo normalizados).
  - `X`, `Y`, `A` (velocidades lineal y angular).
  - `m`, `r` (accel lineal y rotacional).

Para `rectai-tracker` se plantea:

- Usar como `s_id` (session id) el identificador de tracking de cada detección de fiducial.
- Mapear el **fiducial id físico** a un **id lógico de módulo** mediante una tabla interna y usar ese id lógico como `i_id` de TUIO. El core lo interpreta directamente como `logical_id` de `ObjectInstance`.
- Calcular `x`, `y`, `a` a partir de las coordenadas y orientación ya usadas para alimentar el modelo `Scene` en el core (posición normalizada y ángulo en radianes).
- Calcular `X`, `Y` y `A` como velocidades lineales y angular aproximadas a partir de la diferencia entre la última pose estable y la actual, dividiendo por el intervalo de tiempo entre frames. Los campos `m` y `r` se envían actualmente a cero.

## 6. Integración de fingers (2Dcur)

### 6.1 Objetivo

Además de los tangibles fiduciales, queremos soportar la detección de **fingers** en el tracker y exponerlos hacia el core usando el perfil TUIO adecuado. Esto permitirá que el core trate múltiples toques concurrentes de forma análoga a múltiples ratones.

### 6.2 Perfil `tuio/2Dcur`

En TUIO 1.1, los cursores 2D (fingers) se representan típicamente con:

- Dirección: `/tuio/2Dcur`.
- Mensajes `set` con:
  - `s_id` (session id único por dedo activo).
  - `x`, `y` (posición normalizada).
  - `X`, `Y` (velocidad).
  - `m` (aceleración).

Plan para `rectai-tracker` (alto nivel):

- Extender el pipeline de tracking para identificar blobs o puntos candidatos a finger.
- Asignar a cada finger detectado un `s_id` estable mientras el dedo permanezca en la mesa.
- Emitir mensajes TUIO `tuio/2Dcur` sólo cuando la negociación TUIO 1.1 haya tenido éxito.

### 6.3 Traducción a eventos de input en el core

En el core, estos cursores TUIO deberán convertirse, conceptualmente, en **múltiples eventos de puntero**:

- Cada `s_id` de finger se mapeará a un "puntero lógico" independiente.
- Las actualizaciones se traducirán en algo equivalente a eventos de ratón:
  - `finger down` ≈ `mouseDown`.
  - `finger move` ≈ `mouseDrag`/`mouseMove`.
  - `finger up` ≈ `mouseUp`.

Esta traducción requerirá adaptar el código actual del core, que está pensado en términos de **un solo puntero** (ratón), para admitir múltiples punteros concurrentes con id propio. Los detalles de esta adaptación se diseñarán en fases posteriores.

## 7. Comportamiento por fases

Para organizar el trabajo, planteamos una secuencia de fases:

1. **Fase 1 (este documento)**
  - Definición de alto nivel del soporte TUIO 1.1.
  - Decisión de mensajes de anuncio `/tuio/hello` y del modo de salida (TUIO vs OSC propietario).
  - Clarificación de objetivos de compatibilidad y fallback.
2. **Fase 2 (tracker)**
   - Implementar en `rectai-tracker`:
     - Envío del mensaje de saludo TUIO al arrancar.
     - Espera de ack con timeout de 2s y registro de fallback en logs.
     - Emisión de mensajes `tuio/2Dobj` a partir de los fiduciales actuales cuando TUIO 1.1 esté activo.
   - Mantener en paralelo el formato OSC actual mientras se estabiliza la transición (según se decida).
3. **Fase 3 (core)**
   - Implementar en `rectai-core`:
     - Receptor OSC que escuche `/tuio/hello` y registre la información de versión del tracker.
     - Receptor y parser de mensajes `tuio/2Dobj` (y, más adelante, `tuio/2Dcur`).
     - Mapeo de estos mensajes al modelo `Scene` y a la capa de input.
4. **Fase 4 (fingers + multi-touch)**
   - Extender el tracker para emitir `tuio/2Dcur`.
   - Adaptar el core para manejar múltiples punteros lógicos, integrando los eventos de finger en la lógica actual de interacción (arrastre de módulos, gestos de corte, etc.).
5. **Fase 5 (TUIO 2.0)**
   - Evaluar qué partes del diseño actual se pueden reutilizar para TUIO 2.0.
   - Introducir una segunda negociación de versión (p.ej. `"2.0"`) manteniendo compatibilidad con 1.1.

## 8. Pendiente de definir en próximas iteraciones

En futuros pasos de este documento se detallarán:

- Estructura exacta de los mensajes OSC de negociación (tipos, ejemplos concretos de payload).
- Política de reconexión y reintento de negociación cuando el core se reinicia.
- Detalles de cómo se compatibilizará el envío TUIO con el protocolo OSC actual (modo dual, flags de configuración, etc.).
- Mapeo preciso entre coordenadas internas del tracker y el rango normalizado TUIO [0,1].
- Diseño interno en el core para múltiples punteros lógicos y su interacción con la UI actual.
