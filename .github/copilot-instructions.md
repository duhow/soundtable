# Guía para agentes de IA en `rectai-table`

Estas instrucciones están pensadas para GitHub Copilot y otros agentes de IA que colaboren en este repositorio.
El objetivo principal es mantenerse alineado con la arquitectura actual y los flujos de trabajo reales del proyecto.

## Visión general del proyecto

- Proyecto C++20 basado en CMake, orientado a recrear un sistema tipo Reactable.
- Usa **JUCE** para la aplicación principal (`core/`) y **OpenCV** para el servicio de tracking (`tracker/`).
- La documentación de diseño y arquitectura está en `research/` (ver especialmente `architecture_concepts.md` y `tech_stack.md`).

## Arquitectura y módulos clave

- `core/`: aplicación principal JUCE (`rectai-core`).
  - `core/src/Main.cpp`: punto de entrada JUCE (`RectaiApplication` + `MainWindow`).
  - `core/src/MainComponent.{h,cpp}`: componente principal de UI que pinta una "escena" de objetos/módulos.
  - `core/src/core/Scene.{h,cpp}`: modelo de dominio con:
    - `rectai::ObjectInstance`: objeto físico/virtual sobre la mesa (id de tracking, `logical_id`, posición normalizada, ángulo).
    - `rectai::AudioModule` + `rectai::ModuleType`: módulos lógicos (oscilador, filtro, etc.) con puertos, parámetros y metadata UI.
    - `rectai::Connection`: conexiones dirigidas entre puertos de módulos.
    - `rectai::Scene`: agrega módulos, conexiones y objetos con operaciones de alta coherencia (no duplicar ids, limpieza de conexiones al borrar un módulo, etc.).
- `tracker/`: binario `rectai-tracker` basado en OpenCV.
  - `tracker/src/main.cpp`: por ahora sólo abre la cámara por defecto, calcula FPS aproximados y escribe logs por stdout/stderr.
- `tests/`:
  - `tests/scene_tests.cpp`: ejecutable sencillo (no framework externo) que usa `assert` para validar el comportamiento de `Scene/Module/Connection/ObjectInstance` y se integra con **CTest**.

Al extender el modelo de dominio (nuevos tipos de módulos, propiedades de objetos, lógica de conexiones), toma como referencia `core/src/core/Scene.{h,cpp}` y actualiza también los tests en `tests/scene_tests.cpp`.

## Flujos de build y tests

- Build local estándar (fuera de contenedor):
  - Desde la raíz del repo:
    - `mkdir -p build`
    - `cd build`
    - `cmake -DCMAKE_BUILD_TYPE=Debug ..`
    - `cmake --build . --config Debug -- -j"$(nproc)"`
- Targets principales generados (según `CMakeLists.txt`):
  - `rectai-core`: app JUCE principal.
  - `rectai-tracker`: servicio de tracking (CLI).
  - Tests (si `RECTAI_BUILD_TESTS=ON`): ejecutables integrados con CTest.
- Ejecución de tests C++:
  - Desde `build/` tras configurar con CMake: `ctest --output-on-failure`.

Si haces cambios en `Scene`, `AudioModule` o `ObjectInstance`, asegúrate de que `tests/scene_tests.cpp` siga compilando y pasando con `ctest`.

## Convenciones de código C++

- Estándar: C++20, sin extensiones (`CMAKE_CXX_EXTENSIONS OFF`).
- Espacio de nombres de dominio: `rectai` (ver `core/src/core/Scene.h`).
- Estilo observado:
  - Clases y enums en `PascalCase` (`ObjectInstance`, `ModuleType`).
  - Métodos en `camelCase` (`AddModule`, `RemoveConnection`, `set_position`).
  - Uso de `[[nodiscard]]` en getters públicos y métodos que devuelven valores importantes.
  - Uso de `std::unique_ptr` para ownership en la parte JUCE (`MainWindow` en `Main.cpp`).
  - En JUCE se usan las macros estándar (`JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR`). No las elimines.
- Al modificar firmas o estructuras expuestas en `Scene.h`, mantén la coherencia entre declaración y definición en `Scene.cpp` y revisa todos los llamadores (por ejemplo `MainComponent.cpp` y `tests/scene_tests.cpp`).

## Interacción UI ↔ modelo de dominio

- `MainComponent` mantiene un miembro `rectai::Scene scene_;` y construye una escena de ejemplo en el constructor.
- El método `paint`:
  - Obtiene `scene_.objects()` y `scene_.connections()`.
  - Dibuja líneas entre objetos según las conexiones, usando posiciones **normalizadas** `(x, y)` sobre el tamaño actual del componente.
  - Dibuja cada objeto como un círculo con texto centrado con `logical_id`.
- Al introducir nueva lógica de UI que refleje cambios de `Scene` (por ejemplo, selección, resaltado, interacción), sigue este patrón: actualizar primero el modelo (`Scene`) y luego derivar el render desde el estado del modelo en `paint`/`resized`.

## Servicio de tracking

- `rectai-tracker` está pensado como servicio separado que leerá de cámara y, en el futuro, enviará información de tracking a la app principal.
- Por ahora, sólo usa OpenCV para abrir cámara y medir FPS.
- Al extenderlo:
  - Mantén la inicialización básica de cámara en `main`.
  - Extrae la lógica de procesamiento en funciones o clases separadas para poder testearlas y reutilizarlas.

## Expectativas para agentes de IA

- Toma de base la documentación en `research/` para entender el diseño y las decisiones arquitectónicas.
- La implementación que debes seguir, está en `research/tech_stack.md`.
- Después de cada cambio, actualiza el documento `research/progress.md` como referencia futura, para saber qué has implementado y cuáles son los siguientes pasos a realizar en el Roadmap definido.
- Los detalles relevantes tienen que estar escritos en el archivo de progreso, no devolverlos en el chat.
- La documentación será en español, pero el código y comentarios en inglés.
  - Si encuentras comentarios o código en español en el código, tradúcelos al inglés.
- Antes de introducir nuevas dependencias, revisa `DEPENDENCIES.md` y la integración con CMake.
- Si añades nuevos ejecutables o librerías:
  - Decláralos en los `CMakeLists.txt` correspondientes (`core/`, `tracker/`, `tests/` o raíz si aplica).
  - Usa las mismas opciones de compilación (C++20, sin extensiones) y respeta la estructura actual del proyecto.
- Cuando toques código en `core/src/core/Scene.*`, considera también:
  - Actualizar ejemplos en `MainComponent.cpp` si dejan de compilar o de tener sentido.
  - Añadir/ajustar asserts en `tests/scene_tests.cpp` para reflejar el nuevo comportamiento esperado.
- Mantén los mensajes de log del tracker en inglés y con prefijo `[rectai-tracker]` como en `tracker/src/main.cpp`.
- Los comandos de build como `cmake` no están disponibles en el host. Si tienes que ejecutar código de build o test en local, hazlo dentro de Podman con el comando `podman run --rm -it -v $PWD:$PWD:z -w $PWD build`, o usa Earthly.