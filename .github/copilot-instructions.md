# Guía para agentes de IA en `soundtable`

Estas instrucciones son para GitHub Copilot y otros agentes de IA que trabajen en este repo. El objetivo es respetar la arquitectura real y los flujos de trabajo existentes.

## Visión general y arquitectura

- Proyecto C++20 basado en CMake, recreando un sistema tipo Reactable.
- `core/`: app principal JUCE (`soundtable-core`).
  - `core/src/Main.cpp`: entrada JUCE (`SoundtableApplication`, `MainWindow`).
  - `core/src/MainComponent.{h,cpp}`: UI principal; mantiene `soundtable::Scene scene_` y pinta la mesa desde el modelo.
  - `core/src/core/Scene.{h,cpp}`: dominio (`ObjectInstance`, `AudioModule`/`ModuleType`, `Connection`, `Scene`). Cambios aquí suelen requerir actualizar `tests/scene_tests.cpp` y llamadas en `MainComponent`.
- `tracker/`: binario `soundtable-tracker` basado en OpenCV.
  - `tracker/src/main.cpp`: bucle principal de cámara + envío OSC/TUIO; logs con prefijo `[soundtable-tracker]`.
  - `tracker/src/OscSender.{h,cpp}` + `core/src/TrackingOscReceiver.{h,cpp}`: contrato de mensajes OSC/TUIO entre tracker y core.
- Documentación de diseño: `research/architecture_concepts.md`, `research/tech_stack.md` y resto de `research/`.

## Flujos de build, tests y ejecución

- Build local estándar:
  - Desde raíz: `mkdir -p build && cd build && cmake -DCMAKE_BUILD_TYPE=Debug .. && cmake --build . --config Debug -- -j"$(nproc)"`.
  - Targets principales: `soundtable-core`, `soundtable-tracker` y tests (si `SOUNDTABLE_BUILD_TESTS=ON`).
- Tests C++: desde `build/`, ejecutar `ctest --output-on-failure` (incluye tests de dominio en `tests/scene_tests.cpp` y de tracker en `tests/tracker_tests.cpp`).
- Build reproducible: ver `README.md` para `earthly +build` y uso de `Dockerfile` si no quieres instalar todas las dependencias en el host.
- Al cambiar `Scene`, `AudioModule`, `ObjectInstance` o tracking/OSC, recompila con `cmake` y ejecuta `ctest` para validar.

## Patrones de dominio y UI

- Coordenadas de objetos: `ObjectInstance` usa posición normalizada en la mesa; `MainComponent::paint` las proyecta al tamaño actual del componente.
- UI siempre deriva del modelo:
  - Modifica primero `soundtable::Scene` (módulos, conexiones, objetos) y deja que la UI lea `scene_.objects()` / `scene_.connections()` para pintar.
  - Evita estado duplicado de escena dentro de componentes JUCE; si necesitas nuevo estado persistente, considera extender `Scene`.
  - Prioriza definir la lógica en `AudioModules` siempre que sea posible y/o factible, para centralizar el comportamiento de forma genérica, ya que múltiples modelos se comportarán igual, y ayuda a reducir duplicado de código.
- El core espera tracking externo vía OSC/TUIO:
  - `soundtable-tracker` emite TUIO 1.1 o `/soundtable/*`; `TrackingOscReceiver` traduce esto a `ObjectInstance` y eventos de puntero.
  - Si cambias el formato de mensajes, mantén ambos lados sincronizados y actualiza los tests de tracker.

## Convenciones de C++ y dependencias

- C++20 sin extensiones (`CMAKE_CXX_EXTENSIONS OFF`), espacio de nombres `soundtable`.
- Estilo observado: clases/enums en `PascalCase`, métodos en `camelCase`, `[[nodiscard]]` en getters importantes, `std::unique_ptr` para ownership en JUCE, macros JUCE estándar (`JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR`).
- Al cambiar firmas públicas en `Scene.h` o módulos de audio, mantén en sync `Scene.cpp`, `MainComponent` y tests.
- Antes de añadir dependencias nuevas, revisa `DEPENDENCIES.md` y la integración con CMake (raíz, `core/CMakeLists.txt`, `tracker/CMakeLists.txt`).

## Expectativas específicas para agentes de IA

- Usa primero la documentación de `research/` para entender decisiones de arquitectura antes de introducir diseños nuevos.
- La implementación a seguir está descrita en `research/tech_stack.md`; evita reintroducir stacks alternativos descartados ahí.
- Ante la duda sobre cómo implementar ciertos elementos, el objetivo es recrear el comportamiento de un sistema Reactable, pero se pueden añadir funcionalidades avanzadas o experimentales con aprobación del usuario.
- Tras cada cambio de código o de CMake, crea o edita el archivo con fecha de hoy en formato `YYYY-MM-DD`, añade una entrada breve en `research/progress/YYYY-MM-DD.md` (en español) explicando qué se ha implementado y, si aplica, próximos pasos; no dupliques esa explicación en el chat.
- La documentación y comentarios de docs van en español; el código y comentarios en código deben estar en inglés. Si encuentras comentarios en español en código, tradúcelos al inglés al tocarlos.
- Si creas nuevos ejecutables o librerías, decláralos en los `CMakeLists.txt` correspondientes y sigue el patrón de opciones actual (C++20, sin extensiones, integración con tests/CTest si aplica).
