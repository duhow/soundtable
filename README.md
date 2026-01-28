# soundtable

Proyecto de recreación de un sistema tipo Reactable, con prioridad en Linux y soporte multiplataforma.

## Estructura del proyecto

- `core/`: aplicación principal basada en JUCE (core musical + UI de mesa virtual).
- `tracker/`: servicio de tracking basado en OpenCV (captura de cámara y, en el futuro, detección de marcadores).
- `research/`: documentación de investigación, diseño y arquitectura.
- `CMakeLists.txt`: configuración de CMake a nivel raíz.
- `Dockerfile`: entorno de build basado en Ubuntu 22.04.
- `DEPENDENCIES.md`: lista de dependencias y cómo instalarlas.

## Requisitos de sistema (resumen rápido)

Consulta `DEPENDENCIES.md` para ver la lista detallada.

En Ubuntu/Debian típicamente necesitarás:

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git \
  libasound2-dev libjack-jackd2-dev \
  libx11-dev libxcomposite-dev libxcursor-dev \
  libxinerama-dev libxrandr-dev libxrender-dev \
  libfreetype6-dev libgl1-mesa-dev \
  libssl-dev libcurl4-openssl-dev libgtk-3-dev \
  libopencv-dev
```

Y añadir JUCE como submódulo:

```bash
git submodule add https://github.com/juce-framework/JUCE.git JUCE
git submodule update --init --recursive
```

## Compilación en el host

```bash
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
cmake --build . --config Debug -- -j"$(nproc)"
```

Esto generará, entre otros:

- `soundtable-core`: aplicación principal JUCE.
- `soundtable-tracker`: servicio de tracking (CLI).

## Compilación con Earthly (recomendada para builds reproducibles)

Para evitar instalar todas las dependencias en tu sistema host, puedes usar **Earthly**.

1. Instala Earthly siguiendo la guía oficial: https://earthly.dev/get-earthly

2. Desde la raíz del proyecto, ejecuta:

```bash
earthly +build
```

Esto descargará una imagen base de Ubuntu, instalará todas las dependencias de compilación y generará los binarios:

- `build/soundtable-core`
- `build/soundtable-tracker`

## Compilación con Docker

```bash
docker build -t soundtable-build .
docker run --rm -it soundtable-build /bin/bash
```

Dentro del contenedor, los artefactos de build se encuentran en `build/`.

## Notas de diseño

Para detalles sobre motivación, arquitectura y decisiones tecnológicas, revisa los documentos en `research/`:

- `reactable_overview.md`
- `reactivision_technical_notes.md`
- `reactable_requirements_and_use_cases.md`
- `architecture_concepts.md`
- `tech_stack.md`
