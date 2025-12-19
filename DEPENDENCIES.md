# Dependencias del proyecto rectai-table

Este documento lista las dependencias principales del proyecto y cómo instalarlas.

## 1. Dependencias generales de compilación

- **Compilador C++20** (g++ >= 11 o clang++ equivalente).
- **CMake** >= 3.20.
- **Git** (para clonar el repositorio y el submódulo JUCE).

### En Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake git libzip-dev
```

## 2. Dependencias de JUCE (core de la aplicación)

El proyecto espera que el código fuente de JUCE esté disponible en la carpeta `JUCE/` del repositorio, típicamente como **submódulo git**.

### 2.1. Obtener JUCE

Repositorio oficial de JUCE:

- URL: https://github.com/juce-framework/JUCE

Dentro del directorio raíz del proyecto:

```bash
# Añadir JUCE como submódulo
git submodule add https://github.com/juce-framework/JUCE.git JUCE

# Inicializar/actualizar submódulos (si ya estaba declarado)
git submodule update --init --recursive
```

### 2.2. Paquetes de sistema requeridos para JUCE en Linux

JUCE utiliza varias librerías del sistema para audio y UI.

En Ubuntu/Debian, se recomienda instalar:

```bash
sudo apt-get install -y \
  libasound2-dev \
  libjack-jackd2-dev \
  libx11-dev \
  libxcomposite-dev \
  libxcursor-dev \
  libxinerama-dev \
  libxrandr-dev \
  libxrender-dev \
  libfreetype6-dev \
  libgl1-mesa-dev \
  libssl-dev \
  libcurl4-openssl-dev \
  libgtk-3-dev \
  libzip-dev
```

En Windows, se utilizarán las dependencias que JUCE gestiona a través de su integración con Visual Studio / toolchain correspondiente.

## 3. Dependencias del servicio de tracking (OpenCV)

El servicio `rectai-tracker` depende de **OpenCV** para captura y procesado de vídeo.

### 3.1. Paquetes en Ubuntu/Debian

```bash
sudo apt-get install -y libopencv-dev
```

Esto instala OpenCV y los archivos de desarrollo necesarios (`pkg-config`, headers, etc.).

### 3.2. Instalación alternativa desde código fuente

Si se necesita una versión concreta de OpenCV, puede compilarse manualmente desde el repositorio oficial:

- OpenCV: https://github.com/opencv/opencv

(Para la mayoría de casos, el paquete del sistema es suficiente.)

## 4. Soporte de SoundFont2 (FluidSynth)

El módulo `Sampleplay` de la aplicación core utiliza **FluidSynth** para
cargar archivos SoundFont2 (`.sf2`) y enumerar los instrumentos/presets
disponibles en ellos. Esta integración se usa por ahora para construir la
lista de instrumentos mostrada en la UI a partir del propio archivo
SoundFont, en lugar de depender exclusivamente de la lista declarada en el
patch `.rtp`.

En Ubuntu/Debian, instala los headers y librerías de desarrollo con:

```bash
sudo apt-get install -y libfluidsynth-dev
```

El `CMakeLists.txt` de `core/` detecta FluidSynth vía `pkg-config`
(`pkg_check_modules(FLUIDSYNTH REQUIRED fluidsynth)`) y enlaza la
aplicación contra la librería del sistema.

## 4. Docker

El repositorio incluye un `Dockerfile` que instala todas las dependencias necesarias para compilar el proyecto dentro de un contenedor basado en Ubuntu 22.04.

### 4.1. Construir la imagen

Desde la raíz del proyecto:

```bash
docker build -t rectai-table-build .
```

### 4.2. Ejecutar un contenedor interactivo

```bash
docker run --rm -it rectai-table-build /bin/bash
```

Dentro del contenedor, los binarios resultantes (por ejemplo, `rectai-core` y `rectai-tracker`) estarán en el directorio `build/`.

## 5. Resumen rápido de instalación en Linux (host)

```bash
# Dependencias de compilación y librerías
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git libzip-dev \
  libasound2-dev libjack-jackd2-dev \
  libx11-dev libxcomposite-dev libxcursor-dev \
  libxinerama-dev libxrandr-dev libxrender-dev \
  libfreetype6-dev libgl1-mesa-dev \
  libssl-dev libcurl4-openssl-dev libgtk-3-dev \
  libopencv-dev libfluidsynth-dev

# Obtener JUCE como submódulo (si no se ha hecho)
git submodule add https://github.com/juce-framework/JUCE.git JUCE
git submodule update --init --recursive

# Configurar y compilar
mkdir -p build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --config Release -- -j"$(nproc)"
```