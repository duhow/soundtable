VERSION 0.8

# Target base con toolchain y dependencias

the-base:
    FROM ubuntu:22.04

    ENV DEBIAN_FRONTEND=noninteractive

    RUN apt-get update && \
        apt-get install -y --no-install-recommends \
            build-essential \
            cmake \
            git \
            pkg-config \
            libasound2-dev \
            libjack-jackd2-dev \
            ladspa-sdk \
            libx11-dev \
            libxcomposite-dev \
            libxcursor-dev \
            libxext-dev \
            libxinerama-dev \
            libxrandr-dev \
            libxrender-dev \
            libfontconfig1-dev \
            libfreetype6-dev \
            libglu1-mesa-dev \
            mesa-common-dev \
            libgl1-mesa-dev \
            libssl-dev \
            libcurl4-openssl-dev \
            libwebkit2gtk-4.1-dev \
            libgtk-3-dev \
            libopencv-dev && \
        rm -rf /var/lib/apt/lists/*

    WORKDIR /opt/rectai-table

    # Copiamos código del proyecto
    COPY CMakeLists.txt ./
    COPY core ./core
    COPY tracker ./tracker
    COPY tests ./tests
    COPY research ./research

    # JUCE: se espera que exista como submódulo JUCE/ en el repo host.
    # Si no existe, puede clonarse dentro del entorno Earthly manualmente.
    COPY JUCE ./JUCE


# Target principal: compila rectai-core (JUCE) y rectai-tracker (OpenCV)

build:
    FROM +the-base

    RUN mkdir -p build && cd build && \
        cmake -DCMAKE_BUILD_TYPE=Release .. && \
        cmake --build . --config Release -- -j$(nproc)

    # Exportamos los binarios al host como artefactos locales.
    SAVE ARTIFACT build/core/rectai-core_artefacts/Release/RectaiTable AS LOCAL build/RectaiTable
    SAVE ARTIFACT build/rectai-tracker AS LOCAL build/rectai-tracker


# Target de tests: construye y ejecuta CTest

test:
    FROM +the-base

    RUN mkdir -p build && cd build && \
        cmake -DCMAKE_BUILD_TYPE=Debug -DRECTAI_BUILD_TESTS=ON .. && \
        cmake --build . --config Debug -- -j$(nproc) && \
        ctest --output-on-failure
