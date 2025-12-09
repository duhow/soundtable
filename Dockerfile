# Dockerfile para compilar rectai-table (core JUCE + tracker OpenCV)

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
        libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/rectai-table

# Copiamos sólo los archivos necesarios para configurar y compilar.
COPY CMakeLists.txt ./
COPY core ./core
COPY tracker ./tracker
COPY research ./research

# JUCE: se espera que exista como submódulo JUCE/ en el repo host.
# Si no existe, puede clonarse dentro del contenedor manualmente.
COPY JUCE ./JUCE

RUN mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build . --config Release -- -j$(nproc)

CMD ["/bin/bash"]
