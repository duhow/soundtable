# Dockerfile para compilar rectai-table (core JUCE + tracker OpenCV)

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libzip-dev \
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
        libfluidsynth-dev \
        libwebkit2gtk-4.1-dev \
        libgtk-3-dev \
        libopencv-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/rectai-table

#COPY CMakeLists.txt ./
#COPY core ./core
#COPY tracker ./tracker
#COPY research ./research

# JUCE: se espera que exista como submódulo JUCE/ en el repo host.
# Si no existe, puede clonarse dentro del contenedor manualmente.
#COPY JUCE ./JUCE

# RUN mkdir -p build && cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release .. && \
#     cmake --build . --config Release -- -j$(nproc)

CMD ["/bin/bash"]
