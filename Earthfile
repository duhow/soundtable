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

    WORKDIR /opt/soundtable

    # Copiamos código del proyecto
    COPY CMakeLists.txt ./
    COPY core ./core
    COPY tracker ./tracker
    COPY tests ./tests
    COPY resources ./resources
    COPY reacTIVision ./reacTIVision
    COPY research ./research
    COPY external ./external

    # JUCE: se espera que exista como submódulo JUCE/ en el repo host.
    # Si no existe, puede clonarse dentro del entorno Earthly manualmente.
    COPY JUCE ./JUCE


# Target principal: compila soundtable-core (JUCE) y soundtable-tracker (OpenCV)

build:
    FROM +the-base

    RUN mkdir -p build && cd build && \
        cmake -DCMAKE_BUILD_TYPE=Release .. && \
        cmake --build . --config Release -- -j$(nproc)

    # Exportamos los binarios al host como artefactos locales.
    SAVE ARTIFACT build/core/soundtable-core_artefacts/Release/Soundtable AS LOCAL build/Soundtable
    SAVE ARTIFACT build/soundtable-tracker AS LOCAL build/soundtable-tracker


# Target AppImage: genera un AppImage con todas las dependencias necesarias

appimage:
    FROM +build

    # Herramientas adicionales necesarias para crear AppImage (incluye ImageMagick para redimensionar el icono)
    RUN apt-get update && \
        apt-get install -y --no-install-recommends \
            curl \
            file \
            ca-certificates \
            imagemagick && \
        rm -rf /var/lib/apt/lists/*

    # Preparamos AppDir con los binarios principales e iconos en ruta hicolor
    RUN mkdir -p /AppDir/usr/bin /AppDir/usr/share/applications /AppDir/usr/share/icons/hicolor/256x256/apps
    # Icono principal: se ajusta a 256x256 para cumplir con el tamaño del tema hicolor
    COPY resources/reactable-logo.png /AppDir/usr/share/icons/hicolor/256x256/apps/Soundtable.png
    RUN convert /AppDir/usr/share/icons/hicolor/256x256/apps/Soundtable.png \
        -resize 256x256^ -gravity center -extent 256x256 \
        /AppDir/usr/share/icons/hicolor/256x256/apps/Soundtable.png
    # Copia/symlink en la raíz del AppDir para que appimagetool/linuxdeploy lo detecten como icono de AppImage
    RUN ln -sf usr/share/icons/hicolor/256x256/apps/Soundtable.png /AppDir/Soundtable.png && \
        ln -sf Soundtable.png /AppDir/.DirIcon

    RUN cp build/core/soundtable-core_artefacts/Release/Soundtable /AppDir/usr/bin/Soundtable && \
        cp build/soundtable-tracker /AppDir/usr/bin/soundtable-tracker

    # Archivo .desktop mínimo para el lanzador principal (sin heredoc para evitar problemas de indentación)
    RUN printf '%s\n' \
        '[Desktop Entry]' \
        'Type=Application' \
        'Name=Soundtable' \
        'Exec=Soundtable' \
        'Icon=Soundtable' \
        'StartupWMClass=Soundtable' \
        'Categories=AudioVideo;Audio;' \
        > /AppDir/usr/share/applications/Soundtable.desktop

    # Descargamos linuxdeploy y appimagetool como AppImages
    RUN curl -L https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage -o /usr/local/bin/linuxdeploy && \
        chmod +x /usr/local/bin/linuxdeploy && \
        curl -L https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage -o /usr/local/bin/appimagetool && \
        chmod +x /usr/local/bin/appimagetool

    ENV APPIMAGE_EXTRACT_AND_RUN=1
    ENV ARCH=x86_64

    # linuxdeploy analiza los binarios, registra el .desktop y el icono, y copia dependencias compartidas
    RUN /usr/local/bin/linuxdeploy \
        --appdir /AppDir \
        --executable /AppDir/usr/bin/Soundtable \
        --executable /AppDir/usr/bin/soundtable-tracker \
        --desktop-file /AppDir/usr/share/applications/Soundtable.desktop \
        --icon-file /AppDir/usr/share/icons/hicolor/256x256/apps/Soundtable.png

    # Empaquetamos el AppDir en un AppImage con nombre fijo
    RUN /usr/local/bin/appimagetool /AppDir Soundtable-x86_64.AppImage

    # Exportamos el AppImage como artefacto local
    SAVE ARTIFACT Soundtable-x86_64.AppImage AS LOCAL build/Soundtable-x86_64.AppImage


# Target de tests: construye y ejecuta CTest

test:
    FROM +the-base

    RUN mkdir -p build && cd build && \
        cmake -DCMAKE_BUILD_TYPE=Debug -DSOUNDTABLE_BUILD_TESTS=ON .. && \
        cmake --build . --config Debug -- -j$(nproc) && \
        ctest --output-on-failure
