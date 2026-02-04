VERSION 0.8

# Metadatos de AppImage (pueden sobreescribirse vía --build-arg)
# APP_VERSION por defecto se toma de la variable de CMake SOUNDTABLE_VERSION_STRING
# cuando Earthly se ejecuta desde un árbol ya configurado con CMake. Si se construye
# desde cero, se puede sobreescribir manualmente.
ARG APP_VERSION=${SOUNDTABLE_VERSION_STRING:-dev}
ARG APPIMAGE_UPDATE_INFORMATION="gh-releases-zsync|duhow|soundtable|latest|Soundtable-*-x86_64.AppImage.zsync"

# Target base con toolchain y dependencias

base-system:
    FROM ubuntu:22.04

    RUN rm -f /etc/apt/apt.conf.d/docker-clean && \
        echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

    ENV DEBIAN_FRONTEND=noninteractive

    RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
        --mount=type=cache,target=/var/lib/apt,sharing=locked \
        apt-get update

the-base:
    FROM +base-system

    RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
        --mount=type=cache,target=/var/lib/apt,sharing=locked \
        apt-get update && \
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
            libopencv-dev

    WORKDIR /opt/soundtable

    COPY reacTIVision ./reacTIVision
    COPY JUCE ./JUCE
    COPY external ./external

    COPY CMakeLists.txt ./
    COPY resources ./resources
    COPY core ./core
    COPY tracker ./tracker
    COPY tests ./tests
    COPY research ./research

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
    FROM +base-system

    # Herramientas adicionales necesarias para crear AppImage (incluye ImageMagick para redimensionar el icono)
    RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
        --mount=type=cache,target=/var/lib/apt,sharing=locked \
        apt-get update && \
        apt-get install -y --no-install-recommends \
            curl \
            file \
            ca-certificates \
            libzip4 \
            libfluidsynth3 \
            libopencv-core4.5d \
            libopencv-highgui4.5d \
            libopencv-videoio4.5d \
            appstream

    # Descargamos linuxdeploy y appimagetool como AppImages
    RUN curl -L https://github.com/linuxdeploy/linuxdeploy/releases/download/continuous/linuxdeploy-x86_64.AppImage -o /usr/local/bin/linuxdeploy && \
        chmod +x /usr/local/bin/linuxdeploy && \
        curl -L https://github.com/AppImage/AppImageKit/releases/download/continuous/appimagetool-x86_64.AppImage -o /usr/local/bin/appimagetool && \
        chmod +x /usr/local/bin/appimagetool

    RUN mkdir -p \
        /AppDir/resources \
        /AppDir/usr/bin \
        /AppDir/usr/share/applications \
        /AppDir/usr/share/icons/hicolor/256x256/apps \
        /AppDir/usr/share/metainfo

    COPY --if-exists resources/reactableresources.zip /AppDir/resources/

    COPY resources/reactable-logo.png /AppDir/usr/share/icons/hicolor/256x256/apps/Soundtable.png

    # Copia/symlink en la raíz del AppDir para que appimagetool/linuxdeploy lo detecten como icono de AppImage
    RUN ln -sf usr/share/icons/hicolor/256x256/apps/Soundtable.png /AppDir/Soundtable.png && \
        ln -sf Soundtable.png /AppDir/.DirIcon

    COPY resources/app.desktop /AppDir/usr/share/applications/Soundtable.desktop
    #COPY resources/appstream.xml /AppDir/usr/share/metainfo/Soundtable.appdata.xml

    ENV APPIMAGE_EXTRACT_AND_RUN=1
    ENV ARCH=x86_64

    # appimagetool/AppImageUpdate
    ENV VERSION=${APP_VERSION}
    ENV UPDATE_INFORMATION=${APPIMAGE_UPDATE_INFORMATION}

    COPY +build/Soundtable /AppDir/usr/bin/Soundtable
    COPY +build/soundtable-tracker /AppDir/usr/bin/soundtable-tracker

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
