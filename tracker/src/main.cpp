#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

// Esqueleto mínimo del servicio de tracking.
// De momento sólo abre la cámara por defecto y muestra FPS aproximados.

int main()
{
    cv::VideoCapture capture(0);
    if (!capture.isOpened()) {
        std::cerr << "[rectai-tracker] No se pudo abrir la cámara por defecto." << std::endl;
        return 1;
    }

    std::cout << "[rectai-tracker] Cámara abierta. Pulsar Ctrl+C para salir." << std::endl;

    cv::Mat frame;
    const auto start_time = static_cast<double>(cv::getTickCount());
    int frames = 0;

    while (true) {
        if (!capture.read(frame) || frame.empty()) {
            std::cerr << "[rectai-tracker] Error al capturar frame." << std::endl;
            break;
        }

        ++frames;
        const double now = static_cast<double>(cv::getTickCount());
        const double elapsed_sec = (now - start_time) / cv::getTickFrequency();
        if (elapsed_sec >= 1.0) {
            const double fps = static_cast<double>(frames) / elapsed_sec;
            std::cout << "[rectai-tracker] FPS aproximados: " << fps << '\n';
            break;  // En el esqueleto salimos tras una medición.
        }
    }

    return 0;
}
