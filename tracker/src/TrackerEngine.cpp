// Amoeba fiducial tracker.
//
// First, try to detect real Reactable "amoeba" fiducials using the
// original libfidtrack segmentation and tree-based decoder from the
// local reacTIVision sources. When no valid fiducials are found in a
// frame, fall back to a simple blob-based tracker that assigns stable
// numeric IDs based on spatial proximity, so tests and synthetic
// scenarios keep working even without printed markers.
#include "TrackerEngine.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace soundtable::tracker {

namespace {

constexpr int kMaxFiducials = 32;
constexpr int kMinRecognisedMarkerId = 0;
constexpr int kMaxRecognisedMarkerId = 215;

// Upper bound on the resolution used by the amoeba segmenter.
// The camera may run at a higher resolution, but we downscale
// frames before feeding them into libfidtrack. This keeps the
// amount of work in segment.c roughly proportional to this
// fixed area instead of the native sensor resolution.
constexpr int kMaxProcessingWidth = 1280;
constexpr int kMaxProcessingHeight = 720;

// Number of frames over which we evaluate all filters for a given
// fiducial before choosing the "winning" filter.
constexpr int kTrainingWindowFrames = 30;

// Human-readable names for debug logging of per-filter statistics.
constexpr int kNumThresholdFiltersConst = 4;
constexpr const char* kFilterNames[kNumThresholdFiltersConst] = {
    "OtsuBinary",
    "OtsuBinaryInv",
    "AdaptiveBinary",
    "AdaptiveBinaryInv"};

} // namespace

TrackerEngine::TrackerEngine() = default;

TrackerEngine::~TrackerEngine()
{
    if (amoebaInitialised_) {
        terminate_segmenter(&segmenter_);
        terminate_fidtrackerX(&fidtrackerX_);
        terminate_treeidmap(&treeIdMap_);
        amoebaInitialised_ = false;
    }
}

bool TrackerEngine::initialise(const int /*cameraIndex*/,
                  const int requestedWidth,
                  const int requestedHeight,
                  std::string& /*errorMessage*/,
                  const bool enableDownscale)
{
    frameWidth_ = requestedWidth;
    frameHeight_ = requestedHeight;
    lastObjects_.clear();

    if (frameWidth_ <= 0 || frameHeight_ <= 0) {
        initialised_ = false;
        return false;
    }

    // Clamp the internal processing resolution to a reasonable
    // upper bound. This reduces the amount of work done by the
    // segmenter (build_regions / merge_regions, etc.) when the
    // camera runs at HD or higher resolutions.
    if (enableDownscale &&
        (frameWidth_ > kMaxProcessingWidth || frameHeight_ > kMaxProcessingHeight)) {
        const float scaleX = static_cast<float>(kMaxProcessingWidth) / static_cast<float>(frameWidth_);
        const float scaleY = static_cast<float>(kMaxProcessingHeight) / static_cast<float>(frameHeight_);
        const float scale = std::min(scaleX, scaleY);

        frameWidth_ = std::max(1, static_cast<int>(std::round(frameWidth_ * scale)));
        frameHeight_ = std::max(1, static_cast<int>(std::round(frameHeight_ * scale)));
    }

    // Initialise amoeba fiducial tracking using the default tree
    // set bundled with reacTIVision ("default" amoeba markers).
    initialize_treeidmap(&treeIdMap_, "default");
    initialize_segmenter(&segmenter_, frameWidth_, frameHeight_, treeIdMap_.max_adjacencies);
    initialize_fidtrackerX(&fidtrackerX_, &treeIdMap_, nullptr);

    amoebaInitialised_ = true;
    initialised_ = true;
    return initialised_;
}

void TrackerEngine::setOnlySingleFilterByIndex(const bool enabled, const int filterIdx) noexcept
{
    onlySingleFilter_ = enabled;

    switch (filterIdx) {
    case 0:
        activeFilter_ = ThresholdFilter::OtsuBinary;
        break;
    case 1:
        activeFilter_ = ThresholdFilter::OtsuBinaryInv;
        break;
    case 2:
        activeFilter_ = ThresholdFilter::AdaptiveBinary;
        break;
    case 3:
        activeFilter_ = ThresholdFilter::AdaptiveBinaryInv;
        break;
    default:
        break;
    }
}

bool TrackerEngine::isInitialised() const noexcept
{
    return initialised_;
}

TrackedObjectList TrackerEngine::processFrame(const cv::Mat& frame)
{
    return processFrameInternal(frame, nullptr);
}

TrackedObjectList TrackerEngine::processFrame(const cv::Mat& frame, cv::Mat& debugFrame)
{
    return processFrameInternal(frame, &debugFrame);
}

TrackedObjectList TrackerEngine::processFrameInternal(const cv::Mat& frame, cv::Mat* debugFrame)
{
    TrackedObjectList result;

    if (!initialised_ || !amoebaInitialised_) {
        return result;
    }

    if (frame.empty() || frame.cols <= 0 || frame.rows <= 0) {
        return result;
    }

    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }

    cv::Mat resized;
    if (gray.cols != frameWidth_ || gray.rows != frameHeight_) {
        cv::resize(gray, resized, cv::Size(frameWidth_, frameHeight_));
    } else {
        resized = gray;
    }

    cv::Mat binary;
    auto detectWithBinary = [&](const cv::Mat& bin) -> TrackedObjectList {
        if (debugFrame != nullptr) {
            bin.copyTo(*debugFrame);
        }
        return detectAmoebaFiducials(bin);
    };

    auto runFilter = [&](ThresholdFilter filter) -> TrackedObjectList {
        switch (filter) {
        case ThresholdFilter::OtsuBinary:
            cv::threshold(resized, binary, 0.0, 255.0, cv::THRESH_BINARY | cv::THRESH_OTSU);
            return detectWithBinary(binary);
        case ThresholdFilter::OtsuBinaryInv:
            cv::threshold(resized, binary, 0.0, 255.0, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
            return detectWithBinary(binary);
        case ThresholdFilter::AdaptiveBinary:
            cv::adaptiveThreshold(resized, binary, 255.0, cv::ADAPTIVE_THRESH_MEAN_C,
                                  cv::THRESH_BINARY, 21, 5.0);
            return detectWithBinary(binary);
        case ThresholdFilter::AdaptiveBinaryInv:
            cv::adaptiveThreshold(resized, binary, 255.0, cv::ADAPTIVE_THRESH_MEAN_C,
                                  cv::THRESH_BINARY_INV, 21, 5.0);
            return detectWithBinary(binary);
        }

        return {};
    };

    const ThresholdFilter allFilters[kNumThresholdFilters] = {
        ThresholdFilter::OtsuBinary,
        ThresholdFilter::OtsuBinaryInv,
        ThresholdFilter::AdaptiveBinary,
        ThresholdFilter::AdaptiveBinaryInv};

    auto mergeResults = [](const TrackedObjectList& src, TrackedObjectList& dst) {
        for (const auto& obj : src) {
            const auto it = std::find_if(dst.begin(), dst.end(), [&](const TrackedObject& existing) {
                return existing.id == obj.id;
            });
            if (it == dst.end()) {
                dst.push_back(obj);
            }
        }
    };

    auto containsFiducial = [](const TrackedObjectList& list, const int fidId) -> bool {
        return std::any_of(list.begin(), list.end(), [fidId](const TrackedObject& obj) {
            return obj.id == fidId;
        });
    };

    TrackedObjectList fiducials;

    // In single-filter mode, bypass the adaptive multi-filter training
    // and always use the current active filter (default: OtsuBinary).
    if (onlySingleFilter_) {
        fiducials = runFilter(activeFilter_);
        return fiducials;
    }

    // 1) Ventana de entrenamiento: evaluar todos los filtros
    // durante kTrainingWindowFrames para un fiducial principal.
    if (trainingFramesRemaining_ > 0 && primaryFiducialId_ > 0) {
        for (int i = 0; i < kNumThresholdFilters; ++i) {
            const auto filter = allFilters[i];
            const TrackedObjectList filterResult = runFilter(filter);
            mergeResults(filterResult, fiducials);

            for (const auto& obj : filterResult) {
                filterSeenIds_[i].insert(obj.id);
            }

            if (containsFiducial(filterResult, primaryFiducialId_)) {
                ++filterSuccessCount_[i];
            }
        }

        if (containsFiducial(fiducials, primaryFiducialId_)) {
            framesSincePrimarySeen_ = 0;
        } else {
            ++framesSincePrimarySeen_;
            if (framesSincePrimarySeen_ > 4) {
                // Discard this fiducial immediately if it has been
                // missing for more than 4 consecutive frames.
                primaryFiducialId_ = -1;
                trainingFramesRemaining_ = 0;
                hasActiveFilter_ = false;
                primaryLostLastFrame_ = false;
                framesSincePrimarySeen_ = 0;
                std::fill(filterSuccessCount_.begin(), filterSuccessCount_.end(), 0);
                for (auto& ids : filterSeenIds_) {
                    ids.clear();
                }
                return fiducials;
            }
        }

        --trainingFramesRemaining_;

        if (trainingFramesRemaining_ == 0) {
            // Elegir el filtro ganador para este fiducial. En caso
            // de discrepancia, se prioriza el filtro que ha visto
            // menos IDs distintos a lo largo de la ventana, siempre
            // que haya detectado el fiducial principal al menos una vez.
            int bestIndex = -1;
            int bestCount = -1;
            int bestIdSetSize = std::numeric_limits<int>::max();

            for (int i = 0; i < kNumThresholdFilters; ++i) {
                const int count = filterSuccessCount_[i];
                if (count <= 0) {
                    continue;
                }

                const int idSetSize = static_cast<int>(filterSeenIds_[i].size());

                if (idSetSize < bestIdSetSize ||
                    (idSetSize == bestIdSetSize && count > bestCount)) {
                    bestIdSetSize = idSetSize;
                    bestCount = count;
                    bestIndex = i;
                }
            }

            if (bestIndex >= 0 && bestCount > 0) {
                activeFilter_ = allFilters[bestIndex];
                hasActiveFilter_ = true;

#ifndef NDEBUG
                std::cerr << "[soundtable-tracker] filter training completed for fiducial "
                          << primaryFiducialId_ << " over " << kTrainingWindowFrames
                          << " frames. Results:";
                for (int i = 0; i < kNumThresholdFilters; ++i) {
                    const int idSetSize = static_cast<int>(filterSeenIds_[i].size());
                    std::cerr << " " << kFilterNames[i] << "=" << filterSuccessCount_[i]
                              << " (ids=" << idSetSize << ")";
                }
                std::cerr << " (winner=" << kFilterNames[bestIndex] << ")" << std::endl;
#endif
            }
        }

        primaryLostLastFrame_ = false;
        return fiducials;
    }

    // 2) Filtro fijado: usar sólo el filtro ganador hasta que el
    // fiducial principal deje de verse.
    if (hasActiveFilter_ && primaryFiducialId_ > 0 && !primaryLostLastFrame_) {
        fiducials = runFilter(activeFilter_);

        if (containsFiducial(fiducials, primaryFiducialId_)) {
            framesSincePrimarySeen_ = 0;
            return fiducials;
        }

        // El filtro activo ha dejado de ver al fiducial; el
        // siguiente frame probará todos los demás filtros. También
        // contamos cuántos frames lleva desaparecido.
        ++framesSincePrimarySeen_;
        if (framesSincePrimarySeen_ > 4) {
            primaryFiducialId_ = -1;
            trainingFramesRemaining_ = 0;
            hasActiveFilter_ = false;
            primaryLostLastFrame_ = false;
            framesSincePrimarySeen_ = 0;
            std::fill(filterSuccessCount_.begin(), filterSuccessCount_.end(), 0);
            for (auto& ids : filterSeenIds_) {
                ids.clear();
            }
        } else {
            primaryLostLastFrame_ = true;
        }
        return fiducials;
    }

    // 3) Tras perder el fiducial con el filtro activo, probar todos
    // los filtros. Si ninguno vuelve a verlo, se reinicia el estado
    // adaptativo y se vuelve al modo inicial.
    if (hasActiveFilter_ && primaryFiducialId_ > 0 && primaryLostLastFrame_) {
        bool seenAgain = false;

        // Evaluar todos los filtros (incluido el activo) para ver si
        // alguno vuelve a detectar el fiducial principal.
        std::fill(filterSuccessCount_.begin(), filterSuccessCount_.end(), 0);
        for (auto& ids : filterSeenIds_) {
            ids.clear();
        }

        for (int i = 0; i < kNumThresholdFilters; ++i) {
            const auto filter = allFilters[i];
            const TrackedObjectList filterResult = runFilter(filter);
            mergeResults(filterResult, fiducials);

            for (const auto& obj : filterResult) {
                filterSeenIds_[i].insert(obj.id);
            }

            if (containsFiducial(filterResult, primaryFiducialId_)) {
                seenAgain = true;
                ++filterSuccessCount_[i];
            }
        }

        if (seenAgain) {
            framesSincePrimarySeen_ = 0;
            // Volvemos a ver el fiducial: reiniciar ventana de
            // entrenamiento completa desde este frame.
            trainingFramesRemaining_ = kTrainingWindowFrames;
            primaryLostLastFrame_ = false;
            // `filterSuccessCount_` ya contiene el conteo del
            // primer frame de la nueva ventana.
            return fiducials;
        }

        ++framesSincePrimarySeen_;
        if (framesSincePrimarySeen_ > 4) {
            // El fiducial lleva más de 4 frames sin aparecer con
            // ningún filtro: descartar inmediatamente.
            hasActiveFilter_ = false;
            primaryFiducialId_ = -1;
            primaryLostLastFrame_ = false;
            trainingFramesRemaining_ = 0;
            framesSincePrimarySeen_ = 0;
            std::fill(filterSuccessCount_.begin(), filterSuccessCount_.end(), 0);
            for (auto& ids : filterSeenIds_) {
                ids.clear();
            }
        }

        // Devolvemos lo que hayamos visto en este frame, aunque ya
        // no esté el fiducial principal.
        return fiducials;
    }

    // 4) Modo base: aún no hay fiducial principal ni filtro activo.
    // Ejecutar todos los filtros, devolver la unión de detecciones y
    // si aparece un fiducial, comenzar una ventana de entrenamiento.
    std::fill(filterSuccessCount_.begin(), filterSuccessCount_.end(), 0);
    for (auto& ids : filterSeenIds_) {
        ids.clear();
    }

    bool seenPrimaryThisFrame = false;

    for (int i = 0; i < kNumThresholdFilters; ++i) {
        const auto filter = allFilters[i];
        const TrackedObjectList filterResult = runFilter(filter);
        mergeResults(filterResult, fiducials);

        for (const auto& obj : filterResult) {
            filterSeenIds_[i].insert(obj.id);
        }

        if (primaryFiducialId_ <= 0 && !filterResult.empty()) {
            // Escoger el primer fiducial visto como referencia
            // para esta fase de entrenamiento.
            primaryFiducialId_ = filterResult.front().id;
        }

        if (primaryFiducialId_ > 0 && containsFiducial(filterResult, primaryFiducialId_)) {
            ++filterSuccessCount_[i];
            seenPrimaryThisFrame = true;
        }
    }

    if (primaryFiducialId_ > 0 && trainingFramesRemaining_ == 0) {
        trainingFramesRemaining_ = kTrainingWindowFrames;
        // Este frame cuenta como el primero de la ventana.
        --trainingFramesRemaining_;
        framesSincePrimarySeen_ = seenPrimaryThisFrame ? 0 : 1;
    }

    return fiducials;
}

TrackedObjectList TrackerEngine::detectAmoebaFiducials(const cv::Mat& binaryFrame)
{
    TrackedObjectList result;

    if (binaryFrame.empty() || binaryFrame.cols != frameWidth_ || binaryFrame.rows != frameHeight_) {
        return result;
    }

    if (binaryFrame.type() != CV_8UC1) {
        return result;
    }

    FiducialX fiducials[kMaxFiducials];

    step_segmenter(&segmenter_, binaryFrame.ptr<unsigned char>());

#ifndef NDEBUG
    sanity_check_region_initial_values(&segmenter_);
#endif

    const int count = find_fiducialsX(fiducials, kMaxFiducials, &fidtrackerX_, &segmenter_, frameWidth_, frameHeight_);

    for (int i = 0; i < count; ++i) {
        const int fidId = fiducials[i].id;
        // Only keep fiducials within the allowed ID range
        if (fidId <= kMinRecognisedMarkerId || fidId >= kMaxRecognisedMarkerId) {
            continue;
        }

        const float xNorm = fiducials[i].x / static_cast<float>(frameWidth_);
        const float yNorm = fiducials[i].y / static_cast<float>(frameHeight_);

        if (xNorm < 0.0F || xNorm > 1.0F || yNorm < 0.0F || yNorm > 1.0F) {
            continue;
        }

        TrackedObject obj;
        obj.id = fidId;
        obj.x_norm = xNorm;
        obj.y_norm = yNorm;
        obj.angle_rad = fiducials[i].angle;
        obj.alive = true;
        result.push_back(obj);
    }

    return result;
}

int TrackerEngine::filterIndex(const ThresholdFilter filter) noexcept
{
    switch (filter) {
    case ThresholdFilter::OtsuBinary:
        return 0;
    case ThresholdFilter::OtsuBinaryInv:
        return 1;
    case ThresholdFilter::AdaptiveBinary:
        return 2;
    case ThresholdFilter::AdaptiveBinaryInv:
        return 3;
    }

    return 0;
}

} // namespace soundtable::tracker
