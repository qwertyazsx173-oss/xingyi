#include "DetectionWorker.h"

#include <algorithm>
#include <chrono>
#include <exception>
#include <sstream>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace
{
std::int64_t nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string formatMessage(const std::string& prefix, const std::string& detail)
{
    if (detail.empty())
    {
        return prefix;
    }

    return prefix + ": " + detail;
}
} // namespace

DetectionWorker::DetectionWorker()
    : DetectionWorker(DetectionConfig{})
{
}

DetectionWorker::DetectionWorker(const DetectionConfig& config)
    : m_config(config)
{
}

bool DetectionWorker::initialize()
{
    bool ok = true;

    if (!m_config.stage1.modelPath.empty())
    {
        ok = loadStage1Model(m_config.stage1.modelPath) && ok;
    }

    if (!m_config.stage2.modelPath.empty())
    {
        ok = loadStage2Model(m_config.stage2.modelPath) && ok;
    }

    return ok;
}

void DetectionWorker::reset()
{
    m_netStage1 = cv::dnn::Net();
    m_netStage2 = cv::dnn::Net();
    m_stage1Loaded = false;
    m_stage2Loaded = false;
    m_currentStage = DetectionStage::FirstDetect;
    m_lastError.clear();
}

void DetectionWorker::setConfig(const DetectionConfig& config)
{
    m_config = config;
}

const DetectionConfig& DetectionWorker::config() const
{
    return m_config;
}

bool DetectionWorker::loadModels(const std::string& stage1ModelPath,
                                 const std::string& stage2ModelPath)
{
    bool ok = true;

    if (!stage1ModelPath.empty())
    {
        ok = loadStage1Model(stage1ModelPath) && ok;
    }

    if (!stage2ModelPath.empty())
    {
        ok = loadStage2Model(stage2ModelPath) && ok;
    }

    return ok;
}

bool DetectionWorker::loadStage1Model(const std::string& modelPath)
{
    m_config.stage1.modelPath = modelPath;
    return loadModel(modelPath, m_netStage1, m_stage1Loaded, "Stage1");
}

bool DetectionWorker::loadStage2Model(const std::string& modelPath)
{
    m_config.stage2.modelPath = modelPath;
    return loadModel(modelPath, m_netStage2, m_stage2Loaded, "Stage2");
}

bool DetectionWorker::isStage1Loaded() const
{
    return m_stage1Loaded;
}

bool DetectionWorker::isStage2Loaded() const
{
    return m_stage2Loaded;
}

void DetectionWorker::setDetectStage(DetectionStage stage)
{
    m_currentStage = stage;
    log("Detection stage switched to " + std::to_string(static_cast<int>(stage)));
}

void DetectionWorker::setDetectStage(int stage)
{
    switch (stage)
    {
    case 0:
        setDetectStage(DetectionStage::FirstDetect);
        break;
    case 1:
        setDetectStage(DetectionStage::MovingToSecondPose);
        break;
    case 2:
        setDetectStage(DetectionStage::SecondDetect);
        break;
    case 3:
        setDetectStage(DetectionStage::Finished);
        break;
    default:
        setError("Unsupported detection stage: " + std::to_string(stage));
        break;
    }
}

DetectionStage DetectionWorker::detectStage() const
{
    return m_currentStage;
}

void DetectionWorker::setOutputScaleBack(double scale)
{
    if (scale > 0.0)
    {
        m_config.outputScaleBack = scale;
    }
}

double DetectionWorker::outputScaleBack() const
{
    return m_config.outputScaleBack;
}

bool DetectionWorker::processFrame(const cv::Mat& imageBgr, CameraDetectionResult& result)
{
    result = CameraDetectionResult{};
    result.timestampMs = nowMs();

    if (m_currentStage == DetectionStage::SecondDetect)
    {
        if (!m_stage2Loaded)
        {
            setError("[Stage2] ONNX model is not loaded");
            result.stageUsed = static_cast<int>(DetectionStage::SecondDetect);
            return false;
        }

        return detectWithNet(imageBgr,
                             result,
                             m_netStage2,
                             m_config.stage2.confThreshold,
                             m_config.stage2.nmsThreshold,
                             "Stage2",
                             static_cast<int>(DetectionStage::SecondDetect));
    }

    if (!m_stage1Loaded)
    {
        setError("[Stage1] ONNX model is not loaded");
        result.stageUsed = static_cast<int>(DetectionStage::FirstDetect);
        return false;
    }

    return detectWithNet(imageBgr,
                         result,
                         m_netStage1,
                         m_config.stage1.confThreshold,
                         m_config.stage1.nmsThreshold,
                         "Stage1",
                         static_cast<int>(DetectionStage::FirstDetect));
}

const std::string& DetectionWorker::lastError() const
{
    return m_lastError;
}

void DetectionWorker::setLogCallback(LogCallback callback)
{
    m_logCallback = std::move(callback);
}

void DetectionWorker::log(const std::string& message) const
{
    if (m_logCallback)
    {
        m_logCallback(message);
    }
}

void DetectionWorker::setError(const std::string& message)
{
    m_lastError = message;
    log(message);
}

bool DetectionWorker::loadModel(const std::string& modelPath,
                                cv::dnn::Net& net,
                                bool& loadedFlag,
                                const std::string& tag)
{
    if (modelPath.empty())
    {
        loadedFlag = false;
        setError("[" + tag + "] model path is empty");
        return false;
    }

    try
    {
        net = cv::dnn::readNetFromONNX(modelPath);
        loadedFlag = !net.empty();

        if (loadedFlag)
        {
            log("[" + tag + "] ONNX model loaded: " + modelPath);
            return true;
        }

        setError("[" + tag + "] failed to load ONNX model: " + modelPath);
        return false;
    }
    catch (const cv::Exception& e)
    {
        loadedFlag = false;
        setError(formatMessage("[" + tag + "] OpenCV exception while loading model", e.what()));
        return false;
    }
    catch (const std::exception& e)
    {
        loadedFlag = false;
        setError(formatMessage("[" + tag + "] standard exception while loading model", e.what()));
        return false;
    }
}

cv::Mat DetectionWorker::normalizeInputImage(const cv::Mat& imageBgr) const
{
    if (imageBgr.empty())
    {
        return cv::Mat();
    }

    if (imageBgr.type() == CV_8UC3)
    {
        return imageBgr;
    }

    cv::Mat converted;

    switch (imageBgr.channels())
    {
    case 1:
        cv::cvtColor(imageBgr, converted, cv::COLOR_GRAY2BGR);
        break;
    case 3:
        imageBgr.convertTo(converted, CV_8UC3);
        break;
    case 4:
        cv::cvtColor(imageBgr, converted, cv::COLOR_BGRA2BGR);
        break;
    default:
        break;
    }

    return converted;
}

cv::Mat DetectionWorker::letterboxSquare(const cv::Mat& src) const
{
    if (src.empty())
    {
        return cv::Mat();
    }

    const int w = src.cols;
    const int h = src.rows;
    const int m = std::max(w, h);

    cv::Mat dst = cv::Mat::zeros(m, m, CV_8UC3);
    src.copyTo(dst(cv::Rect(0, 0, w, h)));
    return dst;
}

bool DetectionWorker::detectWithNet(const cv::Mat& imageBgr,
                                    CameraDetectionResult& result,
                                    cv::dnn::Net& net,
                                    float confThreshold,
                                    float nmsThreshold,
                                    const std::string& tag,
                                    int stageValue)
{
    try
    {
        result = CameraDetectionResult{};
        result.timestampMs = nowMs();
        result.stageUsed = stageValue;

        cv::Mat frame = normalizeInputImage(imageBgr);
        if (frame.empty())
        {
            setError("[" + tag + "] input frame is empty or unsupported");
            return false;
        }

        cv::Mat inputImage = letterboxSquare(frame);
        if (inputImage.empty())
        {
            setError("[" + tag + "] failed to create letterboxed input");
            return false;
        }

        const float xFactor = static_cast<float>(inputImage.cols) /
                              static_cast<float>(m_config.inputWidth);
        const float yFactor = static_cast<float>(inputImage.rows) /
                              static_cast<float>(m_config.inputHeight);

        cv::Mat blob;
        cv::dnn::blobFromImage(inputImage,
                               blob,
                               1.0 / 255.0,
                               cv::Size(m_config.inputWidth, m_config.inputHeight),
                               cv::Scalar(),
                               true,
                               false);

        net.setInput(blob);
        cv::Mat output = net.forward();

        int dimensions = 0;
        int rows = 0;
        cv::Mat outputT;

        if (output.dims == 3 && output.size[1] == 5)
        {
            dimensions = output.size[1];
            rows = output.size[2];
            cv::Mat output0(dimensions, rows, CV_32F, output.ptr<float>());
            outputT = output0.t();
        }
        else if (output.dims == 3 && output.size[2] == 5)
        {
            rows = output.size[1];
            dimensions = output.size[2];
            outputT = cv::Mat(rows, dimensions, CV_32F, output.ptr<float>());
        }
        else
        {
            std::ostringstream oss;
            oss << "[" << tag << "] unexpected YOLO output shape";
            setError(oss.str());
            return false;
        }

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;

        for (int i = 0; i < outputT.rows; ++i)
        {
            const float* data = outputT.ptr<float>(i);

            const float cx = data[0];
            const float cy = data[1];
            const float w = data[2];
            const float h = data[3];
            const float score = data[4];

            if (score < confThreshold)
            {
                continue;
            }

            const int left = static_cast<int>((cx - 0.5f * w) * xFactor);
            const int top = static_cast<int>((cy - 0.5f * h) * yFactor);
            const int width = static_cast<int>(w * xFactor);
            const int height = static_cast<int>(h * yFactor);

            boxes.emplace_back(left, top, width, height);
            scores.push_back(score);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, confThreshold, nmsThreshold, indices);

        if (indices.empty())
        {
            result.found = false;
            log("[" + tag + "] no target detected");
            return false;
        }

        int bestIdx = indices[0];
        float bestScore = scores[bestIdx];

        for (int idx : indices)
        {
            if (scores[idx] > bestScore)
            {
                bestScore = scores[idx];
                bestIdx = idx;
            }
        }

        const cv::Rect imageBounds(0, 0, frame.cols, frame.rows);
        const cv::Rect bestBox = boxes[bestIdx] & imageBounds;
        if (bestBox.width <= 0 || bestBox.height <= 0)
        {
            result.found = false;
            setError("[" + tag + "] best bounding box is invalid after clipping");
            return false;
        }

        const double scale = (m_config.outputScaleBack > 0.0) ? m_config.outputScaleBack : 1.0;

        result.found = true;
        result.score = bestScore;
        result.bboxX = static_cast<int>(bestBox.x * scale);
        result.bboxY = static_cast<int>(bestBox.y * scale);
        result.bboxW = static_cast<int>(bestBox.width * scale);
        result.bboxH = static_cast<int>(bestBox.height * scale);
        result.centerX = result.bboxX + result.bboxW / 2;
        result.centerY = result.bboxY + result.bboxH / 2;
        result.radius = std::min(result.bboxW, result.bboxH) / 2;

        std::ostringstream oss;
        oss << "[" << tag << "] detection success: score=" << bestScore
            << ", box=[" << bestBox.x << "," << bestBox.y << ","
            << bestBox.width << "," << bestBox.height << "]";
        log(oss.str());

        return true;
    }
    catch (const cv::Exception& e)
    {
        setError(formatMessage("[" + tag + "] OpenCV exception", e.what()));
        return false;
    }
    catch (const std::exception& e)
    {
        setError(formatMessage("[" + tag + "] standard exception", e.what()));
        return false;
    }
    catch (...)
    {
        setError("[" + tag + "] unknown exception");
        return false;
    }
}
