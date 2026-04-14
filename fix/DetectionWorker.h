#pragma once

#include <cstdint>
#include <functional>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

struct CameraDetectionResult
{
    bool found = false;
    int centerX = -1;
    int centerY = -1;
    int bboxX = 0;
    int bboxY = 0;
    int bboxW = 0;
    int bboxH = 0;
    int radius = 0;
    float score = 0.0f;
    std::int64_t timestampMs = 0;
    int stageUsed = 0;
};

enum class DetectionStage
{
    FirstDetect = 0,
    MovingToSecondPose = 1,
    SecondDetect = 2,
    Finished = 3
};

struct DetectionModelConfig
{
    std::string modelPath;
    float confThreshold = 0.5f;
    float nmsThreshold = 0.45f;
};

struct DetectionConfig
{
    int inputWidth = 640;
    int inputHeight = 640;
    double outputScaleBack = 1.0;
    DetectionModelConfig stage1;
    DetectionModelConfig stage2;
};

class DetectionWorker
{
public:
    using LogCallback = std::function<void(const std::string&)>;

public:
    DetectionWorker();
    explicit DetectionWorker(const DetectionConfig& config);
    ~DetectionWorker() = default;

    DetectionWorker(const DetectionWorker&) = delete;
    DetectionWorker& operator=(const DetectionWorker&) = delete;

    bool initialize();
    void reset();

    void setConfig(const DetectionConfig& config);
    const DetectionConfig& config() const;

    bool loadModels(const std::string& stage1ModelPath,
                    const std::string& stage2ModelPath);
    bool loadStage1Model(const std::string& modelPath);
    bool loadStage2Model(const std::string& modelPath);

    bool isStage1Loaded() const;
    bool isStage2Loaded() const;

    void setDetectStage(DetectionStage stage);
    void setDetectStage(int stage);
    DetectionStage detectStage() const;

    void setOutputScaleBack(double scale);
    double outputScaleBack() const;

    bool processFrame(const cv::Mat& imageBgr, CameraDetectionResult& result);

    const std::string& lastError() const;
    void setLogCallback(LogCallback callback);

private:
    void log(const std::string& message) const;
    void setError(const std::string& message);

    bool loadModel(const std::string& modelPath,
                   cv::dnn::Net& net,
                   bool& loadedFlag,
                   const std::string& tag);

    cv::Mat normalizeInputImage(const cv::Mat& imageBgr) const;
    cv::Mat letterboxSquare(const cv::Mat& src) const;

    bool detectWithNet(const cv::Mat& imageBgr,
                       CameraDetectionResult& result,
                       cv::dnn::Net& net,
                       float confThreshold,
                       float nmsThreshold,
                       const std::string& tag,
                       int stageValue);

private:
    DetectionConfig m_config;

    cv::dnn::Net m_netStage1;
    cv::dnn::Net m_netStage2;

    bool m_stage1Loaded = false;
    bool m_stage2Loaded = false;

    DetectionStage m_currentStage = DetectionStage::FirstDetect;

    std::string m_lastError;
    LogCallback m_logCallback;
};
