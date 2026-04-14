#pragma once

#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

#include "MvCamera.h"

struct CameraFrame
{
    cv::Mat image;                 // BGR, OpenCV-friendly
    std::int64_t timestampMs = 0;  // steady wall-clock snapshot time
    unsigned int width = 0;
    unsigned int height = 0;
    unsigned int frameNumber = 0;
};

class CameraManager
{
public:
    using LogCallback = std::function<void(const std::string&)>;

public:
    CameraManager();
    ~CameraManager();

    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;

    bool initialize();
    void finalize();

    bool enumDevices();
    unsigned int deviceCount() const;

    bool openFirstCamera();
    bool openCamera(unsigned int index);
    bool closeCamera();

    bool enableAutoExposure();
    bool lockCurrentExposure();

    bool setExposureAuto(bool enable);
    bool setGainAuto(bool enable);
    bool setExposureTime(float exposureUs);
    bool setGain(float gain);

    bool startCapture();
    bool stopCapture();

    bool grabFrame(cv::Mat& imageBgr, int timeoutMs = 1000);
    bool grabFrame(CameraFrame& frame, int timeoutMs = 1000);

    bool isSdkInitialized() const;
    bool isOpened() const;
    bool isGrabbing() const;

    float lockedExposureTime() const;
    float lockedGain() const;

    const std::string& lastError() const;
    void setLogCallback(LogCallback callback);

private:
    void log(const std::string& message) const;
    void setError(const std::string& message);
    void tryLockAutoExposure();
    bool ensureRgbBufferSize(unsigned int rgbSize);

private:
    CMvCamera m_camera;
    MV_CC_DEVICE_INFO_LIST m_deviceList;

    bool m_sdkInited = false;
    bool m_opened = false;
    bool m_grabbing = false;

    std::vector<unsigned char> m_rgbBuffer;

    bool m_autoExposureLockDone = false;
    int m_autoExposureWarmupMs = 1500;
    float m_lockedExposureTime = 0.0f;
    float m_lockedGain = 0.0f;

    std::int64_t m_captureStartTimestampMs = 0;

    std::string m_lastError;
    LogCallback m_logCallback;
};
