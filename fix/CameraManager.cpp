#include "CameraManager.h"

#include <chrono>
#include <sstream>

#include <opencv2/imgproc.hpp>

namespace
{
std::int64_t nowMs()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

std::string hexRetMessage(const std::string& prefix, int nRet)
{
    std::ostringstream oss;
    oss << prefix << ", nRet = 0x" << std::hex << nRet;
    return oss.str();
}
} // namespace

CameraManager::CameraManager()
{
    std::memset(&m_deviceList, 0, sizeof(m_deviceList));
}

CameraManager::~CameraManager()
{
    stopCapture();
    closeCamera();
    finalize();
}

bool CameraManager::initialize()
{
    if (m_sdkInited)
    {
        return true;
    }

    const int nRet = m_camera.InitSDK();
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("Camera SDK init failed", nRet));
        return false;
    }

    m_sdkInited = true;
    log("Camera SDK initialized");
    return true;
}

void CameraManager::finalize()
{
    if (!m_sdkInited)
    {
        return;
    }

    m_camera.FinalizeSDK();
    m_sdkInited = false;
    log("Camera SDK finalized");
}

bool CameraManager::enumDevices()
{
    if (!m_sdkInited)
    {
        setError("Cannot enumerate cameras before SDK initialization");
        return false;
    }

    std::memset(&m_deviceList, 0, sizeof(m_deviceList));

    const int nRet = CMvCamera::EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_deviceList);
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("EnumDevices failed", nRet));
        return false;
    }

    if (m_deviceList.nDeviceNum == 0)
    {
        setError("No camera found");
        return false;
    }

    log("Camera enumeration succeeded, device count = " + std::to_string(m_deviceList.nDeviceNum));
    return true;
}

unsigned int CameraManager::deviceCount() const
{
    return m_deviceList.nDeviceNum;
}

bool CameraManager::openFirstCamera()
{
    return openCamera(0);
}

bool CameraManager::openCamera(unsigned int index)
{
    if (!m_sdkInited)
    {
        setError("Cannot open camera before SDK initialization");
        return false;
    }

    if (m_opened)
    {
        return true;
    }

    if (m_deviceList.nDeviceNum == 0)
    {
        setError("Cannot open camera because no enumerated device is available");
        return false;
    }

    if (index >= m_deviceList.nDeviceNum)
    {
        setError("Camera index out of range");
        return false;
    }

    const int nRet = m_camera.Open(m_deviceList.pDeviceInfo[index]);
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("Open camera failed", nRet));
        return false;
    }

    m_opened = true;
    m_autoExposureLockDone = false;
    m_lockedExposureTime = 0.0f;
    m_lockedGain = 0.0f;
    log("Camera opened successfully");
    return true;
}

bool CameraManager::closeCamera()
{
    if (!m_opened)
    {
        return true;
    }

    stopCapture();

    const int nRet = m_camera.Close();
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("Close camera failed", nRet));
        return false;
    }

    m_opened = false;
    log("Camera closed");
    return true;
}

bool CameraManager::startCapture()
{
    if (!m_opened)
    {
        setError("Camera is not opened, cannot start capture");
        return false;
    }

    if (m_grabbing)
    {
        return true;
    }

    const int nRet = m_camera.StartGrabbing();
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("StartGrabbing failed", nRet));
        return false;
    }

    m_grabbing = true;
    m_captureStartTimestampMs = nowMs();

    // Keep the same strategy as the Qt version:
    // start with auto exposure/gain, then lock the stabilized values later.
    if (!enableAutoExposure())
    {
        log("Auto exposure warmup could not be enabled; continuing capture anyway");
    }

    log("Camera capture started");
    return true;
}

bool CameraManager::stopCapture()
{
    if (!m_grabbing)
    {
        return true;
    }

    const int nRet = m_camera.StopGrabbing();
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("StopGrabbing failed", nRet));
        return false;
    }

    m_grabbing = false;
    log("Camera capture stopped");
    return true;
}

bool CameraManager::grabFrame(cv::Mat& imageBgr, int timeoutMs)
{
    CameraFrame frame;
    if (!grabFrame(frame, timeoutMs))
    {
        return false;
    }

    imageBgr = frame.image;
    return true;
}

bool CameraManager::grabFrame(CameraFrame& frame, int timeoutMs)
{
    if (!m_grabbing)
    {
        setError("Camera is not grabbing, cannot fetch frame");
        return false;
    }

    MV_FRAME_OUT stFrame = {};
    const int getRet = m_camera.GetImageBuffer(&stFrame, timeoutMs);
    if (getRet != MV_OK)
    {
        setError(hexRetMessage("GetImageBuffer failed", getRet));
        return false;
    }

    const unsigned int width = stFrame.stFrameInfo.nWidth;
    const unsigned int height = stFrame.stFrameInfo.nHeight;
    const unsigned int rgbSize = width * height * 3;

    if (!ensureRgbBufferSize(rgbSize))
    {
        m_camera.FreeImageBuffer(&stFrame);
        return false;
    }

    MV_CC_PIXEL_CONVERT_PARAM_EX convertParam = {};
    convertParam.nWidth = width;
    convertParam.nHeight = height;
    convertParam.pSrcData = stFrame.pBufAddr;
    convertParam.nSrcDataLen = stFrame.stFrameInfo.nFrameLen;
    convertParam.enSrcPixelType = stFrame.stFrameInfo.enPixelType;
    convertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    convertParam.pDstBuffer = m_rgbBuffer.data();
    convertParam.nDstBufferSize = rgbSize;

    const int convertRet = m_camera.ConvertPixelType(&convertParam);
    if (convertRet != MV_OK)
    {
        m_camera.FreeImageBuffer(&stFrame);
        setError(hexRetMessage("ConvertPixelType failed", convertRet));
        return false;
    }

    const cv::Mat rgbView(static_cast<int>(height), static_cast<int>(width), CV_8UC3, m_rgbBuffer.data());
    cv::cvtColor(rgbView, frame.image, cv::COLOR_RGB2BGR);

    frame.timestampMs = nowMs();
    frame.width = width;
    frame.height = height;
    frame.frameNumber = stFrame.stFrameInfo.nFrameNum;

    m_camera.FreeImageBuffer(&stFrame);

    tryLockAutoExposure();
    return true;
}

bool CameraManager::enableAutoExposure()
{
    const bool ok1 = setExposureAuto(true);
    const bool ok2 = setGainAuto(true);

    m_autoExposureLockDone = false;
    m_captureStartTimestampMs = nowMs();
    return ok1 && ok2;
}

bool CameraManager::lockCurrentExposure()
{
    if (!m_opened)
    {
        setError("Camera is not opened, cannot lock exposure");
        return false;
    }

    MVCC_FLOATVALUE exposureValue = {};
    MVCC_FLOATVALUE gainValue = {};

    const int retExposure = m_camera.GetFloatValue("ExposureTime", &exposureValue);
    if (retExposure != MV_OK)
    {
        setError(hexRetMessage("Read ExposureTime failed", retExposure));
        return false;
    }

    const int retGain = m_camera.GetFloatValue("Gain", &gainValue);
    if (retGain != MV_OK)
    {
        setError(hexRetMessage("Read Gain failed", retGain));
        return false;
    }

    m_lockedExposureTime = exposureValue.fCurValue;
    m_lockedGain = gainValue.fCurValue;

    log("Auto exposure result: ExposureTime=" + std::to_string(m_lockedExposureTime) +
        " us, Gain=" + std::to_string(m_lockedGain));

    const bool ok1 = setExposureAuto(false);
    const bool ok2 = setGainAuto(false);
    const bool ok3 = setExposureTime(m_lockedExposureTime);
    const bool ok4 = setGain(m_lockedGain);

    m_autoExposureLockDone = ok1 && ok2 && ok3 && ok4;
    if (m_autoExposureLockDone)
    {
        log("Auto exposure has been locked");
    }
    else
    {
        setError("Auto exposure lock failed");
    }

    return m_autoExposureLockDone;
}

bool CameraManager::setExposureAuto(bool enable)
{
    const int nRet = m_camera.SetEnumValueByString("ExposureAuto", enable ? "Continuous" : "Off");
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("Set ExposureAuto failed", nRet));
        return false;
    }

    log(std::string("ExposureAuto = ") + (enable ? "Continuous" : "Off"));
    return true;
}

bool CameraManager::setGainAuto(bool enable)
{
    const int nRet = m_camera.SetEnumValueByString("GainAuto", enable ? "Continuous" : "Off");
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("Set GainAuto failed", nRet));
        return false;
    }

    log(std::string("GainAuto = ") + (enable ? "Continuous" : "Off"));
    return true;
}

bool CameraManager::setExposureTime(float exposureUs)
{
    const int nRet = m_camera.SetFloatValue("ExposureTime", exposureUs);
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("Set ExposureTime failed", nRet));
        return false;
    }

    log("ExposureTime locked to " + std::to_string(exposureUs) + " us");
    return true;
}

bool CameraManager::setGain(float gain)
{
    const int nRet = m_camera.SetFloatValue("Gain", gain);
    if (nRet != MV_OK)
    {
        setError(hexRetMessage("Set Gain failed", nRet));
        return false;
    }

    log("Gain locked to " + std::to_string(gain));
    return true;
}

bool CameraManager::isSdkInitialized() const
{
    return m_sdkInited;
}

bool CameraManager::isOpened() const
{
    return m_opened;
}

bool CameraManager::isGrabbing() const
{
    return m_grabbing;
}

float CameraManager::lockedExposureTime() const
{
    return m_lockedExposureTime;
}

float CameraManager::lockedGain() const
{
    return m_lockedGain;
}

const std::string& CameraManager::lastError() const
{
    return m_lastError;
}

void CameraManager::setLogCallback(LogCallback callback)
{
    m_logCallback = std::move(callback);
}

void CameraManager::log(const std::string& message) const
{
    if (m_logCallback)
    {
        m_logCallback(message);
    }
}

void CameraManager::setError(const std::string& message)
{
    m_lastError = message;
    log(message);
}

void CameraManager::tryLockAutoExposure()
{
    if (!m_grabbing || m_autoExposureLockDone)
    {
        return;
    }

    const std::int64_t elapsedMs = nowMs() - m_captureStartTimestampMs;
    if (elapsedMs < m_autoExposureWarmupMs)
    {
        return;
    }

    lockCurrentExposure();
}

bool CameraManager::ensureRgbBufferSize(unsigned int rgbSize)
{
    try
    {
        if (m_rgbBuffer.size() < rgbSize)
        {
            m_rgbBuffer.resize(rgbSize);
        }
        return true;
    }
    catch (...)
    {
        setError("Failed to allocate RGB conversion buffer");
        return false;
    }
}
