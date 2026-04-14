#include "CameraManager.h"
#include <QString>
#include <QTimer>

CameraManager::CameraManager(QObject* parent)
    : QObject(parent),
    m_sdkInited(false),
    m_opened(false),
    m_grabbing(false),
    m_timer(nullptr),
    m_rgbBuffer(nullptr),
    m_rgbBufferSize(0)
{
    std::memset(&m_deviceList, 0, sizeof(m_deviceList));
}

CameraManager::~CameraManager()
{
    stopCapture();
    closeCamera();
    finalize();
    if (m_rgbBuffer)
    {
        delete[] m_rgbBuffer;
        m_rgbBuffer = nullptr;
        m_rgbBufferSize = 0;
    }
}

bool CameraManager::initialize()
{
    if (m_sdkInited)
        return true;

    int nRet = m_camera.InitSDK();
    if (nRet != MV_OK)
        return false;

    m_sdkInited = true;
    return true;
}

void CameraManager::finalize()
{
    if (m_sdkInited)
    {
        m_camera.FinalizeSDK();
        m_sdkInited = false;
    }
}

bool CameraManager::enumDevices()
{
    if (!m_sdkInited)
        return false;

    std::memset(&m_deviceList, 0, sizeof(m_deviceList));

    int nRet = CMvCamera::EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &m_deviceList);
    if (nRet != MV_OK)
        return false;

    return m_deviceList.nDeviceNum > 0;
}

bool CameraManager::openFirstCamera()
{
    if (!m_sdkInited)
        return false;

    if (m_opened)
        return true;

    if (m_deviceList.nDeviceNum == 0)
        return false;

    int nRet = m_camera.Open(m_deviceList.pDeviceInfo[0]);
    if (nRet != MV_OK)
        return false;

    m_opened = true;
    emit cameraOpened(true);
    return true;
}

bool CameraManager::closeCamera()
{
    if (!m_opened)
        return true;

    int nRet = m_camera.Close();
    if (nRet != MV_OK)
        return false;

    m_opened = false;
    return true;
}

void CameraManager::startCapture()
{
    if (!m_opened)
    {
        emit logMessage("相机尚未打开，无法开始采集");
        return;
    }

    if (m_grabbing)
        return;

    int nRet = m_camera.StartGrabbing();
    if (nRet != MV_OK)
    {
        emit logMessage(QString("开始抓流失败, nRet = 0x%1").arg(nRet, 0, 16));
        return;
    }

    m_grabbing = true;

    if (!m_timer)
    {
        m_timer = new QTimer(this);
        connect(m_timer, &QTimer::timeout, this, &CameraManager::grabFrame);
    }

    m_timer->start(30);
    emit logMessage("开始采集画面");

    // 启动时自动曝光，稍后锁定
    enableAutoExposure();

    QTimer::singleShot(m_autoExposureWarmupMs, this, [this]()
        {
            if (!m_grabbing)
                return;

            if (m_autoExposureLockDone)
                return;

            lockCurrentExposure();
        });
}

void CameraManager::stopCapture()
{
    if (!m_grabbing)
        return;

    m_grabbing = false;

    if (m_timer)
        m_timer->stop();

    int nRet = m_camera.StopGrabbing();
    if (nRet != MV_OK)
    {
        emit logMessage(QString("停止抓流失败, nRet = 0x%1").arg(nRet, 0, 16));
    }

    emit logMessage("停止采集画面");
}

void CameraManager::grabFrame()
{
    if (!m_grabbing)
        return;

    MV_FRAME_OUT stFrame = {};
    int nRet = m_camera.GetImageBuffer(&stFrame, 1000);
    if (nRet != MV_OK)
    {
        emit logMessage(QString("取图失败, nRet = 0x%1").arg(nRet, 0, 16));
        return;
    }

    unsigned int width = stFrame.stFrameInfo.nWidth;
    unsigned int height = stFrame.stFrameInfo.nHeight;
    unsigned int rgbSize = width * height * 3;

    if (m_rgbBufferSize < rgbSize)
    {
        if (m_rgbBuffer)
            delete[] m_rgbBuffer;

        m_rgbBuffer = new unsigned char[rgbSize];
        m_rgbBufferSize = rgbSize;
    }

    MV_CC_PIXEL_CONVERT_PARAM_EX stConvertParam = {};
    stConvertParam.nWidth = width;
    stConvertParam.nHeight = height;
    stConvertParam.pSrcData = stFrame.pBufAddr;
    stConvertParam.nSrcDataLen = stFrame.stFrameInfo.nFrameLen;
    stConvertParam.enSrcPixelType = stFrame.stFrameInfo.enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    stConvertParam.pDstBuffer = m_rgbBuffer;
    stConvertParam.nDstBufferSize = rgbSize;

    nRet = m_camera.ConvertPixelType(&stConvertParam);
    if (nRet != MV_OK)
    {
        m_camera.FreeImageBuffer(&stFrame);
        emit logMessage(QString("像素格式转换失败, nRet = 0x%1").arg(nRet, 0, 16));
        return;
    }

    QImage image(m_rgbBuffer, width, height, width * 3, QImage::Format_RGB888);
    QImage imageCopy = image.copy();

    emit frameReady(imageCopy);

    m_camera.FreeImageBuffer(&stFrame);
}

bool CameraManager::setExposureAuto(bool enable)
{
    int nRet = m_camera.SetEnumValueByString("ExposureAuto", enable ? "Continuous" : "Off");
    if (nRet != MV_OK)
    {
        emit logMessage(QString("设置 ExposureAuto 失败, nRet = 0x%1").arg(nRet, 0, 16));
        return false;
    }

    emit logMessage(QString("ExposureAuto = %1").arg(enable ? "Continuous" : "Off"));
    return true;
}

bool CameraManager::setGainAuto(bool enable)
{
    int nRet = m_camera.SetEnumValueByString("GainAuto", enable ? "Continuous" : "Off");
    if (nRet != MV_OK)
    {
        emit logMessage(QString("设置 GainAuto 失败, nRet = 0x%1").arg(nRet, 0, 16));
        return false;
    }

    emit logMessage(QString("GainAuto = %1").arg(enable ? "Continuous" : "Off"));
    return true;
}

bool CameraManager::setExposureTime(float exposureUs)
{
    int nRet = m_camera.SetFloatValue("ExposureTime", exposureUs);
    if (nRet != MV_OK)
    {
        emit logMessage(QString("设置 ExposureTime 失败, nRet = 0x%1").arg(nRet, 0, 16));
        return false;
    }

    emit logMessage(QString("ExposureTime 锁定为 %1 us").arg(exposureUs, 0, 'f', 1));
    return true;
}

bool CameraManager::setGain(float gain)
{
    int nRet = m_camera.SetFloatValue("Gain", gain);
    if (nRet != MV_OK)
    {
        emit logMessage(QString("设置 Gain 失败, nRet = 0x%1").arg(nRet, 0, 16));
        return false;
    }

    emit logMessage(QString("Gain 锁定为 %1").arg(gain, 0, 'f', 2));
    return true;
}

bool CameraManager::enableAutoExposure()
{
    bool ok1 = setExposureAuto(true);
    bool ok2 = setGainAuto(true);

    m_autoExposureLockDone = false;
    return ok1 && ok2;
}

bool CameraManager::lockCurrentExposure()
{
    if (!m_opened)
    {
        emit logMessage("相机未打开，无法锁定曝光");
        return false;
    }

    MVCC_FLOATVALUE stExposure = {};
    MVCC_FLOATVALUE stGain = {};

    int nRet1 = m_camera.GetFloatValue("ExposureTime", &stExposure);
    int nRet2 = m_camera.GetFloatValue("Gain", &stGain);

    if (nRet1 != MV_OK)
    {
        emit logMessage(QString("读取 ExposureTime 失败, nRet = 0x%1").arg(nRet1, 0, 16));
        return false;
    }

    if (nRet2 != MV_OK)
    {
        emit logMessage(QString("读取 Gain 失败, nRet = 0x%1").arg(nRet2, 0, 16));
        return false;
    }

    m_lockedExposureTime = stExposure.fCurValue;
    m_lockedGain = stGain.fCurValue;

    emit logMessage(QString("自动曝光结果: ExposureTime=%1 us, Gain=%2")
        .arg(m_lockedExposureTime, 0, 'f', 1)
        .arg(m_lockedGain, 0, 'f', 2));

    bool ok1 = setExposureAuto(false);
    bool ok2 = setGainAuto(false);
    bool ok3 = setExposureTime(m_lockedExposureTime);
    bool ok4 = setGain(m_lockedGain);

    m_autoExposureLockDone = ok1 && ok2 && ok3 && ok4;

    if (m_autoExposureLockDone)
        emit logMessage("自动曝光已完成，并已锁定当前曝光参数");
    else
        emit logMessage("自动曝光锁定失败");

    return m_autoExposureLockDone;
}