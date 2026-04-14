#include "CameraManager.h"
#include<string>

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
    //emit cameraOpened(true);
    return true;
}



CameraManager::CameraManager()
{
    m_sdkInited = false;
    m_opened(false),
        m_grabbing(false),
        m_timer(nullptr),
        m_rgbBuffer(nullptr),
        m_rgbBufferSize(0);
    std::memset(&m_deviceList, 0, sizeof(m_deviceList));
    // -------- 相机测试 --------
    if (!this->initialize())
    {
        m_cameraStartErr = 1;
    }
    else if (!this->enumDevices())
    {
        m_cameraStartErr = 2;
    }
    else if (!this->openFirstCamera())
    {
        m_cameraStartErr = 3;
    }
    else
    {
        //cameraOk = true;
        m_cameraStartErr = 0;
        this->startCapture();
    }
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

int CameraManager::stopCapture()
{
    if (!m_grabbing)
        return;

    m_grabbing = false;

    int nRet = m_camera.StopGrabbing();
    if (nRet != MV_OK)
    {
        return 1;
    }

    return 0;// emit logMessage("停止采集画面");
}

int CameraManager::startCapture()
{

    if (!m_opened)
    {
        return -1;  // emit logMessage("相机尚未打开，无法开始采集");;
    }

    if (m_grabbing)
        return 0;

    int nRet = m_camera.StartGrabbing();
    if (nRet != MV_OK)
    {
        m_cameraStartErr = 4;
        //emit logMessage(QString("开始抓流失败, nRet = 0x%1").arg(nRet, 0, 16));
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