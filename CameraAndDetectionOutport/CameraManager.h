#pragma once
#include"MvCamera.h"

class CameraManager
{
public:
    CameraManager();
    ~CameraManager();

    bool initialize();
    void finalize();

    bool enumDevices();
    bool openFirstCamera();
    bool closeCamera();

    bool enableAutoExposure();
    bool lockCurrentExposure();
    bool setExposureAuto(bool enable);
    bool setGainAuto(bool enable);
    bool setExposureTime(float exposureUs);
    bool setGain(float gain);

    int stopCapture();  //返回0，表示停止成功。返回-1，停止失败
    int startCapture(); //返回0，表示抓取成功。返回-1，抓取失败

private:
    CMvCamera m_camera;
    MV_CC_DEVICE_INFO_LIST m_deviceList;
    bool m_sdkInited;
    bool m_opened;
    bool m_grabbing;

    unsigned char* m_rgbBuffer;
    unsigned int m_rgbBufferSize;

    bool m_autoExposureLockDone = false;
    int m_autoExposureWarmupMs = 1500;   // 自动曝光预热时间，单位毫秒
    float m_lockedExposureTime = 0.0f;
    float m_lockedGain = 0.0f;

    int m_cameraStartErr;
    //0:相机启动成功；1：Camera SDK init failed；2：No camera found；
    //3：Open camera failed；4: startCapture Error
};

