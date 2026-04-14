#pragma once

#include <QObject>
#include <QTimer>
#include <QImage>
#include <QString>
#include <cstring>

#include "MvCamera.h"

class CameraManager : public QObject
{
    Q_OBJECT

public:
    explicit CameraManager(QObject* parent = nullptr);
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

public slots:
    void startCapture();
    void stopCapture();
    void grabFrame();

signals:
    void frameReady(const QImage& image);
    void logMessage(const QString& msg);
    void cameraOpened(bool ok);

private:
    CMvCamera m_camera;
    MV_CC_DEVICE_INFO_LIST m_deviceList;
    bool m_sdkInited;
    bool m_opened;
    bool m_grabbing;
    QTimer* m_timer;

    unsigned char* m_rgbBuffer;
    unsigned int m_rgbBufferSize;

    bool m_autoExposureLockDone = false;
    int m_autoExposureWarmupMs = 1500;   // 自动曝光预热时间，单位毫秒
    float m_lockedExposureTime = 0.0f;
    float m_lockedGain = 0.0f;
};