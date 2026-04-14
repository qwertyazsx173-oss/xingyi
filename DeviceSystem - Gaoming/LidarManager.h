#pragma once

#include <QObject>
#include <QString>
#include <vector>
#include <QDateTime>


#include "sl_lidar.h"
#include "sl_lidar_driver.h"
//给雷达定义的结果结构体
struct RadarDetectionResult
{
    bool found = false;
    float angleDeg = 0.0f;
    float distanceM = 0.0f;
    qint64 timestamp = 0;
};

Q_DECLARE_METATYPE(RadarDetectionResult)

class LidarManager : public QObject
{
    Q_OBJECT

public:
    struct ScanPoint
    {
        float angleDeg;
        float distMm;
        int quality;
        bool isSync;
    };

public:
    explicit LidarManager(QObject* parent = nullptr);
    ~LidarManager();

    bool initialize();
    void finalize();

    bool connectDevice(const char* port, sl_u32 baudrate);
    bool checkHealth();
    bool startScan();
    void stopScan();
    bool grabOneScan(std::vector<ScanPoint>& points);

signals:
    void logMessage(const QString& msg);

private:
    sl::ILidarDriver* m_drv;
    bool m_inited;
    bool m_connected;
    bool m_scanning;
};