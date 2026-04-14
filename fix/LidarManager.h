#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

struct RadarDetectionResult
{
    bool found = false;
    float angleDeg = 0.0f;
    float distanceM = 0.0f;
    std::int64_t timestampMs = 0;
};

class LidarManager
{
public:
    struct ScanPoint
    {
        float angleDeg = 0.0f;
        float distMm = 0.0f;
        int quality = 0;
        bool isSync = false;
    };

    using LogCallback = std::function<void(const std::string&)>;

public:
    LidarManager();
    ~LidarManager();

    LidarManager(const LidarManager&) = delete;
    LidarManager& operator=(const LidarManager&) = delete;

    bool initialize();
    void finalize();

    bool connectDevice(const char* port, sl_u32 baudrate);
    bool checkHealth();

    bool startScan();
    void stopScan();

    bool grabOneScan(std::vector<ScanPoint>& points);

    bool isInitialized() const;
    bool isConnected() const;
    bool isScanning() const;

    const std::string& lastError() const;
    void setLogCallback(LogCallback callback);

private:
    void log(const std::string& message) const;
    void setError(const std::string& message);

private:
    sl::ILidarDriver* m_drv = nullptr;
    bool m_inited = false;
    bool m_connected = false;
    bool m_scanning = false;

    std::string m_lastError;
    LogCallback m_logCallback;
};
