#include "LidarManager.h"

#include <sstream>

namespace
{
std::string makeStatusMessage(const std::string& prefix, int code)
{
    std::ostringstream oss;
    oss << prefix << ", code = 0x" << std::hex << code;
    return oss.str();
}
} // namespace

LidarManager::LidarManager() = default;

LidarManager::~LidarManager()
{
    finalize();
}

bool LidarManager::initialize()
{
    if (m_inited)
    {
        return true;
    }

    m_drv = *sl::createLidarDriver();
    if (!m_drv)
    {
        setError("Failed to create lidar driver");
        return false;
    }

    m_inited = true;
    log("Lidar driver initialized");
    return true;
}

void LidarManager::finalize()
{
    stopScan();

    if (m_drv)
    {
        delete m_drv;
        m_drv = nullptr;
    }

    m_inited = false;
    m_connected = false;
    m_scanning = false;

    log("Lidar driver finalized");
}

bool LidarManager::connectDevice(const char* port, sl_u32 baudrate)
{
    if (!m_inited || !m_drv)
    {
        setError("Cannot connect lidar before initialization");
        return false;
    }

    if (!port || port[0] == '\0')
    {
        setError("Lidar serial port is empty");
        return false;
    }

    sl::IChannel* channel = *sl::createSerialPortChannel(port, baudrate);
    if (!channel)
    {
        setError(std::string("Failed to create lidar serial channel: ") + port +
                 ", baud=" + std::to_string(baudrate));
        return false;
    }

    const sl_result opResult = m_drv->connect(channel);
    if (SL_IS_FAIL(opResult))
    {
        setError(makeStatusMessage(
            std::string("Lidar connect failed: ") + port + ", baud=" + std::to_string(baudrate),
            static_cast<int>(opResult)));
        return false;
    }

    m_connected = true;
    log(std::string("Lidar connected successfully: ") + port +
        ", baud=" + std::to_string(baudrate));
    return true;
}

bool LidarManager::checkHealth()
{
    if (!m_connected || !m_drv)
    {
        setError("Cannot read lidar health before connection");
        return false;
    }

    sl_lidar_response_device_health_t healthInfo = {};
    const sl_result opResult = m_drv->getHealth(healthInfo);
    if (SL_IS_FAIL(opResult))
    {
        setError(makeStatusMessage("Failed to get lidar health", static_cast<int>(opResult)));
        return false;
    }

    if (healthInfo.status == SL_LIDAR_STATUS_OK)
    {
        log("Lidar health is OK");
        return true;
    }

    setError("Lidar health is abnormal, status=" + std::to_string(static_cast<int>(healthInfo.status)));
    return false;
}

bool LidarManager::startScan()
{
    if (!m_connected || !m_drv)
    {
        setError("Cannot start lidar scan before connection");
        return false;
    }

    if (m_scanning)
    {
        return true;
    }

    m_drv->setMotorSpeed();

    const sl_result opResult = m_drv->startScan(0, 1);
    if (SL_IS_FAIL(opResult))
    {
        setError(makeStatusMessage("Failed to start lidar scan", static_cast<int>(opResult)));
        return false;
    }

    m_scanning = true;
    log("Lidar scan started");
    return true;
}

void LidarManager::stopScan()
{
    if (!m_drv || !m_scanning)
    {
        return;
    }

    m_drv->stop();
    m_drv->setMotorSpeed(0);
    m_scanning = false;

    log("Lidar scan stopped");
}

bool LidarManager::grabOneScan(std::vector<ScanPoint>& points)
{
    points.clear();

    if (!m_drv || !m_scanning)
    {
        setError("Cannot grab lidar scan before scan start");
        return false;
    }

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = sizeof(nodes) / sizeof(nodes[0]);

    const sl_result opResult = m_drv->grabScanDataHq(nodes, count);
    if (SL_IS_FAIL(opResult))
    {
        setError(makeStatusMessage("Failed to grab lidar scan data", static_cast<int>(opResult)));
        return false;
    }

    m_drv->ascendScanData(nodes, count);

    points.reserve(count);
    for (size_t i = 0; i < count; ++i)
    {
        ScanPoint pt;
        pt.isSync = (nodes[i].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) != 0;
        pt.angleDeg = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
        pt.distMm = nodes[i].dist_mm_q2 / 4.0f;
        pt.quality = nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
        points.push_back(pt);
    }

    return true;
}

bool LidarManager::isInitialized() const
{
    return m_inited;
}

bool LidarManager::isConnected() const
{
    return m_connected;
}

bool LidarManager::isScanning() const
{
    return m_scanning;
}

const std::string& LidarManager::lastError() const
{
    return m_lastError;
}

void LidarManager::setLogCallback(LogCallback callback)
{
    m_logCallback = std::move(callback);
}

void LidarManager::log(const std::string& message) const
{
    if (m_logCallback)
    {
        m_logCallback(message);
    }
}

void LidarManager::setError(const std::string& message)
{
    m_lastError = message;
    log(message);
}
