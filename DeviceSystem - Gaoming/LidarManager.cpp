#include "LidarManager.h"

using namespace sl;

LidarManager::LidarManager(QObject* parent)
    : QObject(parent),
    m_drv(nullptr),
    m_inited(false),
    m_connected(false),
    m_scanning(false)
{
}

LidarManager::~LidarManager()
{
    finalize();
}

bool LidarManager::initialize()
{
    if (m_inited)
        return true;

    m_drv = *createLidarDriver();
    if (!m_drv)
        return false;

    m_inited = true;
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
}

bool LidarManager::connectDevice(const char* port, sl_u32 baudrate)
{
    if (!m_inited || !m_drv)
        return false;

    sl::IChannel* channel = *sl::createSerialPortChannel(port, baudrate);
    if (!channel)
    {
        emit logMessage(QString("创建雷达串口通道失败: %1, baud=%2").arg(port).arg(baudrate));
        return false;
    }

    sl_result op_result = m_drv->connect(channel);
    if (SL_IS_FAIL(op_result))
    {
        emit logMessage(QString("雷达连接失败: %1, baud=%2").arg(port).arg(baudrate));
        return false;
    }

    m_connected = true;
    emit logMessage(QString("雷达连接成功: %1, baud=%2").arg(port).arg(baudrate));
    return true;
}

bool LidarManager::checkHealth()
{
    if (!m_connected || !m_drv)
        return false;

    sl_lidar_response_device_health_t healthinfo;
    sl_result op_result = m_drv->getHealth(healthinfo);

    if (SL_IS_FAIL(op_result))
    {
        emit logMessage("获取雷达健康状态失败");
        return false;
    }

    if (healthinfo.status == SL_LIDAR_STATUS_OK)
    {
        emit logMessage("雷达健康状态正常");
        return true;
    }

    emit logMessage(QString("雷达健康状态异常, status=%1").arg((int)healthinfo.status));
    return false;
}

bool LidarManager::startScan()
{
    if (!m_connected || !m_drv)
        return false;

    if (m_scanning)
        return true;

    m_drv->setMotorSpeed();

    sl_result op_result = m_drv->startScan(0, 1);
    if (SL_IS_FAIL(op_result))
    {
        emit logMessage("启动雷达扫描失败");
        return false;
    }

    m_scanning = true;
    emit logMessage("雷达开始扫描");
    return true;
}

void LidarManager::stopScan()
{
    if (!m_drv || !m_scanning)
        return;

    m_drv->stop();
    m_drv->setMotorSpeed(0);

    m_scanning = false;
    emit logMessage("雷达停止扫描");
}

bool LidarManager::grabOneScan(std::vector<ScanPoint>& points)
{
    points.clear();

    if (!m_drv || !m_scanning)
        return false;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);

    sl_result op_result = m_drv->grabScanDataHq(nodes, count);
    if (SL_IS_FAIL(op_result))
    {
        emit logMessage("获取雷达扫描数据失败");
        return false;
    }

    m_drv->ascendScanData(nodes, count);

    points.reserve(count);

    for (size_t i = 0; i < count; ++i)
    {
        ScanPoint pt;
        pt.isSync = (nodes[i].flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) != 0;
        pt.angleDeg = (nodes[i].angle_z_q14 * 90.f) / 16384.f;
        pt.distMm = nodes[i].dist_mm_q2 / 4.0f;
        pt.quality = nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;

        points.push_back(pt);
    }

    return true;
}