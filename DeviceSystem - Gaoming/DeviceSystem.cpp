#include "DeviceSystem.h"
#include <QMessageBox>
#include <QPixmap>
#include <QDebug>
#include <QFileDialog>
#include <QDateTime>
#include <QtMath>
#include <QPainter>
#include <QPen>
#include <algorithm>
#include <QThread>
#include <cstdlib>
#include <QShortcut>

//程序启动
DeviceSystem::DeviceSystem(QWidget* parent)
    : QMainWindow(parent)
{
    ui.setupUi(this);

    //按键p用来模拟到达第二检测位置
    QShortcut* scP = new QShortcut(QKeySequence(Qt::Key_P), this);
    scP->setContext(Qt::ApplicationShortcut);

    connect(scP, &QShortcut::activated, this, [this]()
        {
            qDebug() << "[Test] P pressed, force switch to second detect pose.";
            onRobotReachedSecondDetectPose();
        });



    // -------- 相机图像显示 --------
    //相机画面连接到主界面
    connect(&m_cameraManager, &CameraManager::frameReady,
        this, &DeviceSystem::onFrameReady);
    //相机日志
    connect(&m_cameraManager, &CameraManager::logMessage,
        this, &DeviceSystem::onCameraLog);

    // -------- 检测线程 --------
    m_detectionThread = new QThread(this);
    m_detectionWorker = new DetectionWorker();
    m_detectionWorker->moveToThread(m_detectionThread);

    // 检测结果回到主界面
    connect(m_detectionWorker, &DetectionWorker::ringDetected,
        this, &DeviceSystem::onRingDetected,
        Qt::QueuedConnection);

    connect(m_detectionWorker, &DetectionWorker::logMessage,
        this, &DeviceSystem::onCameraLog,
        Qt::QueuedConnection);

    connect(m_detectionThread, &QThread::finished,
        m_detectionWorker, &QObject::deleteLater);

    m_detectionThread->start();
    QMetaObject::invokeMethod(
        m_detectionWorker,
        "setDetectStage",
        Qt::QueuedConnection,
        Q_ARG(int, static_cast<int>(DetectStage::FirstDetect))
    );

    bool cameraOk = false;
    bool lidarOk = false;

    // -------- 相机测试 --------
    if (!m_cameraManager.initialize())
    {
        QMessageBox::warning(this, "Camera", "Camera SDK init failed");
    }
    else if (!m_cameraManager.enumDevices())
    {
        QMessageBox::warning(this, "Camera", "No camera found");
    }
    else if (!m_cameraManager.openFirstCamera())
    {
        QMessageBox::warning(this, "Camera", "Open camera failed");
    }
    else
    {
        cameraOk = true;
        QMessageBox::information(this, "Camera", "Camera opened successfully");

        m_cameraManager.startCapture();
    }

    // -------- 雷达测试 --------
    if (!m_lidarManager.initialize())
    {
        QMessageBox::warning(this, "Lidar", "Lidar init failed");
    }
    else if (!m_lidarManager.connectDevice("COM5", 256000))
    {
        QMessageBox::warning(this, "Lidar", "Lidar connect failed");
    }
    else if (!m_lidarManager.checkHealth())
    {
        QMessageBox::warning(this, "Lidar", "Lidar health check failed");
    }
    else if (!m_lidarManager.startScan())
    {
        QMessageBox::warning(this, "Lidar", "Lidar start scan failed");
    }
    else
    {
        lidarOk = true;
        QMessageBox::information(this, "Lidar", "Lidar scan started");

        m_lidarTimer = new QTimer(this);
        connect(m_lidarTimer, &QTimer::timeout,
            this, &DeviceSystem::updateLidarRealtime);

        m_lidarTimer->start(100);
    }
}

DeviceSystem::~DeviceSystem()
{
    if (m_detectionThread)
    {
        m_detectionThread->quit();
        m_detectionThread->wait();
    }

    if (m_lidarTimer)
        m_lidarTimer->stop();

    m_cameraManager.stopCapture();
    m_cameraManager.closeCamera();
    m_cameraManager.finalize();

    m_lidarManager.stopScan();
    m_lidarManager.finalize();
}
//系统接受图片并送检
void DeviceSystem::onFrameReady(const QImage& image)
{
    if (image.isNull())
        return;
    //只保存最新画面
    m_currentFrame = image.copy();

    // 检测线程空闲时，才送一帧过去
    if (!m_detectionBusy)
    {
        m_detectionBusy = true;

        QImage detectImage = m_currentFrame.scaled(
            static_cast<int>(m_currentFrame.width() * m_detectionScale),
            static_cast<int>(m_currentFrame.height() * m_detectionScale),
            Qt::KeepAspectRatio,
            Qt::FastTransformation
        );

        QMetaObject::invokeMethod(
            m_detectionWorker,
            "processFrame",
            Qt::QueuedConnection,
            Q_ARG(QImage, detectImage)
        );
    }

    // 原图尺寸
    int imgW = image.width();
    int imgH = image.height();

    QPixmap scaledPix = QPixmap::fromImage(image).scaled(
        ui.labelCamera->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation
    );

    QPixmap drawPix = scaledPix.copy();
    QPainter painter(&drawPix);
    painter.setRenderHint(QPainter::Antialiasing, true);

    if (m_lastRingResult.found)
    {
        int drawW = drawPix.width();
        int drawH = drawPix.height();

        double sx = static_cast<double>(drawW) / imgW;
        double sy = static_cast<double>(drawH) / imgH;

        int x = static_cast<int>(m_lastRingResult.bboxX * sx);
        int y = static_cast<int>(m_lastRingResult.bboxY * sy);
        int w = static_cast<int>(m_lastRingResult.bboxW * sx);
        int h = static_cast<int>(m_lastRingResult.bboxH * sy);

        int cx = static_cast<int>(m_lastRingResult.centerX * sx);
        int cy = static_cast<int>(m_lastRingResult.centerY * sy);

        painter.setPen(QPen(Qt::red, 2));
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(x, y, w, h);

        painter.setBrush(Qt::red);
        painter.drawEllipse(QPoint(cx, cy), 4, 4);

        painter.setPen(QPen(Qt::yellow, 2));
        painter.drawText(x, std::max(20, y - 8), "Current Ring");
    }

    ui.labelCamera->setPixmap(drawPix);
    emit cameraFrameUpdated(m_currentFrame);
}

//相机或检测线程发来的日志打印到调试输出
void DeviceSystem::onCameraLog(const QString& msg)
{
    qDebug() << "[Camera]" << msg;
}

//保存图像
void DeviceSystem::on_btnSaveImage_clicked()
{
    if (m_currentFrame.isNull())
    {
        QMessageBox::warning(this, "提示", "当前没有可保存的图像！");
        return;
    }

    QString defaultName = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".png";

    QString fileName = QFileDialog::getSaveFileName(
        this,
        "保存图片",
        defaultName,
        "Image Files (*.png *.jpg *.bmp)"
    );

    if (fileName.isEmpty())
        return;

    if (!saveCurrentImage(fileName))
    {
        QMessageBox::warning(this, "错误", "图片保存失败！");
        return;
    }

    QMessageBox::information(this, "提示", "图片保存成功！");
}

//采样提取更新融合
void DeviceSystem::updateLidarRealtime()
{
    std::vector<LidarManager::ScanPoint> points;
    if (m_lidarManager.grabOneScan(points))   //采样
    {

        RadarDetectionResult radarResult;
        //提取
        detectRingFromLidar(points, radarResult);
        m_lastRadarResult = radarResult;
        //刷新画面
        updateLidarView(points);
        //融合判断
        updateFusionState();
    }
}

//绘制雷达视图
void DeviceSystem::updateLidarView(const std::vector<LidarManager::ScanPoint>& points)
{
    if (ui.labelLidar->width() <= 0 || ui.labelLidar->height() <= 0)
        return;

    QImage canvas(ui.labelLidar->size(), QImage::Format_RGB888);
    canvas.fill(Qt::black);

    QPainter painter(&canvas);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const int w = canvas.width();
    const int h = canvas.height();
    const int CENTER_X = w / 2;
    const int CENTER_Y = h / 2;

    // ===== 按你原来的思路保留这些参数 =====
    const float SCALE_PIXELS_PER_METER = 60.0f;   // 先用 60，后面可调
    const float MAX_DISTANCE = 5.0f;              // 最大显示 5 米
    const float MIN_VALID_DIST = 0.20f;           // 盲区 20cm
    const float SAFE_DIST_THRESHOLD = 0.50f;      // 危险阈值 50cm
    const float TOOL_LENGTH_M = (m_gripperLengthCm + m_valveLengthCm) / 100.0f;   // UI只减夹爪+电磁阀

    // ========== 1. 画背景坐标 ==========
    painter.setPen(QPen(QColor(80, 80, 80), 1));
    painter.drawLine(0, CENTER_Y, w, CENTER_Y);
    painter.drawLine(CENTER_X, 0, CENTER_X, h);

    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(QColor(120, 120, 120), 1, Qt::DotLine));

    for (int r = 1; r <= 5; ++r)
    {
        int radiusPx = static_cast<int>(r * SCALE_PIXELS_PER_METER);
        painter.drawEllipse(QPoint(CENTER_X, CENTER_Y), radiusPx, radiusPx);

        painter.setPen(QPen(QColor(150, 150, 150), 1));
        painter.drawText(CENTER_X + 8, CENTER_Y - radiusPx, QString("%1m").arg(r));
        painter.setPen(QPen(QColor(120, 120, 120), 1, Qt::DotLine));
    }

    // ========== 2. 画机械臂自身盲区 ==========
    int blindRadiusPixels = static_cast<int>(MIN_VALID_DIST * SCALE_PIXELS_PER_METER);
    painter.setPen(QPen(QColor(180, 180, 180), 1));
    painter.setBrush(QColor(60, 60, 60));
    painter.drawEllipse(QPoint(CENTER_X, CENTER_Y), blindRadiusPixels, blindRadiusPixels);

    painter.setPen(QPen(QColor(220, 220, 220), 1));
    painter.drawText(CENTER_X - 55, CENTER_Y - 8, QString("Robot Body"));

    // ========== 3. 转换点云 ==========
    struct DrawPoint
    {
        int x;
        int y;
        float rawDist;
        float usableDist;
        float angle;
    };

    std::vector<DrawPoint> drawPoints;
    drawPoints.reserve(points.size());

    for (const auto& pt : points)
    {
        // 你现在 grabOneScan 里的 distMm 是毫米，这里转米
        float rawDistMeter = pt.distMm / 1000.0f;
        float usableDistMeter = rawDistMeter - TOOL_LENGTH_M;
        float angleDeg = pt.angleDeg;

        if (rawDistMeter <= MIN_VALID_DIST || rawDistMeter > MAX_DISTANCE)
            continue;

        if (usableDistMeter < 0.0f)
            usableDistMeter = 0.0f;

        float rad = qDegreesToRadians(angleDeg);

        int x = CENTER_X + static_cast<int>(rawDistMeter * std::cos(rad) * SCALE_PIXELS_PER_METER);
        int y = CENTER_Y - static_cast<int>(rawDistMeter * std::sin(rad) * SCALE_PIXELS_PER_METER);

        if (x >= 0 && x < w && y >= 0 && y < h)
        {
            drawPoints.push_back({ x, y, rawDistMeter, usableDistMeter, angleDeg });
        }
    }

    // ========== 4. 按角度排序，方便连线 ==========
    std::sort(drawPoints.begin(), drawPoints.end(),
        [](const DrawPoint& a, const DrawPoint& b)
        {
            return a.angle < b.angle;
        });

    // ========== 5. 画环境点 ==========
    for (const auto& p : drawPoints)
    {
        QColor color = Qt::green;
        if (p.usableDist < SAFE_DIST_THRESHOLD)
            color = Qt::red;
        else if (p.usableDist < 1.5f)
            color = Qt::yellow;

        painter.setPen(Qt::NoPen);
        painter.setBrush(color);
        painter.drawEllipse(QPoint(p.x, p.y), 2, 2);
    }

    // ========== 6. 连轮廓线（按你旧逻辑的简化版） ==========
    painter.setPen(QPen(Qt::white, 1));

    const float MAX_CONNECT_ANGLE_DIFF = 6.0f;
    const float MAX_CONNECT_DIST_DIFF = 0.5f;

    int n = static_cast<int>(drawPoints.size());
    if (n > 2)
    {
        for (int i = 0; i < n; ++i)
        {
            const auto& p1 = drawPoints[i];
            const auto& p2 = drawPoints[(i + 1) % n];

            float angleDiff = std::fabs(p2.angle - p1.angle);
            if (angleDiff > 180.0f)
                angleDiff = 360.0f - angleDiff;

            float distDiff = std::fabs(p2.rawDist - p1.rawDist);

            if (angleDiff < MAX_CONNECT_ANGLE_DIFF && distDiff < MAX_CONNECT_DIST_DIFF)
            {
                painter.drawLine(p1.x, p1.y, p2.x, p2.y);
            }
        }
    }

    // ========== 7. 找最近的 3 个点 ==========
    std::vector<DrawPoint> nearestPoints = drawPoints;
    std::sort(nearestPoints.begin(), nearestPoints.end(),
        [](const DrawPoint& a, const DrawPoint& b)
        {
            return a.usableDist < b.usableDist;
        });

    if (nearestPoints.size() > 3)
        nearestPoints.resize(3);

    painter.setPen(QPen(Qt::red, 1));
    painter.setBrush(Qt::red);

    for (const auto& p : nearestPoints)
    {
        painter.drawLine(CENTER_X, CENTER_Y, p.x, p.y);
        painter.drawEllipse(QPoint(p.x, p.y), 4, 4);
        painter.drawText(p.x + 8, p.y - 8, QString("%1m").arg(p.usableDist, 0, 'f', 2));
    }

    // ========== 8. 左上角显示统计 ==========
    painter.setPen(QPen(Qt::green, 1));
    painter.drawText(10, 20, QString("Points: %1").arg(drawPoints.size()));

    painter.setPen(QPen(Qt::white, 1));
    painter.drawText(10, 40, "=== Closest Obstacles ===");

    if (nearestPoints.empty())
    {
        painter.setPen(QPen(Qt::lightGray, 1));
        painter.drawText(10, 65, "No obstacles detected.");
    }
    else
    {
        for (int i = 0; i < static_cast<int>(nearestPoints.size()); ++i)
        {
            const auto& p = nearestPoints[i];
            if (p.usableDist < SAFE_DIST_THRESHOLD)
                painter.setPen(QPen(Qt::red, 1));
            else
                painter.setPen(QPen(Qt::green, 1));

            painter.drawText(
                10,
                65 + i * 20,
                QString("#%1: %2 m")
                .arg(i + 1)
                .arg(p.usableDist, 0, 'f', 2)
            );
        }
    }

    ui.labelLidar->setPixmap(QPixmap::fromImage(canvas));
}

//视觉检测的回调线程
void DeviceSystem::onRingDetected(const CameraDetectionResult& result)
{
    m_lastRingResult = result;
    m_detectionBusy = false;

    if (m_detectStage == DetectStage::FirstDetect &&
        result.found &&
        result.centerY < m_currentFrame.height() / 3)
    {
        m_lastRingResult.found = false;
    }

    qDebug() << "[RingDetected]"
        << "found =" << m_lastRingResult.found
        << "bbox =" << m_lastRingResult.bboxX << m_lastRingResult.bboxY
        << m_lastRingResult.bboxW << m_lastRingResult.bboxH
        << "center =" << m_lastRingResult.centerX << m_lastRingResult.centerY;

    updateFusionState();
}

//雷达目标提取
bool DeviceSystem::detectRingFromLidar(const std::vector<LidarManager::ScanPoint>& points,
    RadarDetectionResult& result)
{
    result = RadarDetectionResult();
    result.timestamp = QDateTime::currentMSecsSinceEpoch();

    float angleLimitDeg = 8.0f;
    float minDistM = 0.20f;
    float maxDistM = 0.80f;

    if (m_detectStage == DetectStage::SecondDetect)
    {
        angleLimitDeg = 12.0f;
        minDistM = 0.15f;
        maxDistM = 1.00f;
    }

    float bestDistM = 1e9f;
    bool found = false;

    for (const auto& pt : points)
    {
        float distM = pt.distMm / 1000.0f;
        float angle = pt.angleDeg;

        if (std::fabs(angle) > angleLimitDeg)
            continue;

        if (distM < minDistM || distM > maxDistM)
            continue;

        if (pt.quality <= 0)
            continue;

        if (distM < bestDistM)
        {
            bestDistM = distM;
            result.found = true;
            result.angleDeg = angle;
            result.distanceM = distM;
            found = true;
        }
    }

    return found;
}

//取当前位置图像中心位置
int DeviceSystem::getImageCenterX() const
{
    if (m_currentFrame.isNull())
        return 0;

    return m_currentFrame.width() / 2;
}
int DeviceSystem::getImageCenterY() const
{
    if (m_currentFrame.isNull())
        return 0;

    return m_currentFrame.height() / 2;
}

//用来获取第一次融合检测位置的雷达距离
float DeviceSystem::getFirstDetectRadarDistanceCm() const
{
    if (!m_lastRadarResult.found)
        return 0.0f;

    return m_lastRadarResult.distanceM * 100.0f;

}

//联合检测
void DeviceSystem::updateFusionDetectionState()
{
    bool fusedFound = false;
    qint64 dt = -1;

    qint64 nowMs = QDateTime::currentMSecsSinceEpoch();

    qint64 camAgeMs = (m_lastRingResult.timestamp > 0)
        ? (nowMs - m_lastRingResult.timestamp) : 999999;

    qint64 radarAgeMs = (m_lastRadarResult.timestamp > 0)
        ? (nowMs - m_lastRadarResult.timestamp) : 999999;

    bool camFreshEnough =
        m_lastRingResult.found &&
        m_lastRingResult.timestamp > 0 &&
        (camAgeMs <= m_cameraResultMaxAgeMs);

    bool radarFreshEnough =
        m_lastRadarResult.found &&
        m_lastRadarResult.timestamp > 0 &&
        (radarAgeMs <= m_radarResultMaxAgeMs);

    if (camFreshEnough && radarFreshEnough)
    {
        dt = std::llabs(m_lastRingResult.timestamp - m_lastRadarResult.timestamp);

        if (dt <= m_fusionTimeToleranceMs)
        {
            fusedFound = true;
            m_lastFusionMatchTimeMs = nowMs;
        }
        else
        {
            if (nowMs - m_lastFusionMatchTimeMs <= m_fusionHoldMs)
                fusedFound = true;
        }
    }

    m_lastFusedFound = fusedFound;

    if (fusedFound)
    {
        m_fusionHitCount++;
        m_fusionMissCount = 0;
    }
    else
    {
        m_fusionMissCount++;
        m_fusionHitCount = 0;
    }

    // ---------- 第一次检测 ----------
    if (m_detectStage == DetectStage::FirstDetect)
    {
        if (!m_fusionStableFound && m_fusionHitCount >= m_fusionHitThreshold)
        {
            m_fusionStableFound = true;
            qDebug() << "[Fusion][Stage1] STABLE FUSED DETECTED";
        }

        // 第一阶段这里也要卡 fusedFound，不能只看相机 found
        if (m_fusionStableFound && fusedFound && !m_secondPoseCommandSent)
        {
            int imgCenterX = getImageCenterX();
            int imgCenterY = getImageCenterY();

            int dx = std::abs(m_lastRingResult.centerX - imgCenterX);
            int dy = std::abs(m_lastRingResult.centerY - imgCenterY);

            // 第一阶段仍然只看上下方向
            bool centeredReady = (dy <= m_imageCenterToleranceY);

            qDebug() << "[CenterCheck][Stage1]"
                << "ringCenter =" << m_lastRingResult.centerX << m_lastRingResult.centerY
                << "imageCenter =" << imgCenterX << imgCenterY
                << "dx =" << dx
                << "dy =" << dy
                << "tolY =" << m_imageCenterToleranceY
                << "centeredReady =" << centeredReady;

            if (centeredReady)
            {
                m_secondPoseCommandSent = true;

                // 这里直接用当前新鲜雷达数据，不再调旧 helper
                m_firstDetectRadarDistanceCm = m_lastRadarResult.distanceM * 100.0f;
                emit firstFusionRadarDistanceUpdated(m_firstDetectRadarDistanceCm);

                m_detectStage = DetectStage::MovingToSecondPose;

                QMessageBox::information(
                    this,
                    QStringLiteral("第一次检测完成"),
                    QStringLiteral("已到第一检测位。\n\n"
                        "请升降车停止，并前往第二检测位。")
                );

                qDebug() << "[Stage1 -> Stage2]"
                    << "first_radar_distance_cm =" << m_firstDetectRadarDistanceCm
                    << "next stage = MovingToSecondPose";
            }
        }
    }
    // ---------- 第二次检测 ----------
    else if (m_detectStage == DetectStage::SecondDetect)
    {
        if (!m_fusionStableFound && m_fusionHitCount >= m_fusionHitThreshold)
        {
            m_fusionStableFound = true;
            qDebug() << "[Fusion][Stage2] STABLE FUSED DETECTED";
        }

        // 第二阶段必须卡当前新鲜融合成功
        if (m_fusionStableFound && fusedFound)
        {
            int imgCenterX = getImageCenterX();
            int imgCenterY = getImageCenterY();

            int dx = m_lastRingResult.centerX - imgCenterX;
            int dy = m_lastRingResult.centerY - imgCenterY;

            bool centeredReady =
                (std::abs(dy) <= m_imageCenterToleranceY);

            qDebug() << "[CenterCheck][Stage2]"
                << "ringCenter =" << m_lastRingResult.centerX << m_lastRingResult.centerY
                << "imageCenter =" << imgCenterX << imgCenterY
                << "dx =" << dx
                << "dy =" << dy
                << "centeredReady =" << centeredReady;

            if (centeredReady && !m_stage2ActionSent)
            {
                m_stage2ActionSent = true;
                qDebug() << "[Stage2] fused and centered, ready for compensation/grasp.";

                // 这里以后接你的第二检测位补偿/夹取逻辑
            }
        }
    }

    if (m_fusionStableFound && m_fusionMissCount >= m_fusionMissThreshold)
    {
        m_fusionStableFound = false;
        m_jobPopupShown = false;

        // 只在第一阶段丢失时，重新允许第一阶段再次触发
        if (m_detectStage == DetectStage::FirstDetect)
            m_secondPoseCommandSent = false;

        qDebug() << "[Fusion] STABLE LOST";
    }

    qDebug() << "[FusionCheck]"
        << "stage =" << static_cast<int>(m_detectStage)
        << "instant =" << fusedFound
        << "stableFusion =" << m_fusionStableFound
        << "hitCount =" << m_fusionHitCount
        << "missCount =" << m_fusionMissCount
        << "dtMs =" << dt
        << "camAgeMs =" << camAgeMs
        << "radarAgeMs =" << radarAgeMs
        << "camFound =" << m_lastRingResult.found
        << "radarFound =" << m_lastRadarResult.found;
}

//状态判断
void DeviceSystem::updateFusionState()
{
    updateFusionDetectionState();

    if (m_detectStage == DetectStage::FirstDetect)
    {
        if (!m_fusionStableFound)
        {
            setWindowTitle(QStringLiteral("DeviceSystem - 第一次检测中"));
        }
        else
        {
            int dy = std::abs(m_lastRingResult.centerY - getImageCenterY());

            if (dy <= m_imageCenterToleranceY)
                setWindowTitle(QStringLiteral("DeviceSystem - 已到第一检测位"));
            else
                setWindowTitle(QStringLiteral("DeviceSystem - 已检测到均压环，等待第一检测位居中"));
        }
    }
    else if (m_detectStage == DetectStage::MovingToSecondPose)
    {
        setWindowTitle(QStringLiteral("DeviceSystem - 正在前往第二检测位"));
    }
    else if (m_detectStage == DetectStage::SecondDetect)
    {
        if (!m_fusionStableFound)
        {
            setWindowTitle(QStringLiteral("DeviceSystem - 第二次检测中"));
        }
        else
        {
            int dx = std::abs(m_lastRingResult.centerX - getImageCenterX());
            int dy = std::abs(m_lastRingResult.centerY - getImageCenterY());

            int dyMove = m_lastRingResult.centerY - getImageCenterY();

            if (dy <= m_imageCenterToleranceY)
                setWindowTitle(QStringLiteral("DeviceSystem - 第二检测位已融合并居中"));
            else
                if(dyMove>0)
                    setWindowTitle(QStringLiteral("DeviceSystem - 第二检测位已融合，等待居中，需向近端移动"));
                else
                    setWindowTitle(QStringLiteral("DeviceSystem - 第二检测位已融合，等待居中，需向远端移动"));
        }
    }
    else
    {
        setWindowTitle(QStringLiteral("DeviceSystem - 已完成"));
    }
}

//状态清零
void DeviceSystem::resetFusionStatus()
{
    m_lastFusedFound = false;
    m_fusionStableFound = false;
    m_fusionHitCount = 0;
    m_fusionMissCount = 0;
    m_lastFusionMatchTimeMs = 0;
    m_jobPopupShown = false;
    m_stage2ActionSent = false;
}

//第二检测位置的之后的动作
void DeviceSystem::onRobotReachedSecondDetectPose()
{
    qDebug() << "[Stage] Robot reached second detect pose.";

    m_detectStage = DetectStage::SecondDetect;
    m_secondPoseCommandSent = false;

    resetFusionStatus();

    // 清空第一阶段残留结果，强制第二阶段等待新数据
    m_lastRingResult = CameraDetectionResult();
    m_lastRadarResult = RadarDetectionResult();
    m_lastRingResult.timestamp = 0;
    m_lastRadarResult.timestamp = 0;
    m_lastRingResult.found = false;
    m_lastRadarResult.found = false;

    QMetaObject::invokeMethod(
        m_detectionWorker,
        "setDetectStage",
        Qt::QueuedConnection,
        Q_ARG(int, static_cast<int>(m_detectStage))
    );

    QMessageBox::information(
        this,
        QStringLiteral("提示"),
        QStringLiteral("机械臂已到达第二检测位，开始进行第二次检测。")
    );
}


//外接接口
DetectStage DeviceSystem::currentStage() const
{
    return m_detectStage;
}

QImage DeviceSystem::currentFrame() const
{
    return m_currentFrame;
}

bool DeviceSystem::saveCurrentImage(const QString& filePath)
{
    if (m_currentFrame.isNull())
        return false;

    if (filePath.trimmed().isEmpty())
        return false;

    return m_currentFrame.save(filePath);
}

float DeviceSystem::firstDetectRadarDistanceCm() const
{
    return m_firstDetectRadarDistanceCm;
}

void DeviceSystem::resetTask()
{
    m_detectStage = DetectStage::FirstDetect;

    m_detectionBusy = false;
    m_secondPoseCommandSent = false;
    m_stage2ActionSent = false;
    m_workReadyStable = false;
    m_hideFusionStatusAfterAck = false;
    m_lastRadarRangeHintTimeMs = 0;

    m_firstDetectRadarDistanceCm = 0.0f;

    resetFusionStatus();

    m_currentFrame = QImage();
    m_lastRingResult = CameraDetectionResult();
    m_lastRadarResult = RadarDetectionResult();

    m_lastRingResult.timestamp = 0;
    m_lastRadarResult.timestamp = 0;
    m_lastRingResult.found = false;
    m_lastRadarResult.found = false;

    if (m_detectionWorker)
    {
        QMetaObject::invokeMethod(
            m_detectionWorker,
            "setDetectStage",
            Qt::QueuedConnection,
            Q_ARG(int, static_cast<int>(DetectStage::FirstDetect))
        );
    }

    ui.labelCamera->clear();
    setWindowTitle(QStringLiteral("DeviceSystem - 第一次检测中"));
}

void DeviceSystem::notifyRobotReachedSecondPose()
{
    onRobotReachedSecondDetectPose();
}