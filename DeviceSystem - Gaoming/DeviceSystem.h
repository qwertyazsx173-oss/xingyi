//DeviceSystem 继承 QMainWindow，作为主窗口，
//同时管理相机、雷达、检测线程、融合判断和检测阶段切换。
#pragma once

#include <QtWidgets/QMainWindow>    //QT主界面
#include <QImage>                   //处理图像
#include <QTimer>                   //周期性任务，定时更新雷达
#include <vector>                   
#include "ui_DeviceSystem.h"         //ui界面
#include "CameraManager.h"           //相机模块
#include "LidarManager.h"            //雷达模块
#include <QThread>                   //分线程
#include "DetectionWorker.h"         //检测模块



enum class DetectStage         //系统状态枚举
{
    FirstDetect,            // 第一次检测
    MovingToSecondPose,     // 正在去第二检测位
    SecondDetect,           // 第二次检测
    Finished                // 最终完成
};


class DeviceSystem : public QMainWindow
{
    Q_OBJECT

public:
    DeviceSystem(QWidget* parent = nullptr);
    ~DeviceSystem();

    //外部业务调用接口
public:
    //状态外部接口
    DetectStage currentStage() const;
    //保存图片按钮
    bool saveCurrentImage(const QString& filePath);
    //第一检测位置雷达扫描到的距离
    float firstDetectRadarDistanceCm() const;
    //重置任务
    void resetTask();
    //到达第二检测位置
    void notifyRobotReachedSecondPose();
    //当前帧图片
    QImage currentFrame() const;

signals:
    void fusedRingDetected(const CameraDetectionResult& camResult,
        const RadarDetectionResult& radarResult);   //融合检测完成，同时发送相机雷达数据

    void cameraFrameUpdated(const QImage& image);
    void firstFusionRadarDistanceUpdated(float distanceCm);

private slots:
    //相机新图像到来时，由这个函数进行接收
    void onFrameReady(const QImage& image);  
    //接收相机模块发过来的日志
    void onCameraLog(const QString& msg);
    //ui界面上的保存图片按钮
    void on_btnSaveImage_clicked();
    //取，更新，融合判断雷达数据
    void updateLidarRealtime();
    //检测线程完成之后  结果回传
    void onRingDetected(const CameraDetectionResult& result);

private:
    //用雷达扫描点刷新ui显示
    void updateLidarView(const std::vector<LidarManager::ScanPoint>& points);
    //雷达扫到物体时输出雷达显示结果
    bool detectRingFromLidar(const std::vector<LidarManager::ScanPoint>& points,
        RadarDetectionResult& result);
    //及时融合状态
    void updateFusionState();
    //稳定判断
    void updateFusionDetectionState();
    //计算图像中心坐标
    int getImageCenterX() const;
    int getImageCenterY() const;
   

    //返回第一次检测结果的距离（雷达）
    float getFirstDetectRadarDistanceCm() const;
    //清空融合状态
    void resetFusionStatus();
    //处理到达第二检测位
    void onRobotReachedSecondDetectPose();

private:
    Ui::DeviceSystemClass ui;
    CameraManager m_cameraManager;
    LidarManager m_lidarManager;

    QImage m_currentFrame;

    // 相机检测结果
    CameraDetectionResult m_lastRingResult;

    // 雷达检测结果
    RadarDetectionResult m_lastRadarResult;

    QTimer* m_lidarTimer = nullptr;
    QThread* m_detectionThread = nullptr;
    DetectionWorker* m_detectionWorker = nullptr;

    bool m_detectionBusy = false;

    // 原始融合瞬时状态
    bool m_lastFusedFound = false;

    // 稳定融合状态
    bool m_fusionStableFound = false;

    // 连续命中/丢失计数
    int m_fusionHitCount = 0;
    int m_fusionMissCount = 0;

    // 阈值
    int m_fusionHitThreshold = 2;
    int m_fusionMissThreshold = 3;

    // 联合检测弹窗只弹一次
    bool m_jobPopupShown = false;

    bool m_hideFusionStatusAfterAck = false;
    // 融合时间流对准阈值（毫秒）
    int m_fusionTimeToleranceMs = 1000;






    // 雷达工作距离    55cm
    float m_workDistance = 0.55f;

    // 距离误差        3cm
    float m_workDistanceTolerance = 0.03f;

    // 1cm对应像素个数 10个
    float m_pixelPerCm = 54.0f;

    // 相机到夹爪垂直差值
    float m_cameraToGripperOffsetCm = 14.0f;


    // 图像中心纵向容差（像素）
    int m_imageCenterToleranceY = 40;

    // 图像中心横向容差（像素）
    int m_imageCenterToleranceX = 120;

    bool m_workReadyStable = false;

    int m_cameraResultMaxAgeMs = 1200;

    double m_detectionScale = 0.5;
    qint64 m_lastFusionMatchTimeMs = 0;

    int m_radarResultMaxAgeMs = 300;
    int m_fusionHoldMs = 800;

    float m_gripperLengthCm = 25.0f;
    float m_valveLengthCm = 9.0f;
    float m_safetyReserveCm = 7.0f;

    float m_rawRadarTargetCm = 35.0f;          // 原始雷达目标距离
    float m_rawRadarToleranceCm = 5.0f;        // 原始雷达允许误差，可自己调

    qint64 m_lastRadarRangeHintTimeMs = 0;     // 上次提示时间
    int m_radarRangeHintIntervalMs = 6000;     // 每6秒提示一次





    DetectStage m_detectStage = DetectStage::FirstDetect;

    // 第一次检测记录的原始雷达距离
    float m_firstDetectRadarDistanceCm = 0.0f;

    // 固定升的位置
    float m_firstLiftUpCm = 60.0f;

    // 防止第一次检测成功后重复发机械臂指令
    bool m_secondPoseCommandSent = false;

    // 防止第二检测位补偿/夹取逻辑被重复触发
    bool m_stage2ActionSent = false;
};