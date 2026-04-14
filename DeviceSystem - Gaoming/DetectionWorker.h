#pragma once

#include <QObject>
#include <QImage>
#include <QDateTime>
#include <QString>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

struct CameraDetectionResult
{
    bool found = false;

    int centerX = -1;
    int centerY = -1;

    int bboxX = 0;
    int bboxY = 0;
    int bboxW = 0;
    int bboxH = 0;

    int radius = 0;

    qint64 timestamp = 0;
};

Q_DECLARE_METATYPE(CameraDetectionResult)

class DetectionWorker : public QObject
{
    Q_OBJECT

public:
    explicit DetectionWorker(QObject* parent = nullptr);

public slots:
    void processFrame(const QImage& image);
    void setDetectStage(int stage);

signals:
    void ringDetected(const CameraDetectionResult& result);
    void logMessage(const QString& msg);

private:
    cv::Mat qimageToMat(const QImage& image);
    cv::Mat letterboxSquare(const cv::Mat& src);

    bool loadModel(const QString& modelPath, cv::dnn::Net& net, const QString& tag);
    bool detectWithNet(const QImage& image,
        CameraDetectionResult& result,
        cv::dnn::Net& net,
        float confThreshold,
        float nmsThreshold,
        const QString& tag);

private:
    cv::dnn::Net m_netStage1;
    cv::dnn::Net m_netStage2;

    bool m_stage1Loaded = false;
    bool m_stage2Loaded = false;

    int m_currentStage = 0;   // 0=FirstDetect, 2=SecondDetect

    int m_inputWidth = 640;
    int m_inputHeight = 640;

    double m_outputScaleBack = 2.0;

    float m_confThresholdStage1 = 0.5f;
    float m_nmsThresholdStage1 = 0.45f;

    float m_confThresholdStage2 = 0.2f;
    float m_nmsThresholdStage2 = 0.45f;
};