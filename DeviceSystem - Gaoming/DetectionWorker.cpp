#include "DetectionWorker.h"
#include <QDebug>
#include <QDateTime>
#include <exception>

DetectionWorker::DetectionWorker(QObject* parent)
    : QObject(parent)
{
    qRegisterMetaType<CameraDetectionResult>("CameraDetectionResult");

    QString modelPathStage1 = "D:/Tempt/ring_dataset/runs/ring_yolov8n3/weights/best.onnx";
    QString modelPathStage2 = "D:/Tempt/two_detection/runs/ring_yolov8n_second/weights/best.onnx";

    m_stage1Loaded = loadModel(modelPathStage1, m_netStage1, "Stage1");
    m_stage2Loaded = loadModel(modelPathStage2, m_netStage2, "Stage2");
}

bool DetectionWorker::loadModel(const QString& modelPath, cv::dnn::Net& net, const QString& tag)
{
    try
    {
        net = cv::dnn::readNetFromONNX(modelPath.toStdString());
        bool loaded = !net.empty();

        if (loaded)
            emit logMessage(QString("[%1] ONNX模型加载成功: %2").arg(tag, modelPath));
        else
            emit logMessage(QString("[%1] ONNX模型加载失败: %2").arg(tag, modelPath));

        return loaded;
    }
    catch (const std::exception& e)
    {
        emit logMessage(QString("[%1] ONNX模型异常: %2").arg(tag, e.what()));
        return false;
    }
}
//阶段切换函数
void DetectionWorker::setDetectStage(int stage)
{
    m_currentStage = stage;
    emit logMessage(QString("Detection stage switched to %1").arg(stage));
}

void DetectionWorker::processFrame(const QImage& image)
{
    CameraDetectionResult result;

    if (m_currentStage == 2)   // SecondDetect
    {
        if (!m_stage2Loaded)
        {
            emit logMessage("[Stage2] ONNX模型未加载，无法检测");
            result.found = false;
        }
        else
        {
            detectWithNet(image, result,
                m_netStage2,
                m_confThresholdStage2,
                m_nmsThresholdStage2,
                "Stage2");
        }
    }
    else   // FirstDetect / MovingToSecondPose 默认都走第一阶段
    {
        if (!m_stage1Loaded)
        {
            emit logMessage("[Stage1] ONNX模型未加载，无法检测");
            result.found = false;
        }
        else
        {
            detectWithNet(image, result,
                m_netStage1,
                m_confThresholdStage1,
                m_nmsThresholdStage1,
                "Stage1");
        }
    }

    emit ringDetected(result);
}

cv::Mat DetectionWorker::qimageToMat(const QImage& image)
{
    QImage img = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat mat(
        img.height(),
        img.width(),
        CV_8UC3,
        const_cast<uchar*>(img.bits()),
        img.bytesPerLine()
    );

    cv::Mat matCopy = mat.clone();
    cv::cvtColor(matCopy, matCopy, cv::COLOR_RGB2BGR);
    return matCopy;
}

cv::Mat DetectionWorker::letterboxSquare(const cv::Mat& src)
{
    int w = src.cols;
    int h = src.rows;
    int m = (std::max)(w, h);

    cv::Mat dst = cv::Mat::zeros(m, m, CV_8UC3);
    src.copyTo(dst(cv::Rect(0, 0, w, h)));
    return dst;
}

bool DetectionWorker::detectWithNet(const QImage& image,
    CameraDetectionResult& result,
    cv::dnn::Net& net,
    float confThreshold,
    float nmsThreshold,
    const QString& tag)
{
    try
    {
        result = CameraDetectionResult();
        result.timestamp = QDateTime::currentMSecsSinceEpoch();

        if (image.isNull())
            return false;

        cv::Mat frame = qimageToMat(image);
        if (frame.empty())
            return false;

        cv::Mat inputImage = letterboxSquare(frame);

        float xFactor = static_cast<float>(inputImage.cols) / m_inputWidth;
        float yFactor = static_cast<float>(inputImage.rows) / m_inputHeight;

        cv::Mat blob;
        cv::dnn::blobFromImage(
            inputImage,
            blob,
            1.0 / 255.0,
            cv::Size(m_inputWidth, m_inputHeight),
            cv::Scalar(),
            true,
            false
        );

        net.setInput(blob);
        cv::Mat output = net.forward();

        if (output.dims != 3 || output.size[1] != 5)
        {
            emit logMessage(QString("[%1] YOLO输出维度异常").arg(tag));
            return false;
        }

        const int dimensions = output.size[1];
        const int rows = output.size[2];

        cv::Mat output0(dimensions, rows, CV_32F, output.ptr<float>());
        cv::Mat outputT = output0.t();

        std::vector<cv::Rect> boxes;
        std::vector<float> scores;

        for (int i = 0; i < outputT.rows; ++i)
        {
            const float* data = outputT.ptr<float>(i);

            float cx = data[0];
            float cy = data[1];
            float w = data[2];
            float h = data[3];
            float score = data[4];

            if (score < confThreshold)
                continue;

            int left = static_cast<int>((cx - 0.5f * w) * xFactor);
            int top = static_cast<int>((cy - 0.5f * h) * yFactor);
            int width = static_cast<int>(w * xFactor);
            int height = static_cast<int>(h * yFactor);

            boxes.emplace_back(left, top, width, height);
            scores.push_back(score);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, confThreshold, nmsThreshold, indices);

        if (indices.empty())
        {
            result.found = false;
            return false;
        }

        int bestIdx = indices[0];
        float bestScore = scores[bestIdx];

        for (int idx : indices)
        {
            if (scores[idx] > bestScore)
            {
                bestScore = scores[idx];
                bestIdx = idx;
            }
        }

        cv::Rect bestBox = boxes[bestIdx] & cv::Rect(0, 0, frame.cols, frame.rows);

        result.found = true;
        result.bboxX = static_cast<int>(bestBox.x * m_outputScaleBack);
        result.bboxY = static_cast<int>(bestBox.y * m_outputScaleBack);
        result.bboxW = static_cast<int>(bestBox.width * m_outputScaleBack);
        result.bboxH = static_cast<int>(bestBox.height * m_outputScaleBack);
        result.centerX = result.bboxX + result.bboxW / 2;
        result.centerY = result.bboxY + result.bboxH / 2;
        result.radius = (std::min)(result.bboxW, result.bboxH) / 2;

        emit logMessage(QString("[%1] 检测成功: score=%2, box=[%3,%4,%5,%6]")
            .arg(tag)
            .arg(bestScore, 0, 'f', 3)
            .arg(bestBox.x)
            .arg(bestBox.y)
            .arg(bestBox.width)
            .arg(bestBox.height));

        return true;
    }
    catch (const cv::Exception& e)
    {
        emit logMessage(QString("[%1] OpenCV异常: %2").arg(tag, e.what()));
        return false;
    }
    catch (const std::exception& e)
    {
        emit logMessage(QString("[%1] 标准异常: %2").arg(tag, e.what()));
        return false;
    }
    catch (...)
    {
        emit logMessage(QString("[%1] 未知异常").arg(tag));
        return false;
    }
}