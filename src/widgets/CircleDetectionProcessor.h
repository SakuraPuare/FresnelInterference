#ifndef CIRCLE_DETECTION_PROCESSOR_H
#define CIRCLE_DETECTION_PROCESSOR_H

#include <vector>
#include <opencv2/opencv.hpp>

// CircleDetectionProcessor：圆检测处理器，支持多种算法

enum class DetectionAlgorithm {
    Hough,
    GeometricCenter
};

struct DetectionParams {
    // Hough参数
    double dp = 1.0;
    int minDist = 50;
    int cannyThresh = 100;
    int centerThresh = 30;
    int minRadius = 10;
    int maxRadius = 100;
    
    // 二值化预处理开关和参数（用于Hough）
    bool useBinaryPreprocessing = false;
    int binaryThresh = 128;
    
    // 几何中心算法参数
    int geometricBinaryThresh = 128;
    bool inverseGeometric = false;  // 是否反向二值化
};

struct DetectionResult {
    std::vector<cv::Vec3f> circles;
    cv::Mat processedImage;
    cv::Mat originalImage;
    bool frameUpdated = false;
    int frameNumber = 0;
};

class CircleDetectionProcessor
{
public:
    CircleDetectionProcessor();
    ~CircleDetectionProcessor() = default;

    // 设置检测参数
    void setAlgorithm(DetectionAlgorithm algorithm);
    void setParams(const DetectionParams& params);
    void clearCache();
    
    // 主要检测接口
    DetectionResult processFrame(const cv::Mat& inputFrame, int frameNumber, bool forceUpdate = false);
    
    // 性能优化：检查是否需要重新处理
    bool needsReprocessing(int frameNumber) const;
    
    // 获取当前状态
    DetectionAlgorithm getCurrentAlgorithm() const { return m_currentAlgorithm; }
    const DetectionParams& getCurrentParams() const { return m_params; }

private:
    // 检测算法模块
    void detectWithHough(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& outProcessedImage);
    void detectWithGeometricCenter(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& outProcessedImage);
    
    // 图像预处理
    cv::Mat preprocessImage(const cv::Mat& input);
    
    // 成员变量
    DetectionAlgorithm m_currentAlgorithm;
    DetectionParams m_params;
    
    // 缓存和性能优化
    int m_lastProcessedFrame;
    bool m_paramsChanged;
    cv::Mat m_cachedResult;
    std::vector<cv::Vec3f> m_cachedCircles;
};

#endif // CIRCLE_DETECTION_PROCESSOR_H 