#include "CircleDetectionProcessor.h"
#include <opencv2/imgproc.hpp>

CircleDetectionProcessor::CircleDetectionProcessor()
    : m_currentAlgorithm(DetectionAlgorithm::Hough)
    , m_lastProcessedFrame(-1)
    , m_paramsChanged(false)
{
}

void CircleDetectionProcessor::setAlgorithm(DetectionAlgorithm algorithm)
{
    if (m_currentAlgorithm != algorithm) {
        m_currentAlgorithm = algorithm;
        m_paramsChanged = true;
    }
}

void CircleDetectionProcessor::setParams(const DetectionParams& params)
{
    // 简单比较参数是否发生变化
    bool changed = (m_params.dp != params.dp ||
                   m_params.minDist != params.minDist ||
                   m_params.cannyThresh != params.cannyThresh ||
                   m_params.centerThresh != params.centerThresh ||
                   m_params.minRadius != params.minRadius ||
                   m_params.maxRadius != params.maxRadius ||
                   m_params.binaryThresh != params.binaryThresh);
    
    if (changed) {
        m_params = params;
        m_paramsChanged = true;
    }
}

bool CircleDetectionProcessor::needsReprocessing(int frameNumber) const
{
    return (frameNumber != m_lastProcessedFrame) || m_paramsChanged;
}

DetectionResult CircleDetectionProcessor::processFrame(const cv::Mat& inputFrame, int frameNumber, bool forceUpdate)
{
    DetectionResult result;
    result.frameNumber = frameNumber;
    result.originalImage = inputFrame.clone();
    
    // 性能优化：如果帧未更新且参数未变化，返回缓存结果
    if (!forceUpdate && !needsReprocessing(frameNumber)) {
        result.circles = m_cachedCircles;
        result.processedImage = m_cachedResult.clone();
        result.frameUpdated = false;
        return result;
    }
    
    // 预处理图像
    cv::Mat processed = preprocessImage(inputFrame);
    
    // 转换为灰度图
    cv::Mat gray;
    if (processed.channels() == 3) {
        cv::cvtColor(processed, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = processed.clone();
    }
    
    // 根据算法进行检测
    std::vector<cv::Vec3f> circles;
    cv::Mat processedOutput;
    
    if (m_currentAlgorithm == DetectionAlgorithm::Hough) {
        detectWithHough(gray, circles, processedOutput);
    } else if (m_currentAlgorithm == DetectionAlgorithm::BinaryCenter) {
        detectWithBinary(gray, circles, processedOutput);
    }
    
    // 在原图和处理图上绘制检测结果
    cv::Mat origWithCircles = inputFrame.clone();
    for (const auto& circle : circles) {
        cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
        int radius = cvRound(circle[2]);
        
        // 在原图上绘制
        cv::circle(origWithCircles, center, 3, cv::Scalar(0, 255, 0), -1);
        cv::circle(origWithCircles, center, radius, cv::Scalar(0, 0, 255), 3);
        
        // 在处理图上绘制
        cv::circle(processedOutput, center, 3, cv::Scalar(0, 255, 0), -1);
        cv::circle(processedOutput, center, radius, cv::Scalar(0, 0, 255), 3);
    }
    
    // 更新缓存
    m_cachedCircles = circles;
    m_cachedResult = processedOutput.clone();
    m_lastProcessedFrame = frameNumber;
    m_paramsChanged = false;
    
    // 填充结果
    result.circles = circles;
    result.processedImage = processedOutput;
    result.originalImage = origWithCircles;
    result.frameUpdated = true;
    
    return result;
}

cv::Mat CircleDetectionProcessor::preprocessImage(const cv::Mat& input)
{
    cv::Mat processed;
    if (input.channels() == 3) {
        cv::cvtColor(input, processed, cv::COLOR_BGR2GRAY);
    } else {
        processed = input.clone();
    }
    
    // 预处理：直方图均衡 + 去噪
    cv::equalizeHist(processed, processed);
    cv::GaussianBlur(processed, processed, cv::Size(5, 5), 1, 1);
    
    // 转回BGR用于显示
    cv::cvtColor(processed, processed, cv::COLOR_GRAY2BGR);
    return processed;
}

void CircleDetectionProcessor::detectWithHough(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& outProcessedImage)
{
    cv::Mat processed = gray.clone();
    cv::GaussianBlur(processed, processed, cv::Size(9, 9), 2, 2);
    
    cv::HoughCircles(processed, circles, cv::HOUGH_GRADIENT,
                     m_params.dp,
                     m_params.minDist,
                     m_params.cannyThresh,
                     m_params.centerThresh,
                     m_params.minRadius,
                     m_params.maxRadius);
    
    cv::cvtColor(processed, outProcessedImage, cv::COLOR_GRAY2BGR);
}

void CircleDetectionProcessor::detectWithBinary(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& outProcessedImage)
{
    cv::Mat binary;
    cv::threshold(gray, binary, m_params.binaryThresh, 255, cv::THRESH_BINARY);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < 100) { // 过滤小噪声
            continue;
        }
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        circles.push_back(cv::Vec3f(center.x, center.y, radius));
    }
    
    cv::cvtColor(binary, outProcessedImage, cv::COLOR_GRAY2BGR);
} 