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
                   m_params.useBinaryPreprocessing != params.useBinaryPreprocessing ||
                   m_params.binaryThresh != params.binaryThresh ||
                   m_params.geometricBinaryThresh != params.geometricBinaryThresh ||
                   m_params.inverseGeometric != params.inverseGeometric);
    
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
        // 根据参数决定是否进行二值化预处理
        if (m_params.useBinaryPreprocessing) {
            cv::Mat binary;
            cv::threshold(gray, binary, m_params.binaryThresh, 255, cv::THRESH_BINARY);
            gray = binary;
        }
        detectWithHough(gray, circles, processedOutput);
    } else if (m_currentAlgorithm == DetectionAlgorithm::GeometricCenter) {
        detectWithGeometricCenter(gray, circles, processedOutput);
    }
    
    // 在原图和处理图上绘制检测结果
    cv::Mat origWithCircles = inputFrame.clone();
    for (const auto& circle : circles) {
        cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
        int radius = cvRound(circle[2]);
        
        // 在原图上绘制
        cv::circle(origWithCircles, center, 3, cv::Scalar(0, 255, 0), -1);
        if (radius > 0) {  // 对于几何中心算法，半径可能为0
            cv::circle(origWithCircles, center, radius, cv::Scalar(0, 0, 255), 3);
        } else {
            // 几何中心算法只绘制十字标记
            cv::line(origWithCircles, cv::Point(center.x - 10, center.y), cv::Point(center.x + 10, center.y), cv::Scalar(0, 0, 255), 2);
            cv::line(origWithCircles, cv::Point(center.x, center.y - 10), cv::Point(center.x, center.y + 10), cv::Scalar(0, 0, 255), 2);
        }
        
        // 在处理图上绘制
        cv::circle(processedOutput, center, 3, cv::Scalar(0, 255, 0), -1);
        if (radius > 0) {
            cv::circle(processedOutput, center, radius, cv::Scalar(0, 0, 255), 3);
        } else {
            cv::line(processedOutput, cv::Point(center.x - 10, center.y), cv::Point(center.x + 10, center.y), cv::Scalar(0, 0, 255), 2);
            cv::line(processedOutput, cv::Point(center.x, center.y - 10), cv::Point(center.x, center.y + 10), cv::Scalar(0, 0, 255), 2);
        }
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
    // 新增：缩小图像
    double scale = 0.5;
    cv::Mat smallGray;
    cv::resize(gray, smallGray, cv::Size(), scale, scale, cv::INTER_AREA);
    cv::GaussianBlur(smallGray, smallGray, cv::Size(9, 9), 2, 2);

    std::vector<cv::Vec3f> smallCircles;
    cv::HoughCircles(smallGray, smallCircles, cv::HOUGH_GRADIENT,
                     m_params.dp,
                     m_params.minDist * scale,
                     m_params.cannyThresh,
                     m_params.centerThresh,
                     m_params.minRadius * scale,
                     m_params.maxRadius * scale);

    // 恢复圆的坐标和半径到原图尺度
    circles.clear();
    for (const auto& c : smallCircles) {
        circles.push_back(cv::Vec3f(c[0] / scale, c[1] / scale, c[2] / scale));
    }

    // 输出处理图像（缩小后转回原尺寸，便于后续绘制）
    cv::Mat processed;
    cv::resize(smallGray, processed, gray.size(), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(processed, outProcessedImage, cv::COLOR_GRAY2BGR);
}

void CircleDetectionProcessor::detectWithGeometricCenter(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& outProcessedImage)
{
    cv::Mat binary;
    
    // 进行二值化处理
    int thresh = m_params.geometricBinaryThresh;
    if (m_params.inverseGeometric) {
        cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY_INV);
    } else {
        cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY);
    }
    
    // 计算白色像素的几何中心
    cv::Moments moments = cv::moments(binary, true);
    
    circles.clear();
    if (moments.m00 != 0) {  // 确保有白色像素
        double centerX = moments.m10 / moments.m00;
        double centerY = moments.m01 / moments.m00;
        
        // 对于几何中心算法，我们不计算真实半径，设为0表示这是中心点
        circles.push_back(cv::Vec3f(centerX, centerY, 0));
    }
    
    // 输出处理图像（显示二值化结果）
    cv::cvtColor(binary, outProcessedImage, cv::COLOR_GRAY2BGR);
} 