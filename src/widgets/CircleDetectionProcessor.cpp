#include "CircleDetectionProcessor.h"

// 第三方库头文件
#include <opencv2/imgproc.hpp>

// 本项目头文件
#include "utils/QtCvUtils.h"

CircleDetectionProcessor::CircleDetectionProcessor()
    : m_currentAlgorithm(DetectionAlgorithm::Hough)
    , m_lastProcessedFrame(-1)
    , m_paramsChanged(false)
{
}

void CircleDetectionProcessor::clearCache()
{
    m_lastProcessedFrame = -1;
    m_cachedCircles.clear();
    m_cachedResult.release();
    m_paramsChanged = true;
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
    // 检查参数是否发生变化
    const bool changed = (m_params.dp != params.dp ||
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
    
    // 输入参数验证
    if (inputFrame.empty()) {
        return result;  // 返回空结果
    }
    
    result.originalImage = inputFrame.clone();
    
    try {
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
        convertToGray(processed, gray);
        
        // 根据算法进行检测
        std::vector<cv::Vec3f> circles;
        cv::Mat processedOutput;
        
        performDetection(gray, circles, processedOutput);
        
        // 绘制检测结果
        cv::Mat origWithCircles = inputFrame.clone();
        drawDetectionResults(circles, origWithCircles, processedOutput);
        
        // 更新缓存
        updateCache(circles, processedOutput, frameNumber);
        
        // 填充结果
        result.circles = circles;
        result.processedImage = processedOutput;
        result.originalImage = origWithCircles;
        result.frameUpdated = true;
        
    } catch (const cv::Exception& e) {
        // OpenCV 异常处理，返回空结果
        result.circles.clear();
        result.processedImage = inputFrame.clone();
        result.originalImage = inputFrame.clone();
        result.frameUpdated = false;
    }
    
    return result;
}

cv::Mat CircleDetectionProcessor::preprocessImage(const cv::Mat& input)
{
    cv::Mat processed;
    QtCvUtils::applyPreprocessing(input, processed);
    return processed;
}

void CircleDetectionProcessor::convertToGray(const cv::Mat& input, cv::Mat& gray)
{
    if (input.channels() == 3) {
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = input.clone();
    }
}

void CircleDetectionProcessor::performDetection(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& processedOutput)
{
    cv::Mat workingGray = gray.clone();
    
    if (m_currentAlgorithm == DetectionAlgorithm::Hough) {
        // 根据参数决定是否进行二值化预处理
        if (m_params.useBinaryPreprocessing) {
            cv::threshold(workingGray, workingGray, m_params.binaryThresh, 255, cv::THRESH_BINARY);
        }
        detectWithHough(workingGray, circles, processedOutput);
    } else if (m_currentAlgorithm == DetectionAlgorithm::GeometricCenter) {
        detectWithGeometricCenter(workingGray, circles, processedOutput);
    }
}

void CircleDetectionProcessor::drawDetectionResults(const std::vector<cv::Vec3f>& circles, cv::Mat& originalImage, cv::Mat& processedImage)
{
    constexpr int CROSS_SIZE = 10;
    constexpr int CIRCLE_THICKNESS = 3;
    constexpr int CENTER_RADIUS = 3;
    
    const cv::Scalar CENTER_COLOR(0, 255, 0);   // 绿色中心点
    const cv::Scalar CIRCLE_COLOR(0, 0, 255);   // 红色圆形
    
    for (const auto& circle : circles) {
        const cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
        const int radius = cvRound(circle[2]);
        
        // 在原图上绘制
        cv::circle(originalImage, center, CENTER_RADIUS, CENTER_COLOR, -1);
        if (radius > 0) {
            cv::circle(originalImage, center, radius, CIRCLE_COLOR, CIRCLE_THICKNESS);
        } else {
            // 几何中心算法绘制十字标记
            cv::line(originalImage, 
                    cv::Point(center.x - CROSS_SIZE, center.y), 
                    cv::Point(center.x + CROSS_SIZE, center.y), 
                    CIRCLE_COLOR, 2);
            cv::line(originalImage, 
                    cv::Point(center.x, center.y - CROSS_SIZE), 
                    cv::Point(center.x, center.y + CROSS_SIZE), 
                    CIRCLE_COLOR, 2);
        }
        
        // 在处理图上绘制
        cv::circle(processedImage, center, CENTER_RADIUS, CENTER_COLOR, -1);
        if (radius > 0) {
            cv::circle(processedImage, center, radius, CIRCLE_COLOR, CIRCLE_THICKNESS);
        } else {
            cv::line(processedImage, 
                    cv::Point(center.x - CROSS_SIZE, center.y), 
                    cv::Point(center.x + CROSS_SIZE, center.y), 
                    CIRCLE_COLOR, 2);
            cv::line(processedImage, 
                    cv::Point(center.x, center.y - CROSS_SIZE), 
                    cv::Point(center.x, center.y + CROSS_SIZE), 
                    CIRCLE_COLOR, 2);
        }
    }
}

void CircleDetectionProcessor::updateCache(const std::vector<cv::Vec3f>& circles, const cv::Mat& processedImage, int frameNumber)
{
    m_cachedCircles = circles;
    m_cachedResult = processedImage.clone();
    m_lastProcessedFrame = frameNumber;
    m_paramsChanged = false;
}

void CircleDetectionProcessor::detectWithHough(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& outProcessedImage)
{
    // Hough 算法优化参数
    constexpr double SCALE_FACTOR = 0.5;
    const cv::Size BLUR_KERNEL_SIZE(9, 9);
    constexpr double BLUR_SIGMA = 2.0;
    
    // 缩小图像以提高性能
    cv::Mat smallGray;
    cv::resize(gray, smallGray, cv::Size(), SCALE_FACTOR, SCALE_FACTOR, cv::INTER_AREA);
    cv::GaussianBlur(smallGray, smallGray, BLUR_KERNEL_SIZE, BLUR_SIGMA, BLUR_SIGMA);

    std::vector<cv::Vec3f> smallCircles;
    cv::HoughCircles(smallGray, smallCircles, cv::HOUGH_GRADIENT,
                     m_params.dp,
                     m_params.minDist * SCALE_FACTOR,
                     m_params.cannyThresh,
                     m_params.centerThresh,
                     static_cast<int>(m_params.minRadius * SCALE_FACTOR),
                     static_cast<int>(m_params.maxRadius * SCALE_FACTOR));

    // 恢复圆的坐标和半径到原图尺度
    circles.clear();
    circles.reserve(smallCircles.size());
    
    const double INV_SCALE = 1.0 / SCALE_FACTOR;
    for (const auto& c : smallCircles) {
        circles.emplace_back(c[0] * INV_SCALE, c[1] * INV_SCALE, c[2] * INV_SCALE);
    }

    // 输出处理图像（缩小后转回原尺寸，便于后续绘制）
    cv::Mat processed;
    cv::resize(smallGray, processed, gray.size(), 0, 0, cv::INTER_LINEAR);
    cv::cvtColor(processed, outProcessedImage, cv::COLOR_GRAY2BGR);
}

void CircleDetectionProcessor::detectWithGeometricCenter(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& outProcessedImage)
{
    cv::Mat binary;
    const int thresh = m_params.geometricBinaryThresh;
    
    // 应用二值化
    if (m_params.inverseGeometric) {
        cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY_INV);
    } else {
        cv::threshold(gray, binary, thresh, 255, cv::THRESH_BINARY);
    }

    // 计算白色像素的几何中心
    const cv::Moments moments = cv::moments(binary, true);
    circles.clear();
    
    if (moments.m00 > 0) {  // 确保分母不为零
        const double centerX = moments.m10 / moments.m00;
        const double centerY = moments.m01 / moments.m00;
        
        // 使用射线法计算平均半径
        const double radius = calculateRadiusUsingRayMethod(binary, centerX, centerY);
        
        circles.emplace_back(static_cast<float>(centerX), static_cast<float>(centerY), static_cast<float>(radius));
    }
    
    // 输出处理图像（显示二值化结果）
    cv::cvtColor(binary, outProcessedImage, cv::COLOR_GRAY2BGR);
}

double CircleDetectionProcessor::calculateRadiusUsingRayMethod(const cv::Mat& binary, double centerX, double centerY)
{
    constexpr int NUM_SAMPLES = 72;  // 采样角度的数量
    constexpr double STEP_SIZE = 1.0;
    
    double sumR = 0.0;
    int countR = 0;

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        const double angle = 2.0 * CV_PI * i / NUM_SAMPLES;
        const double cosAngle = std::cos(angle);
        const double sinAngle = std::sin(angle);
        
        // 沿着射线向外搜索
        for (double r = STEP_SIZE; ; r += STEP_SIZE) {
            const int x = cvRound(centerX + r * cosAngle);
            const int y = cvRound(centerY + r * sinAngle);

            // 检查边界
            if (x < 0 || x >= binary.cols || y < 0 || y >= binary.rows) {
                break;
            }

            // 检查像素值，遇到黑色像素停止
            if (binary.at<uchar>(y, x) == 0) {
                sumR += r;
                countR++;
                break;
            }
        }
    }
    
    return (countR > 0) ? (sumR / countR) : 0.0;
} 