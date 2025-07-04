#include "CircleDetectionProcessor.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <numeric>

CircleDetectionProcessor::CircleDetectionProcessor(Algorithm algo)
    : m_algorithm(algo)
    , m_threshold(-1) // 默认自动阈值
{
}

void CircleDetectionProcessor::setAlgorithm(Algorithm algo)
{
    m_algorithm = algo;
}

CircleDetectionProcessor::Algorithm CircleDetectionProcessor::algorithm() const
{
    return m_algorithm;
}

void CircleDetectionProcessor::setThreshold(int thresh)
{
    m_threshold = thresh;
}

int CircleDetectionProcessor::threshold() const
{
    return m_threshold;
}

CircleDetectionProcessor::Result CircleDetectionProcessor::process(const cv::Mat& frame)
{
    Result ret;
    if (frame.empty()) return ret;

    cv::Mat gray;
    if (frame.channels() == 3)
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    else
        gray = frame.clone();

    switch (m_algorithm)
    {
    case Algorithm::HoughTransform:
        ret = detectWithHough(gray);
        break;
    case Algorithm::GeometricCenter:
        ret = detectWithGeometric(gray);
        break;
    default:
        ret.valid = false;
        break;
    }
    return ret;
}

CircleDetectionProcessor::Result CircleDetectionProcessor::detectWithHough(const cv::Mat& gray)
{
    Result res;
    if (gray.empty()) return res;

    // 可选：均值模糊降噪
    cv::Mat blur;
    cv::medianBlur(gray, blur, 5);

    // 霍夫变换参数
    std::vector<cv::Vec3f> circles;
    double dp = 1.5;
    double minDist = gray.rows / 8.0;
    double param1 = 100; // Canny 高阈值
    double param2 = 30;  // 累加器阈值
    int minRadius = 0;
    int maxRadius = 0;

    cv::HoughCircles(blur, circles, cv::HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);

    if (!circles.empty())
    {
        // 取最大圆
        auto best = *std::max_element(circles.begin(), circles.end(), [](const cv::Vec3f& a, const cv::Vec3f& b){ return a[2] < b[2]; });
        res.center = cv::Point2f(best[0], best[1]);
        res.radius = best[2];
        res.valid = true;
    }
    return res;
}

CircleDetectionProcessor::Result CircleDetectionProcessor::detectWithGeometric(const cv::Mat& gray)
{
    Result res;
    if (gray.empty()) return res;

    cv::Mat bin;
    int threshVal = m_threshold >= 0 ? m_threshold : 0;
    if (m_threshold < 0)
    {
        // Otsu
        cv::threshold(gray, bin, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    }
    else
    {
        cv::threshold(gray, bin, threshVal, 255, cv::THRESH_BINARY);
    }

    // 寻找最大轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return res;

    // 选择面积最大的
    size_t bestIdx = 0;
    double bestArea = 0;
    for (size_t i = 0; i < contours.size(); ++i)
    {
        double area = cv::contourArea(contours[i]);
        if (area > bestArea)
        {
            bestArea = area;
            bestIdx = i;
        }
    }
    if (bestArea < 10) return res; // too small

    cv::Moments m = cv::moments(contours[bestIdx]);
    if (m.m00 == 0) return res;

    res.center = cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
    // 用等效圆面积求半径
    res.radius = std::sqrt(bestArea / CV_PI);
    res.valid = true;
    return res;
}
