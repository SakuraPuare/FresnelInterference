#pragma once

#include <opencv2/core.hpp>
#include <vector>

// CircleDetectionProcessor：负责从图像中检测圆形光源并给出圆心与半径
class CircleDetectionProcessor
{
public:
    enum class Algorithm
    {
        HoughTransform = 0,   // 使用 HoughCircles
        GeometricCenter       // 二值化后取最大连通域的几何中心
    };

    struct Result
    {
        bool valid = false;           // 是否检测成功
        cv::Point2f center;          // 圆心坐标（像素）
        float radius = 0.0f;         // 半径（像素）
    };

    CircleDetectionProcessor(Algorithm algo = Algorithm::HoughTransform);

    // 设置使用的检测算法
    void setAlgorithm(Algorithm algo);
    Algorithm algorithm() const;

    // 设置二值化阈值，<0 表示使用 Otsu 自动阈值
    void setThreshold(int thresh);
    int threshold() const;

    // 主处理入口：输入 BGR 或灰度图像，返回检测结果
    Result process(const cv::Mat& frame);

private:
    Result detectWithHough(const cv::Mat& gray);
    Result detectWithGeometric(const cv::Mat& gray);

    Algorithm m_algorithm;
    int m_threshold; // 0-255，<0 使用 Otsu
};
