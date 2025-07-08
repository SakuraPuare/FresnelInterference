#ifndef CIRCLE_DETECTION_PROCESSOR_H
#define CIRCLE_DETECTION_PROCESSOR_H

// 标准 C++ 头文件
#include <vector>

// 第三方库头文件
#include <opencv2/opencv.hpp>

/**
 * @brief 支持的圆检测算法类型
 */
enum class DetectionAlgorithm {
    Hough,          ///< Hough 圆检测算法
    GeometricCenter ///< 几何中心检测算法
};

/**
 * @brief 检测算法参数结构体
 */
struct DetectionParams {
    // Hough 算法参数
    double dp = 1.0;                    ///< 累加器分辨率与图像分辨率的比值
    int minDist = 50;                   ///< 检测到的圆心之间的最小距离
    int cannyThresh = 100;              ///< Canny 边缘检测高阈值
    int centerThresh = 30;              ///< 圆心检测阈值
    int minRadius = 10;                 ///< 最小检测半径
    int maxRadius = 100;                ///< 最大检测半径
    
    // 二值化预处理参数（用于 Hough 算法）
    bool useBinaryPreprocessing = false; ///< 是否启用二值化预处理
    int binaryThresh = 128;              ///< 二值化阈值
    
    // 几何中心算法参数
    int geometricBinaryThresh = 128;     ///< 几何中心算法二值化阈值
    bool inverseGeometric = false;       ///< 是否使用反向二值化
};

/**
 * @brief 检测结果结构体
 */
struct DetectionResult {
    std::vector<cv::Vec3f> circles;     ///< 检测到的圆形列表 (x, y, radius)
    cv::Mat processedImage;             ///< 处理后的图像
    cv::Mat originalImage;              ///< 原始图像副本
    bool frameUpdated = false;          ///< 帧是否更新
    int frameNumber = 0;                ///< 帧编号
};

/**
 * @brief 圆检测处理器类
 * 
 * 封装了圆形检测的核心算法，支持 Hough 变换和几何中心检测。
 * 提供参数调整、结果缓存和性能优化功能。
 */
class CircleDetectionProcessor
{
public:
    /**
     * @brief 构造函数
     */
    CircleDetectionProcessor();
    
    /**
     * @brief 析构函数
     */
    ~CircleDetectionProcessor() = default;

    /**
     * @brief 设置检测算法
     * @param algorithm 要使用的检测算法
     */
    void setAlgorithm(DetectionAlgorithm algorithm);
    
    /**
     * @brief 设置检测参数
     * @param params 检测参数结构体
     */
    void setParams(const DetectionParams& params);
    
    /**
     * @brief 清除缓存
     */
    void clearCache();
    
    /**
     * @brief 处理单帧图像
     * @param inputFrame 输入的图像帧
     * @param frameNumber 帧编号
     * @param forceUpdate 是否强制更新缓存
     * @return 检测结果
     */
    DetectionResult processFrame(const cv::Mat& inputFrame, int frameNumber, bool forceUpdate = false);
    
    /**
     * @brief 检查是否需要重新处理
     * @param frameNumber 帧编号
     * @return 是否需要重新处理
     */
    bool needsReprocessing(int frameNumber) const;
    
    /**
     * @brief 获取当前使用的算法
     * @return 当前算法类型
     */
    DetectionAlgorithm getCurrentAlgorithm() const { return m_currentAlgorithm; }
    
    /**
     * @brief 获取当前参数
     * @return 当前参数的常量引用
     */
    const DetectionParams& getCurrentParams() const { return m_params; }

private:
    // 检测算法模块
    void detectWithHough(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& outProcessedImage);
    void detectWithGeometricCenter(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& outProcessedImage);
    
    // 图像处理辅助方法
    cv::Mat preprocessImage(const cv::Mat& input);
    void convertToGray(const cv::Mat& input, cv::Mat& gray);
    void performDetection(const cv::Mat& gray, std::vector<cv::Vec3f>& circles, cv::Mat& processedOutput);
    void drawDetectionResults(const std::vector<cv::Vec3f>& circles, cv::Mat& processedImage);
    void updateCache(const std::vector<cv::Vec3f>& circles, const cv::Mat& processedImage, int frameNumber);
    double calculateRadiusUsingRayMethod(const cv::Mat& binary, double centerX, double centerY);
    
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