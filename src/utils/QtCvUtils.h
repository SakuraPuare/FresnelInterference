#ifndef QT_CV_UTILS_H
#define QT_CV_UTILS_H

#include <QPixmap>
#include <opencv2/opencv.hpp>
#include <vector>
#include <optional>

namespace QtCvUtils {
    
    /**
     * @brief 将OpenCV Mat转换为QPixmap
     * @param mat 输入的OpenCV Mat对象
     * @return 转换后的QPixmap对象
     */
    QPixmap matToQPixmap(const cv::Mat& mat);
    
    /**
     * @brief 对图像进行预处理（亮度、对比度、伽马校正）
     * @param src 源图像
     * @param dst 输出图像
     * @param brightness 亮度调整值 (-100 to 100)
     * @param contrast 对比度调整值 (1 to 200, 100为原始值)
     * @param gamma 伽马值 (10 to 300, 100为原始值)
     */
    void applyPreprocessing(const cv::Mat& src, cv::Mat& dst, 
                           double brightness = 0, double contrast = 100, double gamma = 100);
    
    /**
     * @brief FFT频率域中心移位
     * @param mag 幅度谱图像
     */
    void fftshift(cv::Mat& mag);
    
    /**
     * @brief 计算数据的均值和标准差
     * @param data 输入数据向量
     * @param mean 输出均值
     * @param stddev 输出标准差
     */
    void calculateStats(const std::vector<double>& data, double& mean, double& stddev);
    
    /**
     * @brief 在向量中查找峰值
     * @param data 输入数据向量
     * @param threshold 峰值阈值
     * @param minDist 最小峰值间距
     * @param peakIndices 输出峰值索引
     */
    void findPeaks(const std::vector<float>& data, float threshold, int minDist, 
                   std::vector<int>& peakIndices);
    
    /**
     * @brief 查找两个最强的峰值
     * @param data 输入数据向量
     * @param threshold 峰值阈值
     * @param minDist 最小峰值间距
     * @param peakIndices 输出峰值索引
     */
    void findTwoStrongestPeaks(const std::vector<float>& data, float threshold, int minDist, 
                              std::vector<int>& peakIndices);
    
    /**
     * @brief 线性拟合函数
     * @param data 输入的(x,y)数据点对
     * @return 返回拟合的斜率和截距，如果拟合失败返回nullopt
     */
    std::optional<std::pair<double, double>> linearFit(const std::vector<std::pair<double, double>>& data);
}

#endif // QT_CV_UTILS_H