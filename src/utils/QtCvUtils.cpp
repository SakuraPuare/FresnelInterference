#include "QtCvUtils.h"
#include <QImage>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <iostream>

namespace QtCvUtils {

// QtCvUtils：OpenCV与Qt常用工具函数实现

QPixmap matToQPixmap(const cv::Mat& mat)
{
    if (mat.empty()) {
        return QPixmap();
    }
    
    cv::Mat temp;
    QImage::Format format;

    switch (mat.type()) {
        case CV_8UC1:
            cv::cvtColor(mat, temp, cv::COLOR_GRAY2RGB);
            format = QImage::Format_RGB888;
            break;
        case CV_8UC3:
            cv::cvtColor(mat, temp, cv::COLOR_BGR2RGB);
            format = QImage::Format_RGB888;
            break;
        case CV_8UC4:
            cv::cvtColor(mat, temp, cv::COLOR_BGRA2RGBA);
            format = QImage::Format_RGBA8888;
            break;
        default:
            mat.convertTo(temp, CV_8U);
            if (temp.channels() == 1) {
                cv::cvtColor(temp, temp, cv::COLOR_GRAY2RGB);
            } else if (temp.channels() == 4) {
                cv::cvtColor(temp, temp, cv::COLOR_BGRA2RGBA);
                format = QImage::Format_RGBA8888;
                return QPixmap::fromImage(QImage(temp.data, temp.cols, temp.rows, temp.step, format).copy());
            } else if (temp.channels() != 3) {
                return QPixmap();
            }
            cv::cvtColor(temp, temp, cv::COLOR_BGR2RGB);
            format = QImage::Format_RGB888;
            break;
    }
    
    return QPixmap::fromImage(QImage(temp.data, temp.cols, temp.rows, temp.step, format).copy());
}

void applyPreprocessing(const cv::Mat& src, cv::Mat& dst, 
                       double brightness, double contrast, double gamma)
{
    if (src.empty()) {
        dst = cv::Mat();
        return;
    }

    double adjustedContrast = contrast / 50.0;
    double adjustedGamma = gamma / 100.0;

    cv::Mat tempFrame;
    
    // 确保输入为灰度图
    if (src.channels() == 3) {
        cv::cvtColor(src, tempFrame, cv::COLOR_BGR2GRAY);
    } else {
        tempFrame = src.clone();
    }

    // 应用亮度和对比度调整
    tempFrame.convertTo(dst, CV_8U, adjustedContrast, brightness);

    // 应用伽马校正
    if (adjustedGamma != 1.0) {
        cv::Mat lookUpTable(1, 256, CV_8U);
        uchar *p = lookUpTable.ptr();
        for (int i = 0; i < 256; ++i) {
            p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, adjustedGamma) * 255.0);
        }
        cv::LUT(dst, lookUpTable, dst);
    }
}

void fftshift(cv::Mat& mag)
{
    int cx = mag.cols / 2;
    int cy = mag.rows / 2;

    cv::Mat q0(mag, cv::Rect(0, 0, cx, cy));
    cv::Mat q1(mag, cv::Rect(cx, 0, cx, cy));
    cv::Mat q2(mag, cv::Rect(0, cy, cx, cy));
    cv::Mat q3(mag, cv::Rect(cx, cy, cx, cy));

    cv::Mat tmp;
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);
    q2.copyTo(q1);
    tmp.copyTo(q2);
}

void calculateStats(const std::vector<double>& data, double& mean, double& stddev)
{
    if (data.size() < 2) {
        mean = data.empty() ? 0.0 : data[0];
        stddev = 0.0;
        return;
    }
    
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    mean = sum / data.size();
    
    double sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
    stddev = std::sqrt(sq_sum / data.size() - mean * mean);
}

void findPeaks(const std::vector<float>& data, float threshold, int minDist, 
               std::vector<int>& peakIndices)
{
    peakIndices.clear();
    
    for (int i = 1; i < (int)data.size() - 1; ++i) {
        if (data[i] > data[i-1] && data[i] > data[i+1] && data[i] > threshold) {
            if (peakIndices.empty() || (i - peakIndices.back()) >= minDist) {
                peakIndices.push_back(i);
            }
        }
    }
}

void findTwoStrongestPeaks(const std::vector<float>& data, float threshold, int minDist, 
                          std::vector<int>& peakIndices)
{
    std::vector<int> allPeaks;
    findPeaks(data, threshold, minDist, allPeaks);
    
    peakIndices.clear();
    
    if (allPeaks.empty()) {
        return;
    }
    
    // 按峰值强度排序
    std::sort(allPeaks.begin(), allPeaks.end(), 
        [&data](int a, int b) { return data[a] > data[b]; });
    
    // 取最强的两个峰
    peakIndices.push_back(allPeaks[0]);
    if (allPeaks.size() > 1) {
        peakIndices.push_back(allPeaks[1]);
    }
    
    // 按位置排序输出
    std::sort(peakIndices.begin(), peakIndices.end());
}

std::optional<std::pair<double, double>> linearFit(const std::vector<std::pair<double, double>>& data)
{
    if (data.size() < 2) return std::nullopt;
    
    double sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
    int n = data.size();
    
    for (const auto& [x, y] : data) {
        sumX += x;
        sumY += y;
        sumXX += x * x;
        sumXY += x * y;
    }
    
    double denominator = n * sumXX - sumX * sumX;
    if (denominator == 0) return std::nullopt;
    
    double k = (n * sumXY - sumX * sumY) / denominator;
    double b = (sumY * sumXX - sumX * sumXY) / denominator;
    
    return std::make_pair(k, b);
}

} // namespace QtCvUtils 