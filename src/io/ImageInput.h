#ifndef IMAGE_INPUT_H
#define IMAGE_INPUT_H

#include <opencv2/opencv.hpp>
#include <chrono>
#include <atomic>

class ImageInput
{
protected:
    cv::Size imgResolution;
    bool opened;
    
    // 性能优化：帧缓存和变化检测
    cv::Mat m_cachedFrame;
    std::chrono::high_resolution_clock::time_point m_lastReadTime;
    std::atomic<uint64_t> m_frameSequence{0};
    bool m_enableFrameCache;
    int m_cacheTimeout_ms; // 缓存超时时间(毫秒)
    
    // 检查是否需要更新帧
    virtual bool shouldUpdateFrame() const;
    
public:
    ImageInput(cv::Size res = cv::Size(640,480)) 
        : imgResolution(res), opened(false), m_enableFrameCache(true), m_cacheTimeout_ms(30) {}
    
    virtual ~ImageInput() = default;

    virtual bool        init() = 0;
    virtual cv::Mat     read() = 0;
    
    // 高性能读取接口 - 返回引用避免拷贝
    virtual const cv::Mat& readRef();
    
    // 获取帧序列号，用于判断帧是否更新
    uint64_t getFrameSequence() const { return m_frameSequence.load(); }

    ImageInput&         operator>>(cv::Mat& image)  { image = this->read(); return *this; }
    bool                isOpened() const            { return opened; }
    operator            bool()                      { return opened; }

    void setResolution(const cv::Size & _res) { imgResolution = _res; }
    auto getResolution() { return imgResolution; }
    
    // 缓存控制
    void setFrameCache(bool enable, int timeout_ms = 30) { 
        m_enableFrameCache = enable; 
        m_cacheTimeout_ms = timeout_ms;
    }
    
    // 检查缓存是否有效
    bool isCacheValid() const;
};

#endif 
