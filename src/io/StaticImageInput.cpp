#include "StaticImageInput.h"
#include <opencv2/highgui.hpp>

StaticImageInput::StaticImageInput(const std::string& imagePath) {
    this->init();
    m_staticImage = cv::imread(imagePath);
    if (!m_staticImage.empty()) {
        this->opened = true;
    } else {
        this->opened = false;
    }
}

bool StaticImageInput::init() {
    return true; // No complex initialization needed
}

cv::Mat StaticImageInput::read() {
    if (!this->opened) {
        return cv::Mat();
    }
    
    // 性能优化：静态图像天然适合缓存
    if (isCacheValid()) {
        return m_cachedFrame.clone();
    }
    
    // 对于静态图像，缓存超时时间可以设置得很长
    if (m_cachedFrame.empty()) {
        m_cachedFrame = m_staticImage.clone();
        m_lastReadTime = std::chrono::high_resolution_clock::now();
        m_frameSequence++;
    }
    
    return m_cachedFrame.clone();
} 