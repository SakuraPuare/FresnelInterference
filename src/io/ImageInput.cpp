#include "ImageInput.h"

bool ImageInput::shouldUpdateFrame() const
{
    if (!m_enableFrameCache) {
        return true; // 禁用缓存时总是更新
    }
    
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastReadTime).count();
    return elapsed >= m_cacheTimeout_ms;
}

const cv::Mat& ImageInput::readRef()
{
    // 如果缓存有效且帧未过期，返回缓存的帧
    if (m_enableFrameCache && !m_cachedFrame.empty() && !shouldUpdateFrame()) {
        return m_cachedFrame;
    }
    
    // 读取新帧
    cv::Mat newFrame = read();
    if (!newFrame.empty()) {
        m_cachedFrame = newFrame;
        m_lastReadTime = std::chrono::high_resolution_clock::now();
        m_frameSequence++;
    }
    
    return m_cachedFrame;
}

bool ImageInput::isCacheValid() const
{
    return m_enableFrameCache && !m_cachedFrame.empty() && !shouldUpdateFrame();
} 