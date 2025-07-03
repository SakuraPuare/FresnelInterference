#include "FrameManager.h"

FrameManager* FrameManager::s_instance = nullptr;
std::mutex FrameManager::s_mutex;

FrameManager& FrameManager::getInstance()
{
    std::lock_guard<std::mutex> lock(s_mutex);
    if (s_instance == nullptr) {
        s_instance = new FrameManager();
    }
    return *s_instance;
}

FrameManager::FramePtr FrameManager::updateFrame(const cv::Mat& newFrame)
{
    if (newFrame.empty()) {
        return nullptr;
    }
    
    std::lock_guard<std::mutex> lock(m_frameMutex);
    
    uint64_t newSequence = ++m_frameSequence;
    m_currentFrame = std::make_shared<FrameData>(newFrame, newSequence);
    
    return m_currentFrame;
}

FrameManager::FramePtr FrameManager::getCurrentFrame()
{
    std::lock_guard<std::mutex> lock(m_frameMutex);
    return m_currentFrame;
}

void FrameManager::clear()
{
    std::lock_guard<std::mutex> lock(m_frameMutex);
    m_currentFrame.reset();
    m_frameSequence = 0;
} 