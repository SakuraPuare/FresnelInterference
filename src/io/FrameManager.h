#ifndef FRAME_MANAGER_H
#define FRAME_MANAGER_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <atomic>
#include <mutex>

/**
 * @brief 共享帧管理器 - 优化多模块间的帧数据共享
 * 
 * 这个类管理帧数据的生命周期，允许多个模块共享同一帧数据
 * 而无需多次复制，提高性能并减少内存使用
 */
// FrameManager：帧数据共享与管理单例
class FrameManager
{
public:
    struct FrameData {
        cv::Mat frame;
        uint64_t sequence;
        std::chrono::high_resolution_clock::time_point timestamp;
        std::atomic<int> refCount{1}; // 引用计数
        
        FrameData(const cv::Mat& f, uint64_t seq) 
            : frame(f), sequence(seq), timestamp(std::chrono::high_resolution_clock::now()) {}
    };
    
    using FramePtr = std::shared_ptr<FrameData>;

private:
    static FrameManager* s_instance;
    static std::mutex s_mutex;
    
    std::atomic<uint64_t> m_frameSequence{0};
    FramePtr m_currentFrame;
    std::mutex m_frameMutex;
    
    FrameManager() = default;

public:
    static FrameManager& getInstance();
    
    // 更新当前帧（由输入模块调用）
    FramePtr updateFrame(const cv::Mat& newFrame);
    
    // 获取当前帧（由处理模块调用）
    FramePtr getCurrentFrame();
    
    // 获取当前帧序列号
    uint64_t getCurrentSequence() const { return m_frameSequence.load(); }
    
    // 检查是否有新帧
    bool hasNewFrame(uint64_t lastSeenSequence) const {
        return m_frameSequence.load() > lastSeenSequence;
    }
    
    // 清理资源
    void clear();
};

#endif // FRAME_MANAGER_H 