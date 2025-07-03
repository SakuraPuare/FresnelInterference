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
    return m_staticImage.clone();
} 