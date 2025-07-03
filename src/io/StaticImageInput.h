#ifndef STATIC_IMAGE_INPUT_H
#define STATIC_IMAGE_INPUT_H

#include "ImageInput.h"

class StaticImageInput : public ImageInput {
private:
    cv::Mat m_staticImage;

public:
    explicit StaticImageInput(const std::string& imagePath);

    bool    init() override;
    cv::Mat read() override;

    ~StaticImageInput() override = default;
};

#endif // STATIC_IMAGE_INPUT_H 