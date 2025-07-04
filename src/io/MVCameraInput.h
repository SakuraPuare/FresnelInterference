#ifndef CAMERA_INPUT_H
#define CAMERA_INPUT_H

#include "ImageInput.h"

#include "CameraApi.h"

#include <iostream>
#include <string>

class MVCameraInput : public ImageInput {
private:
    unsigned char*          g_pRgbBuffer = nullptr; // 处理后图像输出的数据缓存区地址
    CameraHandle            hCamera;                // 相机句柄

public:
    MVCameraInput();
    bool        init()      final;
    cv::Mat     read()      final;
    
    // 相机参数设置方法
    bool        setExposureTime(double exposureTime);  // 设置曝光时间(微秒)
    bool        setGain(int gain);                      // 设置增益
    bool        setGamma(double gamma);                 // 设置伽马值
    bool        setFrameRate(int frameRate);            // 设置帧率
    bool        setContrast(int contrast);              // 设置对比度
    bool        setSharpness(int sharpness);            // 设置锐度
    bool        setSaturation(int saturation);          // 设置饱和度
    
    // 获取相机参数
    double      getExposureTime();                      // 获取曝光时间
    int         getGain();                              // 获取增益
    double      getGamma();                             // 获取伽马值
    int         getFrameRate();                         // 获取帧率
    int         getContrast();                          // 获取对比度
    int         getSharpness();                         // 获取锐度
    int         getSaturation();                        // 获取饱和度

    ~MVCameraInput();
};

#endif 
