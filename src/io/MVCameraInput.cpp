#include <iostream>
#include <string>
#include "MVCameraInput.h"

// MVCameraInput：迈德威视相机输入实现
MVCameraInput::MVCameraInput()
{
    this->init();
}

bool MVCameraInput::init()
{
    tSdkCameraDevInfo       tCameraEnumList;   // 相机列表
    int                     iCameraCounts = 1; // 摄像头数量
    int                     iStatus = -1;      // 相机初始化返回值
    tSdkCameraCapbility     tCapability;       // 设备描述信息
    int                     nChannel = 3;

    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    //如果需要连接多个设备，则将tCameraEnumList改为数组，同时修改iCameraCounts的值
    CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);

    //没有连接设备
    if (iCameraCounts == 0)
    {
        std::cerr << "No camera was found!\n";
        this->opened = false;
        return false;
    }

    do {
        //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
        //该示例中，我们只假设连接了一个相机。因此，只初始化第一个相机。
        //(-1,-1)表示加载上次退出前保存的参数，如果是第一次使用该相机，则加载默认参数.
        iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);
        if (iStatus != CAMERA_STATUS_SUCCESS) { 
            std::cerr << "Failed to init the camera! Error code is " + std::to_string(iStatus) + "\n"; 
            std::cerr << CameraGetErrorString(iStatus);
        }
    } while (iStatus != CAMERA_STATUS_SUCCESS); //初始化失败

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);

    // 遍历所有支持的分辨率
    for (int i = 0; i < tCapability.iImageSizeDesc; i++) {
        // 打印每个分辨率的信息
        printf("Resolution %d: %s, %d x %d\n", i,
               tCapability.pImageSizeDesc[i].acDescription,
               tCapability.pImageSizeDesc[i].iWidth,
               tCapability.pImageSizeDesc[i].iHeight);

        // 查找原生分辨率
        if (tCapability.pImageSizeDesc[i].iWidth == 2592 && tCapability.pImageSizeDesc[i].iHeight == 1944) {
            printf("Found native resolution at index %d\n", i);
            CameraSetImageResolution(hCamera, &tCapability.pImageSizeDesc[i]);
            this->imgResolution = cv::Size(2592, 1944);
            break; 
        }
    }

    if (tCapability.sIspCapacity.bMonoSensor) {
        nChannel = 1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    } else {
        nChannel = 3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }

    // 分配图像缓冲区
    g_pRgbBuffer =  new unsigned char[tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * nChannel]; 

    /*让SDK进入工作模式，开始接收来自相机发送的图像
    数据。如果当前相机是触发模式，则需要接收到
    触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
    例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
         CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
         CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
         更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
    */
    //printf("iFrameSpeedDesc = %d\n",tCapability.iFrameSpeedDesc);         //frame rate option number,when set,iframeSpeed range (0,iFrameSpeedDec-1)
    //CameraSetFrameSpeed(hCamera, 2);          //set frame rate,the bigger,the quicker(0~2)
    //CameraSetAnalogGain(hCamera, 4);           //mo ni zeng yi
    //int analoggain;
    //CameraGetAnalogGain(hCamera, &analoggain);
    //std::cout << "analoggain = " << analoggain << std::endl;
    CameraSetAeState(hCamera, false);             //设置为手动曝光
    if (CameraSetExposureTime(hCamera, 10000) == 0) //单位为微秒
        std::cout << "set exposure successful!\n";
    else
        std::cout << "set exposure failed!\n";
    double expTime = 0;
    CameraGetExposureTime(hCamera, &expTime);
    std::cout << "explore time = " << expTime << "us" << std::endl;
    
    this->opened = true;
    return true;
}

cv::Mat MVCameraInput::read()
{
    // 性能优化：避免频繁的相机调用
    if (isCacheValid()) {
        return m_cachedFrame.clone();
    }
    
    tSdkFrameHead           sFrameInfo;        //输入图像的帧头信息
    BYTE*                   pbyBuffer;         //输出图像数据的缓冲区地址
    CameraSdkStatus         status;

    cv::Mat matImg;

    // 第四个参数为超时时间，单位为ms - 减少超时时间以提高响应性
    if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 50) == CAMERA_STATUS_SUCCESS) 
    {
        status = CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

        if (status == CAMERA_STATUS_SUCCESS) {

#if defined(WIN32)
            // 由于SDK输出的数据默认是从底到顶的，转换为Opencv图片需要做一下垂直镜像
            CameraFlipFrameBuffer(g_pRgbBuffer, &sFrameInfo, 1);
#endif

            // 创建Mat时避免复制数据
            matImg = cv::Mat(
                cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer
            );

            // 只在分辨率不匹配时才进行resize
            /*if (matImg.size() != this->imgResolution) {
                cv::resize(matImg, matImg, this->imgResolution);
            }*/
        }

        //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
        //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
    }
    
    return matImg;
}

bool MVCameraInput::setExposureTime(double exposureTime)
{
    if (!this->opened) return false;
    return CameraSetExposureTime(hCamera, exposureTime) == CAMERA_STATUS_SUCCESS;
}

bool MVCameraInput::setGain(int gain)
{
    if (!this->opened) return false;
    return CameraSetAnalogGain(hCamera, gain) == CAMERA_STATUS_SUCCESS;
}

bool MVCameraInput::setGamma(double gamma)
{
    if (!this->opened) return false;
    return CameraSetGamma(hCamera, (int)(gamma * 100)) == CAMERA_STATUS_SUCCESS;
}

bool MVCameraInput::setFrameRate(int frameRate)
{
    if (!this->opened) return false;
    return CameraSetFrameRate(hCamera, frameRate) == CAMERA_STATUS_SUCCESS;
}

bool MVCameraInput::setContrast(int contrast)
{
    if (!this->opened) return false;
    return CameraSetContrast(hCamera, contrast) == CAMERA_STATUS_SUCCESS;
}

bool MVCameraInput::setSharpness(int sharpness)
{
    if (!this->opened) return false;
    return CameraSetSharpness(hCamera, sharpness) == CAMERA_STATUS_SUCCESS;
}

bool MVCameraInput::setSaturation(int saturation)
{
    if (!this->opened) return false;
    return CameraSetSaturation(hCamera, saturation) == CAMERA_STATUS_SUCCESS;
}

double MVCameraInput::getExposureTime()
{
    if (!this->opened) return 0.0;
    double exposureTime = 0.0;
    CameraGetExposureTime(hCamera, &exposureTime);
    return exposureTime;
}

int MVCameraInput::getGain()
{
    if (!this->opened) return 0;
    int gain = 0;
    CameraGetAnalogGain(hCamera, &gain);
    return gain;
}

double MVCameraInput::getGamma()
{
    if (!this->opened) return 0.0;
    int gamma = 0;
    CameraGetGamma(hCamera, &gamma);
    return gamma / 100.0;
}

int MVCameraInput::getFrameRate()
{
    if (!this->opened) return 0;
    int frameRate = 0;
    CameraGetFrameRate(hCamera, &frameRate);
    return frameRate;
}

int MVCameraInput::getContrast()
{
    if (!this->opened) return 0;
    int contrast = 0;
    CameraGetContrast(hCamera, &contrast);
    return contrast;
}

int MVCameraInput::getSharpness()
{
    if (!this->opened) return 0;
    int sharpness = 0;
    CameraGetSharpness(hCamera, &sharpness);
    return sharpness;
}

int MVCameraInput::getSaturation()
{
    if (!this->opened) return 0;
    int saturation = 0;
    CameraGetSaturation(hCamera, &saturation);
    return saturation;
}

MVCameraInput::~MVCameraInput()
{
    CameraUnInit(hCamera);
    //注意，先反初始化后再释放图像缓冲区
    delete g_pRgbBuffer;
}
