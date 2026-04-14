#include <opencv2/opencv.hpp>
#include "MvCameraControl.h"
#include "sl_lidar.h"

int main()
{
    cv::Mat img(100, 100, CV_8UC3);

    MV_CC_DEVICE_INFO_LIST camList = {};
    camList.nDeviceNum = 0;

    sl_u32 count = 0;
    count += 1;

    return 0;
}