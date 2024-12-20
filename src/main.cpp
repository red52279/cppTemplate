#include <opencv2/opencv.hpp>
#include "apps.h"
#include "VCOM.h"
#include "camera_driver.h"

VCOM::VCOM com; // 串口

int main() {

    Camera cam("../include/camera_driver/cameraInit/HIKcamera0.yaml");

    return 0;
}