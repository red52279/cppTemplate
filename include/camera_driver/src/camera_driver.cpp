#include "camera_driver.h"

void camCallBack(unsigned char *pData, MV_FRAME_OUT_INFO_EX *pFrameInfo, void *pUser) {

    static int count = 0;
    static auto start = std::chrono::high_resolution_clock::now();

    count++;
    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = now - start;

    if (elapsed.count() >= 1.0) { // 每秒输出一次
        std::cout << "fps : " << count << std::endl;
        count = 0;
        start = now;
    }

    cv::Mat img = cv::Mat(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3, pData);
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    cv::imshow("cap_img", img);
    cv::waitKey(1);

}

HK_Camera::HK_Camera() {
    call_back_ptr_ = nullptr;
    hik_camera_node_ = nullptr;
    handle_ = nullptr;
    device_info_ = nullptr;
    camera_mode_ = 1;
}

HK_Camera::HK_Camera(CallbackType callback, void* node_this = nullptr) {
    call_back_ptr_ = callback;
    hik_camera_node_ = node_this;
    handle_ = nullptr;
    device_info_ = nullptr;
    camera_mode_ = 0;
}

HK_Camera::~HK_Camera() {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
}

void HK_Camera::stopCamera() {
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
}

unsigned int HK_Camera::findConnectableUSBDevice() {
    int nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list_);
    if (device_list_.nDeviceNum > 0) {
        std::cout << "发现 " << device_list_.nDeviceNum << " 个设备 : " << std::endl;
        for (unsigned int i = 0; i < device_list_.nDeviceNum; i++) {
            device_info_ = device_list_.pDeviceInfo[i];
            if (device_info_ == nullptr) break;
            std::cout << "--------------------------------------------" << "\n第 " << i + 1 << " 个 : " << std::endl;
            if (device_info_->nTLayerType == MV_USB_DEVICE && MV_CC_IsDeviceAccessible(device_info_, MV_ACCESS_Monitor)) {
                std::cout << "idProduct : " << device_info_->SpecialInfo.stUsb3VInfo.idProduct << std::endl;
                std::cout << "idVendor : " << device_info_->SpecialInfo.stUsb3VInfo.idVendor << std::endl;
                std::cout << "chDeviceGUID : " << device_info_->SpecialInfo.stUsb3VInfo.chDeviceGUID << std::endl;
                std::cout << "chVendorName : " << device_info_->SpecialInfo.stUsb3VInfo.chVendorName << std::endl;
                std::cout << "chModelName : " << device_info_->SpecialInfo.stUsb3VInfo.chModelName << std::endl;
                std::cout << "chDeviceVersion : " << device_info_->SpecialInfo.stUsb3VInfo.chDeviceVersion << std::endl;
                std::cout << "chSerialNumber : " << device_info_->SpecialInfo.stUsb3VInfo.chSerialNumber << std::endl;
                std::cout << "nDeviceAddress : " << device_info_->SpecialInfo.stUsb3VInfo.nDeviceAddress << std::endl;
            }
            std::cout << "--------------------------------------------" << std::endl;
        }
        if (device_list_.nDeviceNum > 1) {
            int index = -1;
            while (index < 1 && index > device_list_.nDeviceNum) {
                std::cout << "请输入您要连接的设备索引号 : ( 1 到 " << device_list_.nDeviceNum << " )" << std::endl;
                std::cin >> index;
            }
            device_info_ = device_list_.pDeviceInfo[index - 1];
        } else {
            device_info_ = device_list_.pDeviceInfo[0];
        }
    } else {
        std::cout << "未找到可用设备" << std::endl;
    }
    return device_list_.nDeviceNum;
}

bool HK_Camera::cameraInit(const HIKInitStruct &t) {

    int test_num = 3;
    while (device_info_ == nullptr && test_num--) {
        findConnectableUSBDevice();
        sleep(1);
    }
    if (device_info_ == nullptr && test_num == 0) return false;

    MV_CC_CreateHandle(&handle_, device_info_);
    nRet_ = MV_CC_OpenDevice(handle_);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Can not open camera! nRet_ [0x%x]\n", nRet_);
        return false;
    }

    MVCC_INTVALUE_EX nWidthMaxValue = {t.weight}, nHeightMaxValue = {t.height};

    if (t.weight <= 0) {
        //如需获取当前相机图像宽高，需要将WidthMax替换成Width
        nRet_ = MV_CC_GetIntValueEx(handle_, "WidthMax", &nWidthMaxValue);
        if (MV_OK != nRet_) {
            printf("[CameraInit] : Get WidthMax fail! nRet_ [0x%x]\n", nRet_);
        }
    }
    if (t.height <= 0) {
        //如需获取当前相机图像宽高，需要将HeightMax替换成Height
        nRet_ = MV_CC_GetIntValueEx(handle_, "HeightMax", &nHeightMaxValue);
        if (MV_OK != nRet_) {
            printf("[CameraInit] : Get HeightMax fail! nRet_ [0x%x]\n", nRet_);
        }
    }

    // 设置图像像素
    nRet_ = MV_CC_SetIntValue(handle_, "Width", nWidthMaxValue.nCurValue);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set Img Width fail [%x]\n", nRet_);
    }
    nRet_ = MV_CC_SetIntValue(handle_, "Height", nHeightMaxValue.nCurValue);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set Img Height fail [%x]\n", nRet_);
    }

    MVCC_INTVALUE_EX nFPSMaxValue = {t.fps};
    //帧率控制使能，true表示打开，false标识关闭
    nRet_ = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
    if (nRet_ != MV_OK) {
        printf("[CameraInit] : Set AcquisitionBurstFrameCountfail nRet_ [0x%x]!\n", nRet_);
    }
    //设置相机帧率，需注意不要超过相机支持的最大的帧率（相机规格书），超过了也没有意义（需要注意的是不同的像素类型支持的帧率也不同）
    nRet_ = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", t.fps);
    if (nRet_ != MV_OK) {
        printf("[CameraInit] : Set AcquisitionBurstFrameCountfail nRet_ [0x%x]!\n", nRet_);
    }


    if (t.exposure_time > 0) {
        //设置手动曝光，设置曝光时间
        nRet_ = MV_CC_SetEnumValue(handle_, "ExposureMode", 0);
        nRet_ = MV_CC_SetFloatValue(handle_, "ExposureTime", t.exposure_time);
        if (MV_OK != nRet_) {
            printf("[CameraInit] : Set ExposureTime fail nRet_ [0x%x]!\n", nRet_);
        }
    }
    //设置自动曝光
    nRet_ = MV_CC_SetEnumValue(handle_, "ExposureAuto", t.exposure_time > 0 ? 0 : 2); // 0：off 1：once 2：Continuous
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set ExposureAuto fail nRet_ [0x%x]!\n", nRet_);
    }


    //模拟增益设置
    nRet_ = MV_CC_SetFloatValue(handle_, "Gain", 1);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set Gain fail nRet_ [0x%x]\n", nRet_);
    }
    //设置自动增益
    nRet_ = MV_CC_SetEnumValue(handle_, "GainAuto", 1);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set GainAuto fail nRet_ [0x%x]!\n", nRet_);
    }


    //1.打开数字增益使能
    nRet_ = MV_CC_SetBoolValue(handle_, "GammaEnable", true);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set GammaEnable fail! nRet_ [0x%x]\n", nRet_);
    }
    //2.设置gamma类型，user：1，sRGB：2
    nRet_ = MV_CC_SetEnumValue(handle_, "GammaSelector", 1);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set GammaSelector fail! nRet_ [0x%x]\n", nRet_);
    }
    //3.设置gamma值，推荐范围0.5-2，1为线性拉伸
    nRet_ = MV_CC_SetFloatValue(handle_, "Gamma", 0.8);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set Gamma failed! nRet_ [0x%x]\n", nRet_);
    }


    //开启自动白平衡
    nRet_ = MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", true);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set BalanceWhiteAuto fail! nRet_ [0x%x]\n", nRet_);
    }


    // 关闭触发模式
    nRet_ = MV_CC_SetEnumValue(handle_, "TriggerMode", false);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set TriggerMode fail! nRet_ [0x%x]\n", nRet_);
    }

    // PixelType_Gvsp_RGB8_Packed 、 PixelType_Gvsp_BayerRG8 等
    nRet_ = MV_CC_SetEnumValue(handle_, "PixelFormat", t.video_capture_api);
    if (MV_OK != nRet_) {
        printf("[CameraInit] : Set PixelFormat fail! nRet_ [0x%x]\n", nRet_);
    }

    if (camera_mode_ == 0) {
        MV_CC_RegisterImageCallBackForRGB(handle_, call_back_ptr_, hik_camera_node_);
    }
    return true;
}

void HK_Camera::startCamera() {

    if (device_info_ == nullptr) return;
    MV_CC_StartGrabbing(handle_);

}

void HK_Camera::getFrame(cv::Mat &img) {

    if (camera_mode_ == 0) {
        std::cerr << "[CameraGetFrame] : 已设置回调函数模式，无法通过此方法获得数据" << std::endl;
    }

    MV_FRAME_OUT stImageInfo = {0};
    nRet_ = MV_CC_GetImageBuffer(handle_, &stImageInfo, 1000);
    if (nRet_ == MV_OK) {
        //释放指针 MV_CC_GetImageBuffer---&stImageInfo
        nRet_ = MV_CC_FreeImageBuffer(handle_, &stImageInfo);
        if (nRet_ != MV_OK) {
            std::cout << "[CameraGetFrame] : Free Image Buffer fail! nRet=" << nRet_ << std::endl;
        }
        img = cv::Mat(stImageInfo.stFrameInfo.nHeight, stImageInfo.stFrameInfo.nWidth, CV_8UC3, stImageInfo.pBufAddr);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    } else {
        std::cout << "[CameraGetFrame] : No data! nRet=" << nRet_ << std::endl;
    }
}

Camera::Camera (const std::string initFilePath) : camera_is_open_(false) {

    init_file_path_ = initFilePath;

    // 打开 YAML 文件
    cv::FileStorage fs(init_file_path_, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[CameraInit] : Failed to open " << init_file_path_ << std::endl;
        return;
    }

    fs["camera_type"] >> camera_type_;

    if (camera_type_ == USB) {
        fs["base_info"]["weight"] >> usb_init_.weight;
        fs["base_info"]["height"] >> usb_init_.height;
        fs["base_info"]["fps"] >> usb_init_.fps;
        fs["pixel_format"] >> usb_init_.video_capture_api;
        fs["index"] >> usb_init_.index;
        usb_init_.fourcc.clear();
        for (const auto &node: fs["fourcc"]) {
            std::string ch = node;
            usb_init_.fourcc += ch;
        }

    } else if (camera_type_ == HIK) {
        fs["base_info"]["weight"] >> hik_init_.weight;
        fs["base_info"]["height"] >> hik_init_.height;
        fs["base_info"]["fps"] >> hik_init_.fps;
        fs["hik_info"]["camera_mode"] >> hik_init_.camera_mode;
        fs["hik_info"]["exposure_time"] >> hik_init_.exposure_time;
        fs["pixel_format"] >> hik_init_.video_capture_api;

    } else {
        std::cerr << "[CameraInit] : Unexpected Camera Type " << std::endl;
        return;
    }
    // 读取 camera_matrix
    fs["camera_matrix"] >> camera_matrix_;
    // 读取 dist_coeffs_matrix
    fs["dist_coeffs_matrix"] >> dist_coeffs_matrix_;
    // 读取 "pos" 数组
    cv::FileNode RTNode = fs["RT"];
    if (RTNode.empty()) {
        std::cerr << "[CameraInit] : Failed to find RT" << std::endl;
        return;
    }
    for (const auto& node : RTNode) {
        if (node["yaw"].empty()) {
            // 读取坐标 x, y, z
            coordinate_[0] = static_cast<float>((int)node["x"]);
            coordinate_[1] = static_cast<float>((int)node["y"]);
            coordinate_[2] = static_cast<float>((int)node["z"]);
        } else {
            // 读取欧拉角 yaw, pitch, roll
            euler_angles_[0] = static_cast<float>((int)node["yaw"]);
            euler_angles_[1] = static_cast<float>((int)node["pitch"]);
            euler_angles_[2] = static_cast<float>((int)node["roll"]);
        }
    }
    // 关闭文件
    fs.release();
    //打开相机
    open();
}

Camera::~Camera() {
    stop();
}

bool Camera::getFrame(cv::Mat &img) {

    if (!camera_is_open_) {
        std::cerr << "[CameraGetFrame] : Camera has not opened! " << std::endl;
        return false;
    }
    if (camera_type_ == USB) {
        cap_->read(img);
        return true;
    } else if (camera_type_ == HIK) {
        hik_cam_->getFrame(img);
        return true;
    } else {
        std::cerr << "[CameraGetFrame] : Unexpected Camera Type " << std::endl;
        return false;
    }
}

double Camera::cameraCalibrate(const std::string imgPath) {

    // 棋盘格的尺寸：内角点数 (棋盘格的格子数 - 1)
    int board_width = 9;  // 内角点列数
    int board_height = 6; // 内角点行数
    cv::Size board_size(board_width, board_height);

    // 格子边长为 2 厘米
    float square_size = 2.0f;  // 格子边长为 2 厘米

    // 准备棋盘格的 3D 世界坐标点（单位：厘米）
    std::vector<cv::Point3f> object_points;
    for (int i = 0; i < board_height; i++) {
        for (int j = 0; j < board_width; j++) {
            object_points.push_back(cv::Point3f(j * square_size, i * square_size, 0));  // 按照格子边长计算
        }
    }

    std::vector<std::vector<cv::Point3f>> object_points_all;
    std::vector<std::vector<cv::Point2f>> image_points_all;

    // 读取标定图像
    std::vector<cv::String> images;
    cv::glob(imgPath + "*.jpg", images);

    for (size_t i = 0; i < images.size(); i++) {
        cv::Mat image = cv::imread(images[i]);
        std::vector<cv::Point2f> corners;

        // 寻找棋盘格角点
        bool found = cv::findChessboardCorners(image, board_size, corners);

        if (found) {
            image_points_all.push_back(corners);
            object_points_all.push_back(object_points);

            // 画出角点并显示
            cv::drawChessboardCorners(image, board_size, corners, found);
            cv::imshow("Chessboard", image);
            cv::waitKey(1);
        }
    }

    // 标定相机
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F); // 相机内参矩阵
    cv::Mat dist_coeffs; // 畸变系数
    std::vector<cv::Mat> rvecs, tvecs; // 旋转和平移向量

    cv::Size image_size = cv::imread(images[0]).size();
    double rms = cv::calibrateCamera(object_points_all, image_points_all, image_size,
                                     camera_matrix, dist_coeffs, rvecs, tvecs);

    // 输出结果
    std::cout << "Reprojection error: " << rms << std::endl;
    std::cout << "Camera Matrix:\n" << camera_matrix << std::endl;
    std::cout << "Distortion Coefficients:\n" << dist_coeffs << std::endl;

    cv::destroyAllWindows();

    return rms;
}

bool Camera::isOpened() {
    return camera_is_open_;
}

int Camera::getFPS() {
    if (camera_type_ == USB) {
        return usb_init_.fps;
    } else if (camera_type_ == HIK) {
        return hik_init_.fps;
    }
}

void Camera::open() {
    if (camera_is_open_) return;
    if (camera_type_ == USB) {
        cap_ = new cv::VideoCapture(usb_init_.index, usb_init_.video_capture_api);
        cap_->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(
                usb_init_.fourcc[0], usb_init_.fourcc[1], usb_init_.fourcc[2], usb_init_.fourcc[3]));
        cap_->set(cv::CAP_PROP_FRAME_WIDTH, usb_init_.weight);
        cap_->set(cv::CAP_PROP_FRAME_HEIGHT, usb_init_.height);
        cap_->set(cv::CAP_PROP_FPS, usb_init_.fps);
        if (cap_->isOpened()) camera_is_open_ = true;
    } else if (camera_type_ == HIK) {
        hik_cam_ = new HK_Camera();
        camera_is_open_ = hik_cam_->cameraInit(hik_init_);
        if (camera_is_open_) hik_cam_->startCamera();
    }
}

void Camera::stop() {
    if (!camera_is_open_) return; 

    if (camera_type_ == USB) {
        delete cap_;
        camera_is_open_ = false;
    } else if (camera_is_open_ == HIK) {
        hik_cam_->stopCamera();
        delete hik_cam_;
        camera_is_open_ = false;
    }
}
