#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctime>
#include <sstream>
#include <filesystem>  // C++17 必须的文件系统库
#include "camera_driver.h"

#define GETPHOTO TRUE

int main() {

//    std::string initFilePath = "../include/camera_driver/cameraInit/USBcamera0.yaml";
    std::string initFilePath = "../include/camera_driver/cameraInit/CalibrateCamera.yaml";

    Camera cam(initFilePath);
    if (!cam.isOpened()) {
        std::cout << "相机未打开" << std::endl;
        return -1;
    }

    // 设置保存图片的文件夹路径
    std::string folderPath = "../imgs/";
    if (!std::filesystem::exists(folderPath)) {
        std::filesystem::create_directory(folderPath);
    } else {
        std::vector<cv::String> images;
        cv::glob(folderPath + "*.jpg", images);
        if (!images.empty()) {
            for (const auto &img : images) {
                std::filesystem::remove(img);
            }
        }
    }

    cv::Mat frame;
    cv::Mat lastCapturedFrame;  // 存储上一次拍摄的图片
    std::string lastSavedFile;  // 存储最近保存的文件路径

    std::cout << "按 'n' 取消保存，按 'p' 保存并拍摄下一帧，按下 'q' 键退出。" << std::endl;

    while (true) {
        cam.getFrame(frame);  // 获取一帧图像
        if (frame.empty()) {
            std::cerr << "捕获的图像为空！" << std::endl;
            continue;  // 跳过当前帧，重新捕获
        }

        // 显示实时相机画面
        cv::imshow("Camera", frame);

        char key = cv::waitKey(1);

        if (key == 'p' || key == 'P') {
            // 第一次按下 'p' 时缓存当前帧并显示
            if (!lastCapturedFrame.empty()) { // 如果之前已经拍照并且图片未保存，保存上次拍摄的图片

                std::time_t now = std::time(0);
                std::stringstream filename;
                filename << folderPath << "photo_" << now << ".jpg";

                std::cout << "准备保存图片: " << filename.str() << std::endl;

                if (cv::imwrite(filename.str(), lastCapturedFrame)) {
                    lastSavedFile = filename.str();
                    std::cout << "上次拍摄的图片已保存为 " << lastSavedFile << std::endl;
                } else {
                    std::cerr << "图片保存失败！" << std::endl;
                }

            }
            lastCapturedFrame = frame.clone();  // 缓存当前帧
            cv::imshow("Captured Photo", lastCapturedFrame);  // 显示缓存的图像
            std::cout << "已缓存当前帧，按 'n' 取消保存，按 'p' 保存并拍摄下一帧，按下 'q' 键退出。" << std::endl;
        }

        if (key == 'n' || key == 'N') {
            if (!lastCapturedFrame.empty()) {
                // 清空缓存
                lastCapturedFrame.release();  // 释放缓存中的图像
                std::cout << "取消保存当前图片，缓存已清空。" << std::endl;
                cv::destroyWindow("Captured Photo");  // 销毁窗口
            }
        }

        // 按下 'q' 键退出
        if (key == 'q' || key == 'Q') {
            break;
        }
    }

    cv::destroyAllWindows();

    std::vector<cv::String> images;
    cv::glob(folderPath + "*.jpg", images);
    if (images.size() > 0) {
        std::cout << "已读取共 " << images.size() << " 张图片，正在标定..." << std::endl;
        cam.cameraCalibrate(folderPath);
    } else {
        std::cerr << "未拍摄并保存图像" << std::endl;
    }

    for (const auto &img : images) {
        std::filesystem::remove(img);
    }
    std::filesystem::remove(folderPath);

    return 0;
}
