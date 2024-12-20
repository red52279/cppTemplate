#include "apps.h"

TrackbarWindow::TrackbarWindow(std::string windowName)
        : name_(std::move(windowName)), is_init_(false) {}

TrackbarWindow::~TrackbarWindow() {
    if (is_init_) cv::destroyWindow(name_);
}

void TrackbarWindow::tackBarRegister(const std::string &name, int min_val, int max_val, int init_val) {
    if (init_val < min_val || init_val > max_val) {
        std::cerr << "Error: init_val must be within [min_val, max_val].\n";
        return;
    }
    auto it_ = trackbar_.find(name);
    if (it_ == trackbar_.end()) {
        trackbar_.emplace(name, std::make_tuple(min_val, max_val, init_val));
    } else {
        it_->second = std::make_tuple(min_val, max_val, init_val);
    }
}

void TrackbarWindow::tackBarInit() {
    cv::namedWindow(name_);
    for (auto &it_ : trackbar_) {
        const auto &[min_val, max_val, current_val] = it_.second;

        // OpenCV's trackbar position range starts at 0, so we shift the range
        int range = max_val - min_val;
        cv::createTrackbar(it_.first, name_, nullptr, range, nullptr);
        cv::setTrackbarPos(it_.first, name_, current_val - min_val); // Set initial position
    }
    is_init_ = true;
}

int TrackbarWindow::getTrackBarData(const std::string &name) {
    auto it_ = trackbar_.find(name);
    if (it_ == trackbar_.end()) return 0;

    const auto &[min_val, max_val, current_val] = it_->second;

    // Get the current position from OpenCV and add the minimum offset
    int pos = cv::getTrackbarPos(name, name_);
    return min_val + pos;
}

void TrackbarWindow::imgShow(cv::Mat &img) {
    cv::imshow(name_, img);
}



double calculateEllipseError(const cv::Point2f& point, const cv::RotatedRect& ellipse) {
    // 计算点到椭圆中心的距离
    cv::Point2f center = ellipse.center;
    double a = ellipse.size.width / 2;  // 长轴
    double b = ellipse.size.height / 2; // 短轴
    double distance = cv::norm(point - center);  // 点到椭圆中心的距离

    // 假设误差计算方法是根据椭圆的长短轴来衡量
    double error = std::abs(distance - (a + b) / 2); // 示例误差计算

    return error;
}

double calculateEllipseCoverage(const cv::RotatedRect& ellipse, const cv::Mat& ranged_img) {
    // 创建与输入图像大小相同的掩膜，用于绘制椭圆
    cv::Mat ellipseMask = cv::Mat::zeros(ranged_img.size(), CV_8U);
    cv::ellipse(ellipseMask, ellipse, cv::Scalar(255), -1); // 绘制椭圆掩膜

    // 计算椭圆的理论面积
    double ellipse_area = CV_PI * (ellipse.size.width / 2.0) * (ellipse.size.height / 2.0);

    // 计算椭圆内部实际覆盖的像素点数量
    double covered_area = cv::countNonZero(ranged_img & ellipseMask);

    // 防止除零错误
    if (ellipse_area <= 0) {
        return 0.0;
    }

    // 返回椭圆内部的覆盖比例
    return covered_area / ellipse_area;
}





ColorRange::ColorRange (std::string window_name, int H_low, int S_low, int V_low, int H_high, int S_high, int V_high,
                        int first_erode_time, int first_dilate_time, int second_erode_time,
                        int second_dilate_time, int first_size, int second_size) :
        TrackbarWindow(std::move(window_name)), first_erode_time_(first_erode_time),
        second_erode_time_(second_erode_time), first_dilate_time_(first_dilate_time),
        second_dilate_time_(second_dilate_time), first_size_(first_size), second_size_(second_size),
        obj_center_que_(5, 2), color_circle_center_que_(5, 2) {
    addColorRange(H_low, S_low, V_low, H_high, S_high, V_high);
    tackBarRegister(HighNameH, MAX_H, H_high);
    tackBarRegister(HighNameS, MAX_S, S_high);
    tackBarRegister(HighNameV, MAX_V, V_high);
    tackBarRegister(LowNameH, MAX_H, H_low);
    tackBarRegister(Low_NameS, MAX_S, S_low);
    tackBarRegister(LowNameV, MAX_V, V_low);
    first_size = std::max(first_size, 2);
    second_size = std::max(second_size, 2);
    tackBarRegister(FirstErodeTimeName, 10, first_erode_time);
    tackBarRegister(FirstDilateTimeName, 10, first_dilate_time);
    tackBarRegister(FirstEDSizeName, 9, first_size);
    tackBarRegister(SecondErodeTimeName, 10, second_erode_time);
    tackBarRegister(SecondDilateTimeName, 10, second_dilate_time);
    tackBarRegister(SecondEDSizeName, 9, second_size);
    tackBarInit();
    cv::setTrackbarMin(FirstEDSizeName, name_, 2);
    cv::setTrackbarMin(SecondEDSizeName, name_, 2);
}

ColorRange::ColorRange (std::string window_name, const cv::Scalar &upper_color_range, const cv::Scalar &lower_color_range,
                        int first_erode_time, int first_dilate_time, int second_erode_time,
                        int second_dilate_time, int first_size, int second_size) :
        TrackbarWindow(std::move(window_name)), first_erode_time_(first_erode_time),
        second_erode_time_(second_erode_time), first_dilate_time_(first_dilate_time),
        second_dilate_time_(second_dilate_time), first_size_(first_size), second_size_(second_size),
        obj_center_que_(5, 2), color_circle_center_que_(5, 2) {
    addColorRange(upper_color_range, lower_color_range);
    tackBarRegister(HighNameH, MAX_H, static_cast<int>(upper_color_range[0]));
    tackBarRegister(HighNameS, MAX_S, static_cast<int>(upper_color_range[1]));
    tackBarRegister(HighNameV, MAX_V, static_cast<int>(upper_color_range[2]));
    tackBarRegister(LowNameH, MAX_H, static_cast<int>(lower_color_range[0]));
    tackBarRegister(Low_NameS, MAX_S, static_cast<int>(lower_color_range[1]));
    tackBarRegister(LowNameV, MAX_V, static_cast<int>(lower_color_range[2]));
    first_size = std::max(first_size, 2);
    second_size = std::max(second_size, 2);
    tackBarRegister(FirstErodeTimeName, 10, first_erode_time);
    tackBarRegister(FirstDilateTimeName, 10, first_dilate_time);
    tackBarRegister(FirstEDSizeName, 9, first_size);
    tackBarRegister(SecondErodeTimeName, 10, second_erode_time);
    tackBarRegister(SecondDilateTimeName, 10, second_dilate_time);
    tackBarRegister(SecondEDSizeName, 9, second_size);
    tackBarInit();
    cv::setTrackbarMin(FirstEDSizeName, name_, 2);
    cv::setTrackbarMin(SecondEDSizeName, name_, 2);
}
void ColorRange::addColorRange(int H_low, int S_low, int V_low, int H_high, int S_high, int V_high) {
    color_ranges_.emplace_back(cv::Scalar(H_high, S_high, V_high),
                               cv::Scalar(H_low, S_low, V_low));
}
void ColorRange::addColorRange(const cv::Scalar &UpperColorRange, const cv::Scalar &LowerColorRange) {
    color_ranges_.emplace_back(UpperColorRange, LowerColorRange);
}
void ColorRange::replaceColorRange(const cv::Scalar &UpperColorRange, const cv::Scalar &LowerColorRange, int index) {
    if (index < 0 || index > color_ranges_.size() - 1) return;
    this->color_ranges_[index] = std::make_pair(UpperColorRange, LowerColorRange);
}
void ColorRange::replaceColorRange(int H_low, int S_low, int V_low, int H_high, int S_high, int V_high, int index) {
    if (index < 0 || index > color_ranges_.size() - 1) return;
    this->color_ranges_[index] = std::make_pair(cv::Scalar(H_high, S_high, V_high),
                                                cv::Scalar(H_low, S_low, V_low));
}
void ColorRange::updateTackBarData() {
    int H_high = getTrackBarData(HighNameH);
    int S_high = getTrackBarData(HighNameS);
    int V_high = getTrackBarData(HighNameV);
    int H_low = getTrackBarData(LowNameH);
    int S_low = getTrackBarData(Low_NameS);
    int V_low = getTrackBarData(LowNameV);
    first_size_ = getTrackBarData(FirstEDSizeName);
    second_size_ = getTrackBarData(SecondEDSizeName);
    first_erode_time_ = getTrackBarData(FirstErodeTimeName);
    first_dilate_time_ = getTrackBarData(FirstDilateTimeName);
    second_erode_time_ = getTrackBarData(SecondErodeTimeName);
    second_dilate_time_ = getTrackBarData(SecondDilateTimeName);
    replaceColorRange(H_low, S_low, V_low, H_high, S_high, V_high);
}
void ColorRange::imgColorRange(cv::Mat &cap_img, cv::Mat &ranged_img) {
    cv::Mat hsv_cap_img, img;
    cv::cvtColor(cap_img, hsv_cap_img, cv::COLOR_BGR2HSV);
    for (int i = 0; i < color_ranges_.size(); i++) {
        cv::inRange(hsv_cap_img, color_ranges_[i].second, color_ranges_[i].first, img);
        if (!i) ranged_img = cv::Mat::zeros(img.size(), img.type());
        bitwise_or(img, ranged_img, ranged_img);
    }
    for (int i = 0; i < first_dilate_time_; i++)
        morphologyEx(ranged_img, ranged_img, cv::MORPH_DILATE,
                     cv::getStructuringElement(0, cv::Size(first_size_, first_size_)));
    for (int i = 0; i < first_erode_time_; i++)
        morphologyEx(ranged_img, ranged_img, cv::MORPH_ERODE,
                     cv::getStructuringElement(0, cv::Size(first_size_, first_size_)));
    for (int i = 0; i < second_dilate_time_; i++)
        morphologyEx(ranged_img, ranged_img, cv::MORPH_DILATE,
                     cv::getStructuringElement(0, cv::Size(second_size_, second_size_)));
    for (int i = 0; i < second_erode_time_; i++)
        morphologyEx(ranged_img, ranged_img, cv::MORPH_ERODE,
                     cv::getStructuringElement(0, cv::Size(second_size_, second_size_)));
    imgShow(ranged_img);
}


void ColorRange::setED(int first_erode_time, int second_erode_time, int first_dilate_time, int second_dilate_time,
                       int first_size, int second_size) {
    first_erode_time_ = first_erode_time;
    first_dilate_time_ = first_dilate_time;
    first_size_ = first_size;
    second_erode_time_ = second_erode_time;
    second_dilate_time_ = second_dilate_time;
    second_size_ = second_size;
}







cv::Vec3f ColorRange::getWorldCoordinates(cv::Vec3f &t_vec, float yaw, float pitch, float roll, const cv::Vec3f &translation) {

    float yaw_rad = yaw * CV_PI / 180.0;
    float pitch_rad = pitch * CV_PI / 180.0;
    float roll_rad = roll * CV_PI / 180.0;

    // Yaw 旋转矩阵
    cv::Mat R_yaw = (cv::Mat_<float>(3, 3) <<
                                           cos(yaw_rad),  0, sin(yaw_rad),
            0,             1,          0,
            -sin(yaw_rad), 0, cos(yaw_rad));

    // Pitch 旋转矩阵
    cv::Mat R_pitch = (cv::Mat_<float>(3, 3) <<
                                             1, 0,              0,
            0, cos(pitch_rad), -sin(pitch_rad),
            0, sin(pitch_rad), cos(pitch_rad));

    // Roll 旋转矩阵
    cv::Mat R_roll = (cv::Mat_<float>(3, 3) <<
                                            cos(roll_rad), -sin(roll_rad), 0,
            sin(roll_rad), cos(roll_rad),  0,
            0,             0,              1);

    // 总旋转矩阵 R
    cv::Mat R = R_yaw * R_pitch * R_roll;

    // 输入向量转换为矩阵
    cv::Mat t_vec_mat = (cv::Mat_<float>(3, 1) << t_vec[0], t_vec[1], t_vec[2]);

    // 旋转变换
    cv::Mat rotated = R * t_vec_mat;

    cv::Vec3f world_coordinates(
            rotated.at<float>(0, 0) + translation[0],
            rotated.at<float>(1, 0) + translation[1],
            rotated.at<float>(2, 0) + translation[2]
    );

    return world_coordinates;
}

void ColorRange::getOIPoints(const cv::RotatedRect &obj_rect, int cluster_count,
                             std::vector<cv::Point2f> &img_points, std::vector<cv::Point3f> &obj_points) {
    if (cluster_count == 1) { // 在顶面

        // 提取椭圆参数
        cv::Point2f center = obj_rect.center;     // 椭圆中心
        float majorAxis = obj_rect.size.width / 2.0f;  // 椭圆长轴半径
        float minorAxis = obj_rect.size.height / 2.0f; // 椭圆短轴半径
        float angle = obj_rect.angle;            // 旋转角度（度）

        // 转换角度为弧度
        float theta = angle * CV_PI / 180.0;

        int num_points = 50; // 生成点数量

        // 生成 img_points（椭圆上的点）
        for (int i = 0; i < num_points; ++i) {
            // 计算标准圆上的点
            float t = CV_2PI * i / num_points; // 参数 t 表示椭圆上的角度
            float x = majorAxis * cos(t);        // 未旋转时，x 在椭圆上的位置
            float y = minorAxis * sin(t);        // 未旋转时，y 在椭圆上的位置

            // 应用旋转变换
            float rotated_x = x * cos(theta) - y * sin(theta);
            float rotated_y = x * sin(theta) + y * cos(theta);

            // 平移到椭圆的中心
            img_points.emplace_back(rotated_x + center.x, rotated_y + center.y);
        }

        cv::Rect obj_rect = cv::boundingRect(img_points);
        img_points.clear();
        float center_x = obj_rect.x + obj_rect.width / 2,
                center_y = obj_rect.y + obj_rect.height / 2;
        img_points.emplace_back(static_cast<float>(center_x + obj_rect.width / 2), center_y);
        img_points.emplace_back(center_x, static_cast<float>(center_y - obj_rect.height / 2));
        img_points.emplace_back(static_cast<float>(center_x - obj_rect.width / 2), center_y);
        img_points.emplace_back(center_x, static_cast<float>(center_y + obj_rect.height / 2));

        float radius = 2.5f;  // 圆的半径
        // 生成圆形上的点 (z=0平面)
        for (int i = 0; i < 4; ++i) {
            float angle = (CV_2PI * i) / 4;
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            if (std::abs(x) < 1e-6) x = 0;
            if (std::abs(y) < 1e-6) y = 0;
            obj_points.emplace_back(x, y, 0);  // z=0
        }
    } else {

        // 步骤 1: 生成 obj_points（圆形点集）
        float radius = 2.5f;  // 圆的半径
        int num_points = 4; // 生成点数量

        // 生成圆形上的点 (z=0平面)
        for (int i = 0; i < num_points; ++i) {
            float angle = (CV_2PI * i) / num_points;
            float x = radius * cos(angle);
            float y = radius * sin(angle);
            if (std::abs(x) < 1e-6) x = 0;
            if (std::abs(y) < 1e-6) y = 0;
            obj_points.emplace_back(x, y, 0);  // z=0
        }

        // 提取椭圆参数
        cv::Point2f center = obj_rect.center;     // 椭圆中心
        float majorAxis = obj_rect.size.width / 2.0f;  // 椭圆长轴半径
        float minorAxis = obj_rect.size.height / 2.0f; // 椭圆短轴半径
        float angle = obj_rect.angle;            // 旋转角度（度）

        // 转换角度为弧度
        float theta = angle * CV_PI / 180.0;

        // 生成 img_points（椭圆上的点）
        for (int i = 0; i < num_points; ++i) {
            // 计算标准圆上的点
            float t = CV_2PI * i / num_points; // 参数 t 表示椭圆上的角度
            float x = majorAxis * cos(t);        // 未旋转时，x 在椭圆上的位置
            float y = minorAxis * sin(t);        // 未旋转时，y 在椭圆上的位置

            // 应用旋转变换
            float rotated_x = x * cos(theta) - y * sin(theta);
            float rotated_y = x * sin(theta) + y * cos(theta);

            // 平移到椭圆的中心
            img_points.emplace_back(rotated_x + center.x, rotated_y + center.y);
        }

    }
}


void smallImgShow(std::string win_name, const cv::Mat &img) {
    cv::Mat small_img;
    cv::resize(img, small_img, cv::Size(debug_img_w, debug_img_h));
    cv::imshow(win_name, small_img);
}