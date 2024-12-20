#ifndef NUTEIA_APPS_H
#define NUTEIA_APPS_H

#include <opencv2/opencv.hpp>
#include <numeric>

constexpr int debug_img_w = 1260;
constexpr int debug_img_h = 960;

void smallImgShow(std::string win_name, const cv::Mat &img);

class TrackbarWindow {
public:
    TrackbarWindow(std::string windowName);
    ~TrackbarWindow();
    void tackBarRegister(const std::string &name, int min_val, int max_val, int init_val = 0);
    void tackBarInit();
    int getTrackBarData(const std::string &name);
    void imgShow(cv::Mat &img);
    std::string name_;

private:
    bool is_init_;
    std::map<std::string, std::tuple<int, int, int>> trackbar_; // (min_val, max_val, current_val)
};


template <typename PointT>
class LimitedDataQueue {
private:
    std::deque<PointT> dataQueue;       // 数据队列
    size_t maxSize;                     // 队列大小限制
    float varianceThreshold;            // 方差阈值
    size_t dimension;                   // 点的维度
    std::vector<float> sumPerDim;       // 每个维度的总和
    std::vector<float> variancePerDim;  // 每个维度的方差

    // 检查点类型是否具有 z 分量
    static constexpr bool hasZ = std::is_same_v<PointT, cv::Point3f>;
    static constexpr bool hasY = std::is_same_v<PointT, cv::Point2f> || hasZ;

    // 获取点的维度
    static constexpr size_t getDimension() {
        if constexpr (hasZ) {
            return 3; // 3D 点
        } else if constexpr (hasY) {
            return 2; // 2D 点
        } else {
            return 1;
        }
    }

    // 访问点的分量
    static float getComponent(const PointT& point, size_t index) {
        switch (index) {
            case 0:
                if constexpr (hasY || hasZ) {
                    return point.x;
                } else {
                    return point;
                }
            case 1:
                if constexpr (hasZ) {

                    return point.y;
                } else {
                    throw std::out_of_range("Accessing z component of a 1D point.");
                }
            case 2:
                if constexpr (hasZ) {
                    return point.z;
                } else {
                    throw std::out_of_range("Accessing z component of a 2D point.");
                }
            default: throw std::out_of_range("Index out of range for PointT");
        }
    }

    // 添加点时更新统计量
    void addPoint(const PointT& point) {
        size_t n = dataQueue.size();
        if (n > 0) {
            for (size_t i = 0; i < dimension; ++i) {
                float meanOld = sumPerDim[i] / n;
                sumPerDim[i] += getComponent(point, i);
                float meanNew = sumPerDim[i] / (n + 1);
                variancePerDim[i] = variancePerDim[i] * n / (n + 1)
                                    + (getComponent(point, i) - meanNew) * (getComponent(point, i) - meanNew)
                                    - (meanOld - meanNew) * (meanOld - meanNew);
            }
        } else {
            for (size_t i = 0; i < dimension; ++i) {
                sumPerDim[i] = getComponent(point, i);
                variancePerDim[i] = 0.0f;
            }
        }
        dataQueue.push_back(point);

        // 确保方差非负
        for (auto& v : variancePerDim) {
            v = std::max(v, 0.0f);
        }
    }

    // 移除点时更新统计量
    void removePoint() {
        if (dataQueue.empty()) return;

        const PointT& point = dataQueue.front();
        size_t n = dataQueue.size();
        for (size_t i = 0; i < dimension; ++i) {
            float meanOld = sumPerDim[i] / n;
            sumPerDim[i] -= getComponent(point, i);
            if (n > 1) {
                float meanNew = sumPerDim[i] / (n - 1);
                variancePerDim[i] = variancePerDim[i] * n / (n - 1)
                                    - (getComponent(point, i) - meanNew) * (getComponent(point, i) - meanNew)
                                    + (meanOld - meanNew) * (meanOld - meanNew);
            } else {
                variancePerDim[i] = 0.0f;
            }
        }
        dataQueue.pop_front();

        // 确保方差非负
        for (auto& v : variancePerDim) {
            v = std::max(v, 0.0f);
        }
    }

    // 重置队列
    void reset() {
        dataQueue.clear();
        sumPerDim.assign(dimension, 0.0f);
        variancePerDim.assign(dimension, 0.0f);
    }

public:
    // 构造函数
    LimitedDataQueue(size_t size, float threshold)
            : maxSize(size),
              varianceThreshold(threshold),
              dimension(getDimension()),
              sumPerDim(dimension, 0.0f),
              variancePerDim(dimension, 0.0f) {}

    // 添加新数据，返回是否满足条件和结果
    std::pair<bool, PointT> addData(const PointT& newData, bool hasInput) {
        if (!hasInput) {
            reset();
            return {false, PointT()};
        }

        if (dataQueue.size() == maxSize) {
            removePoint();
        }

        addPoint(newData);

        if (dataQueue.size() == maxSize) {
            bool varianceSatisfied = true;
            for (const auto& v : variancePerDim) {
                if (v >= varianceThreshold) {
                    varianceSatisfied = false;
                    break;
                }
            }
            if (varianceSatisfied) {
                PointT mean{};
                for (size_t i = 0; i < dimension; ++i) {
                    switch (i) {
                        case 0:
                            if constexpr (hasY || hasZ) {
                                mean.x = sumPerDim[i] / dataQueue.size();
                                break;
                            } else {
                                mean = sumPerDim[i] / dataQueue.size();
                                break;
                            }
                        case 1:
                            if constexpr (hasY) {
                                mean.y = sumPerDim[i] / dataQueue.size();
                            }
                            break;
                        case 2:
                            if constexpr (hasZ) {
                                mean.z = sumPerDim[i] / dataQueue.size();
                            }
                            break;
                    }
                }
                return {true, mean};
            }
        }

        return {false, PointT()};
    }
};




#define MAX_H 180
#define MAX_S 255
#define MAX_V 255
#define HighNameH "H_high"
#define HighNameS "S_high"
#define HighNameV "V_high"
#define LowNameH "H_low"
#define Low_NameS "S_low"
#define LowNameV "V_low"
#define FirstErodeTimeName "|FirstErodeTimes|"
#define FirstDilateTimeName "|FirstDilateTimes|"
#define SecondErodeTimeName "|SecondErodeTimes|"
#define SecondDilateTimeName "|SecondDilateTimes|"
#define FirstEDSizeName "|FirstSize|"
#define SecondEDSizeName "|SecondSize|"

double calculateEllipseError(const cv::Point2f& point, const cv::RotatedRect& ellipse);
double calculateEllipseCoverage(const cv::RotatedRect& ellipse, const cv::Mat& binary_image);

class ColorRange : TrackbarWindow {
public:
    ColorRange (std::string window_name, int H_low, int S_low, int V_low, int H_high, int S_high, int V_high,
                int first_erode_time = 0, int first_dilate_time = 0, int second_erode_time = 0,
                int second_dilate_time = 0, int first_size = 2, int second_size = 2);
    ColorRange (std::string window_name, const cv::Scalar &upper_color_range, const cv::Scalar &lower_color_range,
                int first_erode_time = 0, int first_dilate_time = 0, int second_erode_time = 0,
                int second_dilate_time = 0, int first_size = 2, int second_size = 2);
    void addColorRange(int H_low, int S_low, int V_low, int H_high, int S_high, int V_high);
    void addColorRange(const cv::Scalar &UpperColorRange, const cv::Scalar &LowerColorRange);
    void replaceColorRange(const cv::Scalar &UpperColorRange, const cv::Scalar &LowerColorRange, int index = 0);
    void replaceColorRange(int H_low, int S_low, int V_low, int H_high, int S_high, int V_high, int index = 0);
    void updateTackBarData();
    void imgColorRange(cv::Mat &cap_img, cv::Mat &ranged_img);
    void setED(int first_erode_time, int second_erode_time, int first_dilate_time,
               int second_dilate_time, int first_size, int second_size);
    static cv::Vec3f getWorldCoordinates(cv::Vec3f &t_vec, float yaw, float pitch, float roll, const cv::Vec3f &translation = {0, 0, 0});
private:
    void getOIPoints(const cv::RotatedRect &obj_rect, int cluster_count, std::vector<cv::Point2f> &img_points,
                     std::vector<cv::Point3f> &obj_points);
    std::vector<std::pair<cv::Scalar, cv::Scalar>> color_ranges_;
    int first_erode_time_, second_erode_time_;
    int first_dilate_time_, second_dilate_time_;
    int first_size_, second_size_;
    LimitedDataQueue<cv::Point2f> obj_center_que_;
    LimitedDataQueue<cv::Point2f> color_circle_center_que_;

};



#endif //NUTEIA_APPS_H
