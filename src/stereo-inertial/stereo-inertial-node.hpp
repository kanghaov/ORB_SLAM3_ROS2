#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>
#include <map>
#include <mutex>
#include <queue>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual);
    ~StereoInertialNode();

private:
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImageLeft(const ImageMsg::SharedPtr msgLeft);
    void GrabImageRight(const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);

    void try_match_from_left_(int64_t tL);
    void try_match_from_right_(int64_t tR);
    void publish_pair_(ImageMsg::ConstSharedPtr left_in, ImageMsg::ConstSharedPtr right_in, int64_t dt_ns);
    void gc_old_();

    static int64_t to_ns(const builtin_interfaces::msg::Time &t);
    static builtin_interfaces::msg::Time from_ns(int64_t ns);

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;

    ORB_SLAM3::System *SLAM_;

    // IMU
    queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutex_;

    // Image pairing
    std::map<int64_t, ImageMsg::ConstSharedPtr> left_q_;
    std::map<int64_t, ImageMsg::ConstSharedPtr> right_q_;
    std::mutex bufMutexLeft_, bufMutexRight_;

    int64_t max_diff_ns_ = 10 * 1000 * 1000;   // 10 ms
    int64_t gc_window_ns_ = 500 * 1000 * 1000; // 500 ms
    int64_t image_time_shift_ns_ = 0;
    bool force_right_stamp_to_left_ = false;

    uint64_t kept_ = 0;
    uint64_t drop_left_gc_ = 0;
    uint64_t drop_right_gc_ = 0;
    double last_dt_ms_ = 0.0;
    int64_t last_imu_ns_ = 0;

    bool doRectify_;
    bool doEqual_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
