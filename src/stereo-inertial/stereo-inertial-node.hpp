#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <cv_bridge/cv_bridge.h>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <utility>

#include <rclcpp/qos.hpp>
#include <rcl/time.h>

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
    void on_imu(const ImuMsg::SharedPtr msg);
    void on_left(const ImageMsg::SharedPtr msgLeft);
    void on_right(const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);

    void push_and_try_(std::deque<ImageMsg::SharedPtr> &queue, const ImageMsg::SharedPtr &msg);
    bool find_nearest_locked_(const std::deque<ImageMsg::SharedPtr> &queue,
                              const rclcpp::Time &target,
                              ImageMsg::SharedPtr &matched) const;
    void try_match_and_feed_();
    bool gather_imu_locked_(const rclcpp::Time &t_left,
                            const rclcpp::Time &t_prev,
                            std::vector<ImuMsg::SharedPtr> &collected,
                            bool &tail_within_margin,
                            double &imu_span_ms);
    void publish_pair_(const ImageMsg::SharedPtr &left,
                       const ImageMsg::SharedPtr &right,
                       const std::vector<ORB_SLAM3::IMU::Point> &imu_points,
                       bool tail_within_margin,
                       double imu_span_ms);

    static int64_t to_ns(const builtin_interfaces::msg::Time &t);
    static builtin_interfaces::msg::Time from_ns(int64_t ns);

    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgLeft_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImgRight_;

    ORB_SLAM3::System *SLAM_;

    // Buffers and synchronization
    std::deque<ImageMsg::SharedPtr> left_queue_;
    std::deque<ImageMsg::SharedPtr> right_queue_;
    std::deque<ImuMsg::SharedPtr> imu_queue_;
    mutable std::mutex data_mutex_;
    mutable std::condition_variable imu_cv_;
    rclcpp::Time last_frame_stamp_{0, 0, RCL_ROS_TIME};

    // Parameters
    double max_lr_diff_ms_ = 10.0;
    double imu_recent_epsilon_ms_ = 1.0;
    double min_imu_span_ms_ = 20.0;
    double imu_wait_timeout_ms_ = 50.0;
    std::size_t max_image_queue_size_ = 60;
    std::size_t max_imu_queue_size_ = 400;

    bool doRectify_;
    bool doEqual_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
