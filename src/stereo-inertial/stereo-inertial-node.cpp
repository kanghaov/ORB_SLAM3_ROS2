#include "stereo-inertial-node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#include <rmw/qos_profiles.h>
#include <opencv2/core/core.hpp>

using std::placeholders::_1;

namespace
{
inline rclcpp::QoS make_reliable_qos(std::size_t depth)
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(depth));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    return qos;
}

inline rclcpp::QoS q_img()
{
    return make_reliable_qos(30);
}

inline rclcpp::QoS q_imu()
{
    return make_reliable_qos(400);
}

inline double duration_ms(const rclcpp::Duration &d)
{
    return static_cast<double>(d.nanoseconds()) / 1e6;
}
} // namespace

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM,
                                       const string &strSettingsFile,
                                       const string &strDoRectify,
                                       const string &strDoEqual)
    : Node("ORB_SLAM3_ROS2"),
      SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            std::cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << std::endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    (void)this->declare_parameter<bool>("use_sensor_data_qos", false);
    (void)this->declare_parameter<bool>("approximate_sync", false);
    (void)this->declare_parameter<int>("sync_queue_size", 0);
    (void)this->declare_parameter<bool>("visualization", true);
    (void)this->declare_parameter<bool>("force_right_stamp_to_left", true);

    max_lr_diff_ms_ = std::max(0.1, this->declare_parameter<double>("max_lr_diff_ms", max_lr_diff_ms_));
    imu_recent_epsilon_ms_ = std::max(0.1, this->declare_parameter<double>("imu_tail_epsilon_ms", imu_recent_epsilon_ms_));
    min_imu_span_ms_ = std::max(0.0, this->declare_parameter<double>("min_imu_span_ms", min_imu_span_ms_));
    imu_wait_timeout_ms_ = std::max(1.0, this->declare_parameter<double>("imu_wait_timeout_ms", imu_wait_timeout_ms_));
    const int declared_img_queue = this->declare_parameter<int>("max_image_queue_size", static_cast<int>(max_image_queue_size_));
    const int declared_imu_queue = this->declare_parameter<int>("max_imu_queue_size", static_cast<int>(max_imu_queue_size_));
    const int max_img_queue = std::max(1, declared_img_queue);
    const int max_imu_queue = std::max(1, declared_imu_queue);
    max_image_queue_size_ = static_cast<std::size_t>(max_img_queue);
    max_imu_queue_size_ = static_cast<std::size_t>(max_imu_queue);

    subImu_ = this->create_subscription<ImuMsg>("imu", q_imu(), std::bind(&StereoInertialNode::on_imu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", q_img(), std::bind(&StereoInertialNode::on_left, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", q_img(), std::bind(&StereoInertialNode::on_right, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "pairing: lr_diff_ms<=%.2f imu_tail_eps_ms<=%.2f imu_span_ms>=%.2f imu_wait_ms=%.2f queues(img=%zu imu=%zu)",
                max_lr_diff_ms_,
                imu_recent_epsilon_ms_,
                min_imu_span_ms_,
                imu_wait_timeout_ms_,
                max_image_queue_size_,
                max_imu_queue_size_);
}

StereoInertialNode::~StereoInertialNode()
{
    SLAM_->Shutdown();
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::on_imu(const ImuMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    imu_queue_.push_back(msg);
    while (imu_queue_.size() > max_imu_queue_size_)
    {
        imu_queue_.pop_front();
    }
    imu_cv_.notify_all();
}

void StereoInertialNode::on_left(const ImageMsg::SharedPtr msgLeft)
{
    push_and_try_(left_queue_, msgLeft);
}

void StereoInertialNode::on_right(const ImageMsg::SharedPtr msgRight)
{
    push_and_try_(right_queue_, msgRight);
}

void StereoInertialNode::push_and_try_(std::deque<ImageMsg::SharedPtr> &queue, const ImageMsg::SharedPtr &msg)
{
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        queue.push_back(msg);
        while (queue.size() > max_image_queue_size_)
        {
            queue.pop_front();
        }
    }
    try_match_and_feed_();
}

bool StereoInertialNode::find_nearest_locked_(const std::deque<ImageMsg::SharedPtr> &queue,
                                              const rclcpp::Time &target,
                                              ImageMsg::SharedPtr &matched) const
{
    if (queue.empty())
    {
        return false;
    }

    double best_dt_ms = std::numeric_limits<double>::max();
    ImageMsg::SharedPtr best;
    for (const auto &msg : queue)
    {
        const double dt_ms = std::abs(duration_ms(rclcpp::Time(msg->header.stamp) - target));
        if (dt_ms < best_dt_ms)
        {
            best_dt_ms = dt_ms;
            best = msg;
        }
    }

    if (best && best_dt_ms <= max_lr_diff_ms_)
    {
        matched = best;
        return true;
    }
    return false;
}

bool StereoInertialNode::gather_imu_locked_(const rclcpp::Time &t_left,
                                            const rclcpp::Time &t_prev,
                                            std::vector<ImuMsg::SharedPtr> &collected,
                                            bool &tail_within_margin,
                                            double &imu_span_ms)
{
    collected.clear();
    tail_within_margin = false;
    imu_span_ms = 0.0;

    while (!imu_queue_.empty() && rclcpp::Time(imu_queue_.front()->header.stamp) <= t_prev)
    {
        imu_queue_.pop_front();
    }

    for (const auto &imu : imu_queue_)
    {
        const rclcpp::Time t_imu(imu->header.stamp);
        if (t_imu > t_left)
        {
            break;
        }
        collected.push_back(imu);
    }

    if (!collected.empty())
    {
        const rclcpp::Time first(collected.front()->header.stamp);
        const rclcpp::Time last(collected.back()->header.stamp);
        imu_span_ms = duration_ms(last - first);
        tail_within_margin = duration_ms(t_left - last) <= imu_recent_epsilon_ms_;
    }

    while (!imu_queue_.empty() && rclcpp::Time(imu_queue_.front()->header.stamp) <= t_left)
    {
        imu_queue_.pop_front();
    }

    return !collected.empty();
}

void StereoInertialNode::try_match_and_feed_()
{
    ImageMsg::SharedPtr left;
    ImageMsg::SharedPtr right;
    rclcpp::Time t_left;
    std::vector<ImuMsg::SharedPtr> imu_msgs;
    bool tail_ok = false;
    double imu_span_ms = 0.0;

    std::unique_lock<std::mutex> lock(data_mutex_);
    if (left_queue_.empty() || right_queue_.empty())
    {
        return;
    }

    left = left_queue_.back();
    t_left = rclcpp::Time(left->header.stamp);

    ImageMsg::SharedPtr right_candidate;
    if (!find_nearest_locked_(right_queue_, t_left, right_candidate))
    {
        return;
    }

    left_queue_.clear();
    auto it = std::find(right_queue_.begin(), right_queue_.end(), right_candidate);
    if (it != right_queue_.end())
    {
        right_queue_.erase(it);
    }

    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(imu_wait_timeout_ms_));
    double gap_ms = std::numeric_limits<double>::infinity();

    while (rclcpp::ok())
    {
        ImuMsg::SharedPtr latest_before_left;
        for (auto rit = imu_queue_.rbegin(); rit != imu_queue_.rend(); ++rit)
        {
            if (rclcpp::Time((*rit)->header.stamp) <= t_left)
            {
                latest_before_left = *rit;
                break;
            }
        }

        gap_ms = std::numeric_limits<double>::infinity();
        if (latest_before_left)
        {
            gap_ms = duration_ms(t_left - rclcpp::Time(latest_before_left->header.stamp));
            if (gap_ms <= imu_recent_epsilon_ms_)
            {
                break;
            }
        }

        if (std::chrono::steady_clock::now() >= deadline)
        {
            RCLCPP_WARN(this->get_logger(),
                        "IMU tail gap=%.3f ms > eps=%.3f, waited %.1f ms => feeding anyway",
                        gap_ms,
                        imu_recent_epsilon_ms_,
                        imu_wait_timeout_ms_);
            break;
        }

        imu_cv_.wait_for(lock, std::chrono::milliseconds(1));
        if (!rclcpp::ok())
        {
            return;
        }
    }

    const bool have_prev_frame = last_frame_stamp_.nanoseconds() > 0;
    rclcpp::Time t_prev = have_prev_frame ? last_frame_stamp_ : rclcpp::Time(t_left.nanoseconds() - static_cast<int64_t>(2e7), RCL_ROS_TIME);
    if (t_prev > t_left)
    {
        t_prev = t_left;
    }

    gather_imu_locked_(t_left, t_prev, imu_msgs, tail_ok, imu_span_ms);

    right = std::make_shared<ImageMsg>(*right_candidate);
    right->header.stamp = left->header.stamp;

    const bool span_ok = (imu_span_ms >= min_imu_span_ms_) || !have_prev_frame;

    const auto left_copy = left;
    const auto right_copy = right;
    const auto imu_copy = imu_msgs;

    lock.unlock();

    std::vector<ORB_SLAM3::IMU::Point> imu_points;
    imu_points.reserve(imu_copy.size());
    for (const auto &imu : imu_copy)
    {
        const double t = Utility::StampToSec(imu->header.stamp);
        cv::Point3f acc(imu->linear_acceleration.x, imu->linear_acceleration.y, imu->linear_acceleration.z);
        cv::Point3f gyr(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);
        imu_points.emplace_back(acc, gyr, t);
    }

    if (!span_ok)
    {
        std::cout << "[WARN] IMU span " << std::fixed << std::setprecision(3) << imu_span_ms
                  << " ms < required " << min_imu_span_ms_ << " ms -- feeding anyway" << std::endl;
    }

    publish_pair_(left_copy, right_copy, imu_points, tail_ok, imu_span_ms);

    std::lock_guard<std::mutex> lock_update(data_mutex_);
    last_frame_stamp_ = t_left;
}

void StereoInertialNode::publish_pair_(const ImageMsg::SharedPtr &left,
                                       const ImageMsg::SharedPtr &right,
                                       const std::vector<ORB_SLAM3::IMU::Point> &imu_points,
                                       bool tail_within_margin,
                                       double imu_span_ms)
{
    if (!left || !right)
    {
        return;
    }

    cv::Mat imLeft = GetImage(left);
    cv::Mat imRight = GetImage(right);

    if (bClahe_)
    {
        clahe_->apply(imLeft, imLeft);
        clahe_->apply(imRight, imRight);
    }

    if (doRectify_)
    {
        cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
        cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
    }

    const double tLeft = Utility::StampToSec(left->header.stamp);

    std::cout << std::fixed << std::setprecision(6)
              << "[PAIR] t=" << tLeft
              << " imu_n=" << imu_points.size()
              << " span_ms=" << std::setprecision(3) << imu_span_ms
              << " tail<=eps=" << (tail_within_margin ? "YES" : "NO")
              << " (L ts == R ts: " << (left->header.stamp == right->header.stamp ? "YES" : "NO") << ")"
              << std::endl;

    SLAM_->TrackStereo(imLeft, imRight, tLeft, imu_points);
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

int64_t StereoInertialNode::to_ns(const builtin_interfaces::msg::Time &t)
{
    return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

builtin_interfaces::msg::Time StereoInertialNode::from_ns(int64_t ns)
{
    if (ns < 0)
    {
        ns = 0;
    }

    builtin_interfaces::msg::Time t;
    t.sec = static_cast<int32_t>(ns / 1000000000LL);
    t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    return t;
}
