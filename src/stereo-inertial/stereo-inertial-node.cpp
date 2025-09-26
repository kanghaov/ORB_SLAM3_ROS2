#include "stereo-inertial-node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <vector>
#include <opencv2/core/core.hpp>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
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
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
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
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    double max_time_diff_ms = this->declare_parameter<double>("max_time_diff_ms", 10.0);
    double gc_window_ms = this->declare_parameter<double>("gc_window_ms", 500.0);
    double image_time_shift_ms = this->declare_parameter<double>("image_time_shift_ms", 0.0);
    force_right_stamp_to_left_ = this->declare_parameter<bool>("force_right_stamp_to_left", false);

    max_diff_ns_ = static_cast<int64_t>(std::llround(max_time_diff_ms * 1e6));
    gc_window_ns_ = static_cast<int64_t>(std::llround(std::max(0.0, gc_window_ms) * 1e6));
    image_time_shift_ns_ = static_cast<int64_t>(std::llround(image_time_shift_ms * 1e6));

    subImu_ = this->create_subscription<ImuMsg>("imu", 1000, std::bind(&StereoInertialNode::GrabImu, this, _1));
    subImgLeft_ = this->create_subscription<ImageMsg>("camera/left", 100, std::bind(&StereoInertialNode::GrabImageLeft, this, _1));
    subImgRight_ = this->create_subscription<ImageMsg>("camera/right", 100, std::bind(&StereoInertialNode::GrabImageRight, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "pairing: max_time_diff_ms=%.3f, gc_window_ms=%.1f, image_time_shift_ms=%.3f, force_right_stamp_to_left=%s",
                max_time_diff_ms,
                gc_window_ms,
                image_time_shift_ms,
                force_right_stamp_to_left_ ? "true" : "false");
}

StereoInertialNode::~StereoInertialNode()
{
    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(bufMutex_);
    imuBuf_.push(msg);
    last_imu_ns_ = to_ns(msg->header.stamp);
}

void StereoInertialNode::GrabImageLeft(const ImageMsg::SharedPtr msgLeft)
{
    const int64_t t_ns = to_ns(msgLeft->header.stamp);
    {
        std::lock_guard<std::mutex> lock(bufMutexLeft_);
        left_q_[t_ns] = msgLeft;
    }

    try_match_from_left_(t_ns);
    gc_old_();
}

void StereoInertialNode::GrabImageRight(const ImageMsg::SharedPtr msgRight)
{
    const int64_t t_ns = to_ns(msgRight->header.stamp);
    {
        std::lock_guard<std::mutex> lock(bufMutexRight_);
        right_q_[t_ns] = msgRight;
    }

    try_match_from_right_(t_ns);
    gc_old_();
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
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

void StereoInertialNode::try_match_from_left_(int64_t tL)
{
    ImageMsg::ConstSharedPtr left;
    ImageMsg::ConstSharedPtr right;
    int64_t best_dt = std::numeric_limits<int64_t>::max();
    bool matched = false;

    {
        std::unique_lock<std::mutex> left_lock(bufMutexLeft_, std::defer_lock);
        std::unique_lock<std::mutex> right_lock(bufMutexRight_, std::defer_lock);
        std::lock(left_lock, right_lock);
        auto it_left = left_q_.find(tL);
        if (it_left == left_q_.end() || right_q_.empty())
        {
            return;
        }

        auto candidate = right_q_.lower_bound(tL);
        auto best_it = right_q_.end();

        if (candidate != right_q_.end())
        {
            int64_t dt = std::llabs(candidate->first - tL);
            best_dt = dt;
            best_it = candidate;
        }
        if (candidate != right_q_.begin())
        {
            auto prev_it = std::prev(candidate);
            int64_t dt = std::llabs(prev_it->first - tL);
            if (dt < best_dt)
            {
                best_dt = dt;
                best_it = prev_it;
            }
        }

        if (best_it != right_q_.end() && best_dt <= max_diff_ns_)
        {
            left = it_left->second;
            right = best_it->second;
            left_q_.erase(it_left);
            right_q_.erase(best_it);
            matched = true;
        }
        else if (best_it != right_q_.end())
        {
            double dt_ms = static_cast<double>(best_dt) / 1e6;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "big time difference (L→R): %.3f ms", dt_ms);
        }
    }

    if (matched)
    {
        publish_pair_(left, right, best_dt);
    }
}

void StereoInertialNode::try_match_from_right_(int64_t tR)
{
    ImageMsg::ConstSharedPtr right;
    ImageMsg::ConstSharedPtr left;
    int64_t best_dt = std::numeric_limits<int64_t>::max();
    bool matched = false;

    {
        std::unique_lock<std::mutex> left_lock(bufMutexLeft_, std::defer_lock);
        std::unique_lock<std::mutex> right_lock(bufMutexRight_, std::defer_lock);
        std::lock(left_lock, right_lock);
        auto it_right = right_q_.find(tR);
        if (it_right == right_q_.end() || left_q_.empty())
        {
            return;
        }

        auto candidate = left_q_.lower_bound(tR);
        auto best_it = left_q_.end();

        if (candidate != left_q_.end())
        {
            int64_t dt = std::llabs(candidate->first - tR);
            best_dt = dt;
            best_it = candidate;
        }
        if (candidate != left_q_.begin())
        {
            auto prev_it = std::prev(candidate);
            int64_t dt = std::llabs(prev_it->first - tR);
            if (dt < best_dt)
            {
                best_dt = dt;
                best_it = prev_it;
            }
        }

        if (best_it != left_q_.end() && best_dt <= max_diff_ns_)
        {
            right = it_right->second;
            left = best_it->second;
            right_q_.erase(it_right);
            left_q_.erase(best_it);
            matched = true;
        }
        else if (best_it != left_q_.end())
        {
            double dt_ms = static_cast<double>(best_dt) / 1e6;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "big time difference (R→L): %.3f ms", dt_ms);
        }
    }

    if (matched)
    {
        publish_pair_(left, right, best_dt);
    }
}

void StereoInertialNode::gc_old_()
{
    std::unique_lock<std::mutex> left_lock(bufMutexLeft_, std::defer_lock);
    std::unique_lock<std::mutex> right_lock(bufMutexRight_, std::defer_lock);
    std::lock(left_lock, right_lock);

    int64_t now_ns = 0;
    if (!left_q_.empty())
    {
        now_ns = std::max(now_ns, left_q_.rbegin()->first);
    }
    if (!right_q_.empty())
    {
        now_ns = std::max(now_ns, right_q_.rbegin()->first);
    }

    if (now_ns == 0 || gc_window_ns_ <= 0)
    {
        return;
    }

    const int64_t cutoff = now_ns - gc_window_ns_;
    std::size_t dropped_left = 0;
    std::size_t dropped_right = 0;

    while (!left_q_.empty() && left_q_.begin()->first < cutoff)
    {
        left_q_.erase(left_q_.begin());
        ++drop_left_gc_;
        ++dropped_left;
    }

    while (!right_q_.empty() && right_q_.begin()->first < cutoff)
    {
        right_q_.erase(right_q_.begin());
        ++drop_right_gc_;
        ++dropped_right;
    }

    if (dropped_left || dropped_right)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "GC dropped frames: left=%zu right=%zu (window %.1f ms)",
                             dropped_left,
                             dropped_right,
                             static_cast<double>(gc_window_ns_) / 1e6);
    }
}

void StereoInertialNode::publish_pair_(ImageMsg::ConstSharedPtr left_in,
                                       ImageMsg::ConstSharedPtr right_in,
                                       int64_t dt_ns)
{
    if (!left_in || !right_in)
    {
        return;
    }

    auto left = std::make_shared<ImageMsg>(*left_in);
    auto right = std::make_shared<ImageMsg>(*right_in);

    if (force_right_stamp_to_left_)
    {
        right->header.stamp = left->header.stamp;
    }

    if (image_time_shift_ns_ != 0)
    {
        const int64_t shift = image_time_shift_ns_;
        int64_t left_ns = to_ns(left->header.stamp) - shift;
        int64_t right_ns = to_ns(right->header.stamp) - shift;
        left->header.stamp = from_ns(std::max<int64_t>(0, left_ns));
        right->header.stamp = from_ns(std::max<int64_t>(0, right_ns));
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

    const int64_t left_ns = to_ns(left->header.stamp);
    const double tLeft = Utility::StampToSec(left->header.stamp);

    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    {
        std::lock_guard<std::mutex> lock(bufMutex_);
        while (!imuBuf_.empty() && to_ns(imuBuf_.front()->header.stamp) <= left_ns)
        {
            const auto &imu_msg = imuBuf_.front();
            double t = Utility::StampToSec(imu_msg->header.stamp);
            cv::Point3f acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
            cv::Point3f gyr(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
            vImuMeas.emplace_back(acc, gyr, t);
            imuBuf_.pop();
        }
    }

    SLAM_->TrackStereo(imLeft, imRight, tLeft, vImuMeas);

    kept_++;
    last_dt_ms_ = static_cast<double>(dt_ns) / 1e6;
    double imu_lag_ms = 0.0;
    if (last_imu_ns_ > 0)
    {
        imu_lag_ms = static_cast<double>(last_imu_ns_ - left_ns) / 1e6;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "PAIR summary: kept=%lu dropL(GC)=%lu dropR(GC)=%lu last_dt_ms=%.3f imu_lag_ms=%.3f",
                         kept_,
                         drop_left_gc_,
                         drop_right_gc_,
                         last_dt_ms_,
                         imu_lag_ms);
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
