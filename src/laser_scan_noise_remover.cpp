#include "laser_scan_noise_remover/laser_scan_noise_remover.h"

namespace laser_scan_noise_remover {
LaserScanNoiseRemover::LaserScanNoiseRemover(void)
    : nh_private_("~")
{
    ROS_INFO("Initializing... [laser_scan_noise_remover]");

    sub_ = nh_.subscribe(
        "scan", 1, &LaserScanNoiseRemover::scan_callback_, this, ros::TransportHints().reliable().tcpNoDelay());
    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan/filtered", 1);

    nh_private_.param<double>("hz", config_.hz, 30);
    nh_private_.param<double>("cutoff_distance", config_.cutoff_distance, 0.5);
}

void LaserScanNoiseRemover::scan_callback_(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_ = *msg;
}

void LaserScanNoiseRemover::remove_noise_by_distance(sensor_msgs::LaserScan& scan)
{
    float pre_range = scan.ranges.at(0);

    for (auto& range : scan.ranges) {
        if (range < config_.cutoff_distance) {
            range = pre_range;
        }
        pre_range = range;
    }
}

void LaserScanNoiseRemover::process(void)
{
    ros::Rate loop_rate(config_.hz);

    while (ros::ok()) {
        sensor_msgs::LaserScan current_scan = scan_;
        remove_noise_by_distance(current_scan);
        pub_.publish(current_scan);
    }
}
} // namespace laser_scan_noise_remover
