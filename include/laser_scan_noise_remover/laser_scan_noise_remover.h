#ifndef __LASER_SCAN_NOISE_REMOVER_H__
#define __LASER_SCAN_NOISE_REMOVER_H__

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace laser_scan_noise_remover {
struct Config {
    double hz;
    double cutoff_distance;
};

class LaserScanNoiseRemover {
public:
    LaserScanNoiseRemover(void);
    void remove_noise_by_distance(sensor_msgs::LaserScan& scan);
    void process(void);

protected:
    void scan_callback_(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    Config config_;
    sensor_msgs::LaserScan scan_;
};
} // namespace laser_scan_noise_remover

#endif // __LASER_SCAN_NOISE_REMOVER_H__
