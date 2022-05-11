#include "laser_scan_noise_remover/laser_scan_noise_remover.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_scan_noise_remover");
    laser_scan_noise_remover::LaserScanNoiseRemover laser_scan_noise_remover;
    laser_scan_noise_remover.process();

    return 0;
}
