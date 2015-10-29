#ifndef _DENSITY_FILTER_H_
#define _DENSITY_FILTER_H_

#include <filters/filter_base.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>

namespace density_filter
{
class DensityFilter: public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

    ~DensityFilter();

    bool update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out);

    bool configure();
private:
    double angle_increment_;
};
}

#endif
