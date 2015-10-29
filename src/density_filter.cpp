#include <density_filter.h>

PLUGINLIB_DECLARE_CLASS(density_filter, DensityFilter, density_filter::DensityFilter, filters::FilterBase<sensor_msgs::LaserScan>)

density_filter::DensityFilter::~DensityFilter() {}

bool density_filter::DensityFilter::configure()
{

    angle_increment_ = -1.0;
    getParam("angle_increment", angle_increment_);

    ROS_INFO("read angle increment desired %f", angle_increment_);

    return true;
}


double range_at(double angle, const sensor_msgs::LaserScan& msg)
{
    int index = std::floor((angle - msg.angle_min)/msg.angle_increment);

    if (index < 0 or index >= msg.ranges.size() - 1)
        return 1.1*msg.range_max;

    if (msg.ranges[index] < msg.range_min or msg.ranges[index + 1] < msg.range_min or msg.ranges[index] > msg.range_max or msg.ranges[index + 1] > msg.range_max)
        return 1.1*msg.range_max;

    double m = (msg.ranges[index + 1] - msg.ranges[index])/msg.angle_increment;
    double n = msg.ranges[index]  - m * (msg.angle_min + index*msg.angle_increment);
    return m * angle + n;
}


bool density_filter::DensityFilter::update(const sensor_msgs::LaserScan& data_in, sensor_msgs::LaserScan& data_out)
{
    if (angle_increment_ > data_in.angle_increment)
    {
        data_out = data_in;
    }
    else
    {
        data_out.header = data_in.header;
        data_out.angle_increment = angle_increment_;
        data_out.angle_max = data_in.angle_max;
        data_out.angle_min = data_in.angle_min;
        data_out.range_max = data_in.range_max;
        data_out.range_min = data_in.range_min;
        data_out.scan_time = data_in.scan_time;
        data_out.time_increment = data_in.time_increment;

        std::size_t n = std::floor((data_out.angle_max - data_out.angle_min)/angle_increment_) + 1;
        data_out.intensities.resize(n);
        data_out.ranges.resize(n);

        data_out.ranges.front() = data_in.ranges.front();
        for (std::size_t i = 1; i < n-1; ++i)
        {
            double angle = data_out.angle_min + i * angle_increment_;
            data_out.ranges[i] = range_at(angle, data_in);
        }

        data_out.ranges.back() = data_in.ranges.back();
    }

    return true;
}


