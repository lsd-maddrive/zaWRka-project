#ifndef PARKING_CORE_HPP_
#define PARKING_CORE_HPP_

#include <ros/ros.h>
#include <car_parking/Point2D.h>
#include <car_parking/Points2D.h>
#include <car_parking/Polygons.h>
#include <car_parking/Statuses.h>

#include <nav_msgs/OccupancyGrid.h>

namespace wr8_parking {

class CarParking
{
    public:
        CarParking();
        int Process(const nav_msgs::OccupancyGrid::ConstPtr& _grid,
                    const car_parking::Polygons::ConstPtr& msg,
                    car_parking::Statuses& statuses) const;
};

} //end namespace wr8_parking
#endif // PARKING_CORE_HPP_

