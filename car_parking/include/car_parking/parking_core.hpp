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
        int Process(const nav_msgs::OccupancyGrid::ConstPtr& grid,
                    const car_parking::Polygons::ConstPtr& polygons,
                    car_parking::Statuses& statuses);
    private:
        enum Status_t: uint8_t{
            NO_INFO = 0,
            EMPTY = 1,
            FILLED = 2,
            OUT_OF_RANGE = 3,
            BAD_POLYGON = 4,
        };

        bool IsPolygonConvex(const car_parking::Points2D& poly);
        bool IsConvexInsideGrid(const car_parking::Points2D& poly);
        bool IsPolygonEmpty(const car_parking::Points2D& poly);

        bool WorldPoseToIndexes(const car_parking::Points2D& poly);
        size_t WorldPoseToColIndex(float world_pose);
        size_t WorldPoseToRowIndex(float world_pose);

        const nav_msgs::OccupancyGrid::ConstPtr& grid_;
        const car_parking::Polygons::ConstPtr& polygons_;

        float grid_resolution;
        float grid_left, grid_right, grid_bot, grid_top;
        float poly_left, poly_right, poly_bot, poly_top;

        float left_bot_x_, left_bot_y_,
              left_top_x_, left_top_y_,
              right_bot_x_, right_bot_y_,
              right_top_x_, right_top_y_;
};

} //end namespace wr8_parking
#endif // PARKING_CORE_HPP_

