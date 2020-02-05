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
        int Process(car_parking::Statuses& statuses);
        void UpdateGrid(const nav_msgs::OccupancyGrid::ConstPtr& grid);
        void UpdatePolygones(const car_parking::Polygons::ConstPtr& polygons);
    private:
        enum Status_t: uint8_t{
            NO_INFO = 0,
            EMPTY = 1,
            FILLED = 2,
            OUT_OF_RANGE = 3,
            BAD_POLYGON = 4,
        };

        typedef std::pair<size_t, size_t> Border_t;
        struct PolygonInfo{
            std::vector<Border_t> borders;
            float x_min;
            float y_min;
        };

        bool IsPolygonConvex(const car_parking::Points2D& poly);
        bool IsConvexInsideGrid(const car_parking::Points2D& poly);
        bool IsPolygonEmpty(const car_parking::Points2D& poly);

        void CalculateEdgeIndexes(const car_parking::Points2D& poly,
                                  float& pose_x_min, float& pose_x_max,
                                  float& pose_y_min, float& pose_y_max);

        bool WorldPoseToIndexes(const car_parking::Points2D& poly);
        size_t WorldPoseToColIndex(float world_pose);
        size_t WorldPoseToRowIndex(float world_pose);

        nav_msgs::OccupancyGrid::ConstPtr grid_;
        car_parking::Polygons::ConstPtr polygons_;
        std::vector<PolygonInfo> polygons_info_;

        float grid_resolution_;
        float grid_left_, grid_right_, grid_bot_, grid_top_;
        float poly_left_, poly_right_, poly_bot_, poly_top_;
};

} //end namespace wr8_parking
#endif // PARKING_CORE_HPP_

