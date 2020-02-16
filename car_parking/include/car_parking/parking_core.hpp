#ifndef PARKING_CORE_HPP_
#define PARKING_CORE_HPP_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <car_parking/Polygons.h>
#include <car_parking/Statuses.h>
#include <car_parking/Points2D.h>

namespace wr8_parking {

class CarParking
{
    public:
        CarParking();
        int Process(car_parking::Statuses& statuses);
        void UpdateGrid(const nav_msgs::OccupancyGrid::ConstPtr& grid);
        void UpdatePolygons(const car_parking::Polygons::ConstPtr& polygons);
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
            bool IsPolygonCorrect() const {return !borders.empty();};
            std::vector<Border_t> borders;
            float min_x;
            float max_x;
            float min_y;
            float max_y;
        };

        bool IsPolygonConvex(const car_parking::Points2D& poly) const;
        bool IsConvexInsideGrid(const PolygonInfo& poly) const;
        bool IsPolygonEmpty(const PolygonInfo& poly) const;
        void CalculateEdgeIndexes(const car_parking::Points2D& poly,
                                  float& pose_x_min, float& pose_x_max,
                                  float& pose_y_min, float& pose_y_max) const;

        bool PrintGrid() const;

        nav_msgs::OccupancyGrid::ConstPtr grid_;
        std::vector<PolygonInfo> polygons_;
        float grid_resolution_;
        float grid_left_, grid_right_, grid_bot_, grid_top_;
};

} //end namespace wr8_parking
#endif // PARKING_CORE_HPP_

