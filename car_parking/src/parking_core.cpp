#include "car_parking/parking_core.hpp"
#include <algorithm>
#include <cmath>

namespace wr8_parking {

CarParking::CarParking(): grid_(nullptr), polygons_(nullptr){
    std::cout << "Core: Hello!\n";
}


/*
 * @brief Check if there are an obtacles on polygones area using grid and write
 * result to the statuses
 * @param [in]  grid
 * @param [in]  poly
 * @param [out]  statuses
 * @return 0 - success, -1 - error occured
 */
int CarParking::Process(const nav_msgs::OccupancyGrid::ConstPtr& grid,
                        const car_parking::Polygons::ConstPtr& poly,
                        car_parking::Statuses& statuses){
    ROS_INFO("Core: Process() was called.");

    if(grid == nullptr || poly == nullptr){
        ROS_WARN("Core: Grid or poly are nullptr.");
        return -1;
    }

    if(grid != nullptr){
        grid_resolution = grid->info.resolution;

        grid_left = grid->info.origin.position.x;
        grid_right = grid->info.origin.position.x + grid->info.width * grid_resolution;
        grid_bot = grid->info.origin.position.y;
        grid_top = grid->info.origin.position.y + grid->info.height * grid_resolution;

        left_bot_x_ = grid_left;
        left_bot_y_ = grid_bot;
        left_top_x_ = grid_left;
        left_top_y_ = grid_top;
        right_bot_x_ = grid_right;
        right_bot_y_ = grid_bot;
        right_top_x_ = grid_right;
        right_top_y_ = grid_top;

        //ROS_INFO("Core: grid->info.resolution = %f", grid->info.resolution);
        //ROS_INFO("Core: grid->info.width = %u", grid->info.width);
        //ROS_INFO("Core: grid->info.height = %u", grid->info.height);
        //ROS_INFO("Core: grid->info.origin.position.x = %f", grid->info.origin.position.x);
        //ROS_INFO("Core: grid->info.origin.position.y = %f", grid->info.origin.position.y);

        ROS_INFO("Core: left_bot = %f/%f", left_bot_x_, left_bot_y_);
        ROS_INFO("Core: left_top = %f/%f", left_top_x_, left_top_y_);
        ROS_INFO("Core: right_bot = %f/%f", right_bot_x_, right_bot_y_);
        ROS_INFO("Core: right_top = %f/%f", right_top_x_, right_top_y_);
    }

    for(auto i = poly->polygons.begin(); i != poly->polygons.end(); i++){
        if(!IsPolygonConvex(*i)){
            statuses.statuses.push_back(Status_t::BAD_POLYGON);
            ROS_WARN("Core: There is non-convex polygon!");
        }else if(!IsConvexInsideGrid(*i)){
            statuses.statuses.push_back(Status_t::OUT_OF_RANGE);
            ROS_INFO("Core: Convex is out of grid.");
        }else if(!IsPolygonEmpty(*i)){
            statuses.statuses.push_back(Status_t::FILLED);
            ROS_INFO("Core: Convex is filled.");
        }else{
            statuses.statuses.push_back(Status_t::EMPTY);
            ROS_INFO("Core: Convex is empty.");
        }
    }
    return 0;
}

bool CarParking::IsPolygonConvex(const car_parking::Points2D& poly){
    return true;
}

// polygon must be inside grid!
bool CarParking::IsPolygonEmpty(const car_parking::Points2D& poly){
    return true;
}

size_t CarParking::WorldPoseToColIndex(float world_pose){
    return (world_pose - grid_left) / grid_resolution;
}
size_t CarParking::WorldPoseToRowIndex(float world_pose){
    return (grid_top - world_pose) / grid_resolution;
}

bool CarParking::WorldPoseToIndexes(const car_parking::Points2D& poly){
    return true;
}

// polygon must be convex
bool CarParking::IsConvexInsideGrid(const car_parking::Points2D& poly){
    bool is_polygon_in_grid = true;
    if(poly.points.size() == 4){
        std::vector<float> x, y;
        for(size_t i = 0; i < poly.points.size(); i++){
            x.push_back(poly.points[i].x);
            y.push_back(poly.points[i].y);
        }

        poly_left = *std::min_element(x.begin(), x.end());
        poly_right = *std::max_element(x.begin(), x.end());
        poly_bot = *std::min_element(y.begin(), y.end());
        poly_top = *std::max_element(y.begin(), y.end());
        
        std::cout << "left = " << poly_left << " " << grid_left << std::endl;
        std::cout << "right = " << poly_right << " " << grid_right << std::endl;
        std::cout << "bot = " << poly_bot << " " << grid_bot << std::endl;
        std::cout << "top = " << poly_top << " " << grid_top << std::endl;

        if(poly_bot < grid_bot || poly_left < grid_left || 
           poly_top > grid_top || poly_right > grid_right){
            is_polygon_in_grid = false;
        }
    }else{
        is_polygon_in_grid = false;
    }
    if(is_polygon_in_grid){
        ROS_INFO("Polygon in area.");
    }else{
        ROS_INFO("Polygon is out of area.");
    }

    return is_polygon_in_grid;
}

} //end namespace wr8_parking
