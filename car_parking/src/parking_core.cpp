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
 * @param [out]  statuses
 * @return 0 - success, -1 - error occured
 */
int CarParking::Process(car_parking::Statuses& statuses){
    ROS_INFO("Core: Process() was called.");

    if(grid_ == nullptr || polygons_ == nullptr){
        ROS_WARN("Core: Grid or poly are nullptr.");
        return -1;
    }

    for(auto i = polygons_->polygons.begin(); i != polygons_->polygons.end(); i++){
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

void CarParking::UpdateGrid(const nav_msgs::OccupancyGrid::ConstPtr& grid){
    grid_ = grid;

    grid_resolution_ = grid->info.resolution;
    grid_left_ = grid->info.origin.position.x;
    grid_right_ = grid->info.origin.position.x + grid->info.width * grid_resolution_;
    grid_bot_ = grid->info.origin.position.y;
    grid_top_ = grid->info.origin.position.y + grid->info.height * grid_resolution_;

    ROS_INFO("Core: grid_left = %f.", grid_left_);
    ROS_INFO("Core: grid_right = %f.", grid_right_);
    ROS_INFO("Core: grid_bot = %f.", grid_bot_);
    ROS_INFO("Core: grid_top = %f.", grid_top_);

    //ROS_INFO("Core: grid_resolution = %f", grid_resolution_);
    //ROS_INFO("Core: grid->info.width = %u", grid->info.width);
    //ROS_INFO("Core: grid->info.height = %u", grid->info.height);
}

void CarParking::UpdatePolygones(const car_parking::Polygons::ConstPtr& polygons){
    polygons_ = polygons;
}

bool CarParking::IsPolygonConvex(const car_parking::Points2D& poly){
    return true;
}

// polygon must be inside grid!
bool CarParking::IsPolygonEmpty(const car_parking::Points2D& poly){
    return true;
}

size_t CarParking::WorldPoseToColIndex(float world_pose){
    return (world_pose - grid_left_) / grid_resolution_;
}
size_t CarParking::WorldPoseToRowIndex(float world_pose){
    return (grid_top_ - world_pose) / grid_resolution_;
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

        poly_left_ = *std::min_element(x.begin(), x.end());
        poly_right_ = *std::max_element(x.begin(), x.end());
        poly_bot_ = *std::min_element(y.begin(), y.end());
        poly_top_ = *std::max_element(y.begin(), y.end());
        
        std::cout << "left = " << poly_left_ << " " << grid_left_ << std::endl;
        std::cout << "right = " << poly_right_ << " " << grid_right_ << std::endl;
        std::cout << "bot = " << poly_bot_ << " " << grid_bot_ << std::endl;
        std::cout << "top = " << poly_top_ << " " << grid_top_ << std::endl;

        if(poly_bot_ < grid_bot_ || poly_left_ < grid_left_ || 
           poly_top_ > grid_top_ || poly_right_ > grid_right_){
            is_polygon_in_grid = false;
        }
    }else{
        is_polygon_in_grid = false;
    }
    return is_polygon_in_grid;
}

} //end namespace wr8_parking
