#include "car_parking/parking_core.hpp"
#include <algorithm>
#include <cmath>

namespace wr8_parking {

CarParking::CarParking(): grid_(nullptr), polygons_(nullptr), grid_resolution_(0.05){
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

    if(grid_ == nullptr){
        ROS_WARN("Core: Grid is nullptr!");
        return -1;
    }
   else if(polygons_ == nullptr){
        ROS_WARN("Core: Poly is nullptr!");
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
    if(polygons == nullptr){
        ROS_WARN("Core: polygons can't be nullptr!");
        return;
    }
    polygons_ = polygons;
    for(auto polygon = polygons_->polygons.begin(); polygon != polygons->polygons.end(); polygon++){
        if(!IsPolygonConvex(*polygon)){
            ROS_WARN("Core: There is non-convex polygon!");
            continue;
        }

        // 1. find max and min values of points:
        float pose_x_min, pose_x_max, pose_y_min, pose_y_max;
        CalculateEdgeIndexes(*polygon, pose_x_min, pose_x_max, pose_y_min, pose_y_max);

        // 2. initialize empty borders vector
        size_t min_row = 0;
        size_t max_row = (pose_y_max - pose_y_min) / grid_resolution_;
        std::vector<Border_t> poly_borders;
        for(size_t i = min_row; i <= max_row; i++){
            poly_borders.push_back(Border_t(SIZE_MAX, 0));
        }

        // 3. add ab, bc, cd, da
        std::vector<std::pair<size_t, size_t>> pairs;
        pairs.push_back(std::pair<size_t, size_t>(0, 1));
        pairs.push_back(std::pair<size_t, size_t>(1, 2));
        pairs.push_back(std::pair<size_t, size_t>(2, 3));
        pairs.push_back(std::pair<size_t, size_t>(3, 0));

        for(auto pair = pairs.begin(); pair != pairs.end(); pair++){
            ssize_t a_col = (polygon->points[pair->first].x - pose_x_min) / grid_resolution_;
            ssize_t a_row = (polygon->points[pair->first].y - pose_y_min) / grid_resolution_;
            ssize_t b_col = (polygon->points[pair->second].x - pose_x_min) / grid_resolution_;
            ssize_t b_row = (polygon->points[pair->second].y - pose_y_min) / grid_resolution_;

            ssize_t row_amount = abs(b_row - a_row);
            ssize_t d_row = (b_row >= a_row) ? 1 : -1;
            ssize_t d_col = (row_amount) ? (b_col - a_col) / row_amount : (b_col - a_col);

            //std::cout << "row range(" << a_row << ", " << b_row << ", by " << d_row << ")" << std::endl;
            //std::cout << "col range(" << a_col << ", " << b_col << ", by " << d_col << ")" << std::endl;

            size_t cur_col = a_col;
            for(size_t cur_row = a_row; cur_row != (b_row + d_row); cur_row += d_row){
                //std::cout << "cur [" << cur_row << "] is [" << poly_borders[cur_row].first << 
                //             "/" << poly_borders[cur_row].second << "]" << std::endl;
                if(cur_col < poly_borders[cur_row].first){
                    poly_borders[cur_row].first = cur_col;
                    //std::cout << "set [" << cur_row << "].left to " << cur_col << std::endl;
                }
                if(cur_col > poly_borders[cur_row].second){
                    poly_borders[cur_row].second = cur_col;
                    //std::cout << "set [" << cur_row << "/" << cur_col << "].right to " << cur_col << std::endl;
                }
                cur_col += d_col;
            }
        }

        // 4. show data
        ROS_INFO("Polygon border vector size is %lu", poly_borders.size());
        ROS_INFO("pose_x_min = %f", pose_x_min);
        ROS_INFO("pose_x_max = %f", pose_x_max);
        ROS_INFO("pose_y_min = %f", pose_y_min);
        ROS_INFO("pose_y_max = %f", pose_y_max);
        for(auto i = poly_borders.begin(); i != poly_borders.end(); i++){
            std::cout << i->first << " " << i->second << std::endl;
        }
        ROS_INFO(" ");
    }
}

void CarParking::CalculateEdgeIndexes(const car_parking::Points2D& polygon,
                                      float& pose_x_min, float& pose_x_max,
                                      float& pose_y_min, float& pose_y_max){
        pose_x_min = FLT_MAX;
        pose_x_max = FLT_MIN;
        pose_y_min = FLT_MAX;
        pose_y_max = FLT_MIN;
        for(auto point = polygon.points.begin(); point != polygon.points.end(); point++){
            if(point->x < pose_x_min){
                pose_x_min = point->x;
            }
            if(point->x > pose_x_max){
                pose_x_max = point->x;
            }
            if(point->y < pose_y_min){
                pose_y_min = point->y;
            }
            if(point->y > pose_y_max){
                pose_y_max = point->y;
            }
        }
}

bool CarParking::IsPolygonConvex(const car_parking::Points2D& poly){
    return true;
}

// polygon must be inside grid!
bool CarParking::IsPolygonEmpty(const car_parking::Points2D& polygon){
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
