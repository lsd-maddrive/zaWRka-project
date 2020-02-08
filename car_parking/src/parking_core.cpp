#include "car_parking/parking_core.hpp"
#include <algorithm>
#include <cmath>

namespace wr8_parking {

CarParking::CarParking(): grid_(nullptr), grid_resolution_(0.05){
}


/*
 * @brief Check if there are an obtacles on polygones area using grid and write
 * result to the statuses
 * @param [out]  statuses
 * @return 0 - success, -1 - error occured
 */
int CarParking::Process(car_parking::Statuses& statuses){
    if(grid_ == nullptr || polygons_.empty()){
        ROS_WARN("Core: Grid is nullptre or poly is empty!");
        return -1;
    }
    Status_t status;
    statuses.statuses.clear();
    for(auto i = polygons_.begin(); i != polygons_.end(); i++){
        if(!i->IsPolygonCorrect()){
            status = Status_t::BAD_POLYGON;
            ROS_WARN("Core: There is non-convex polygon!");
        }else if(!IsConvexInsideGrid(*i)){
            status = Status_t::OUT_OF_RANGE;
        }else if(!IsPolygonEmpty(*i)){
            status = Status_t::FILLED;
        }else{
            status = Status_t::EMPTY;
        }
        statuses.statuses.push_back(status);
    }
    return 0;
}

void CarParking::UpdateGrid(const nav_msgs::OccupancyGrid::ConstPtr& grid){
    if(grid == nullptr){
        ROS_WARN("Core: grid can't be nullptr!");
        return;
    }
    grid_ = grid;
    grid_resolution_ = grid->info.resolution;
    grid_left_ = grid->info.origin.position.x;
    grid_right_ = grid->info.origin.position.x + grid->info.width * grid_resolution_;
    grid_bot_ = grid->info.origin.position.y;
    grid_top_ = grid->info.origin.position.y + grid->info.height * grid_resolution_;
}

void CarParking::UpdatePolygons(const car_parking::Polygons::ConstPtr& polygons){
    polygons_.clear();
    if(polygons == nullptr){
        ROS_WARN("Core: polygons can't be nullptr!");
        return;
    }
    for(auto polygon = polygons->polygons.begin(); polygon != polygons->polygons.end(); polygon++){
        polygons_.push_back(PolygonInfo());
        if(!IsPolygonConvex(*polygon)){
            ROS_WARN("Core: There is non-convex polygon!");
            continue;
        }
        // 1. find max and min values of points:
        CalculateEdgeIndexes(*polygon, polygons_.back().min_x, polygons_.back().max_x,
                                       polygons_.back().min_y, polygons_.back().max_y);

        // 2. initialize empty borders vector
        size_t min_row = 0;
        size_t max_row = (polygons_.back().max_y - polygons_.back().min_y) / grid_resolution_;
        for(size_t i = min_row; i <= max_row; i++){
            polygons_.back().borders.push_back(Border_t(SIZE_MAX, 0));
        }

        // 3. add ab, bc, cd, da
        size_t first[] = {0, 1, 2, 3};
        size_t second[] = {1, 2, 3, 0};
        ssize_t a_col, a_row, b_col, b_row, row_amount, d_row;
        float d_col;
        for(size_t i = 0; i < 4; i++){
            a_col = (polygon->points[first[i]].x - polygons_.back().min_x) / grid_resolution_;
            a_row = (polygon->points[first[i]].y - polygons_.back().min_y) / grid_resolution_;
            b_col = (polygon->points[second[i]].x - polygons_.back().min_x) / grid_resolution_;
            b_row = (polygon->points[second[i]].y - polygons_.back().min_y) / grid_resolution_;

            row_amount = abs(b_row - a_row);
            d_row = (b_row >= a_row) ? 1 : -1;
            d_col = (row_amount) ? static_cast<float>(b_col - a_col) / row_amount : (b_col - a_col);

            float cur_col = a_col;
            for(size_t cur_row = a_row; cur_row != (b_row + d_row); cur_row += d_row){
                if(cur_col < polygons_.back().borders[cur_row].first){
                    polygons_.back().borders[cur_row].first = cur_col;
                }
                if(cur_col > polygons_.back().borders[cur_row].second){
                    polygons_.back().borders[cur_row].second = cur_col;
                }
                cur_col += d_col;
            }
        }
    }
}

void CarParking::CalculateEdgeIndexes(const car_parking::Points2D& polygon,
                                      float& pose_x_min, float& pose_x_max,
                                      float& pose_y_min, float& pose_y_max){
    pose_x_min = FLT_MAX;
    pose_x_max = -FLT_MAX;
    pose_y_min = FLT_MAX;
    pose_y_max = -FLT_MAX;
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
bool CarParking::IsPolygonEmpty(const PolygonInfo& polygon){
    ssize_t offset_col = (polygon.min_x - grid_left_) / grid_resolution_;
    ssize_t offset_row = (polygon.min_y - grid_bot_) / grid_resolution_;
    int8_t max_value = 0;
    int32_t average_value = 0;
    size_t counter = 0;

    for(size_t row = offset_row; row < offset_row + polygon.borders.size() - 1; row++){
        for(size_t col = polygon.borders[row - offset_row].first  + offset_col;
                   col < polygon.borders[row - offset_row].second + offset_col;
                   col++){
            int8_t cell_value = grid_->data[col + row * grid_->info.width];
            if(cell_value > max_value){
                max_value = cell_value;
            }
            average_value += cell_value;
            counter++;
            //std::cout << cell_value + 0 << " ";
        }
        //std::cout << std::endl;
    }
    if(counter != 0){
        average_value /= counter;
    }
    ROS_INFO("max = %d, avg = %d", max_value + 0, average_value);
    if(max_value <= 80){
        return true;
    }
    return false;
}

// polygon must be convex
bool CarParking::IsConvexInsideGrid(const PolygonInfo& poly_info){
    bool is_polygon_inside_grid;
    if(poly_info.min_y < grid_bot_ || poly_info.min_x < grid_left_ || 
       poly_info.max_y > grid_top_ || poly_info.max_x > grid_right_){
        is_polygon_inside_grid = false;
    }
    else{
        is_polygon_inside_grid = true;
    }
    return is_polygon_inside_grid;
}

// grid must be != nullptr
bool CarParking::PrintGrid(){
    for(size_t r = 0; r < grid_->info.height; r++){
        std::cout << r << ": ";
        for(size_t c = 0; c < grid_->info.width; c++){
            std::cout << grid_->data[c + r * grid_->info.width] + 0 << " ";
        }
        std::cout << std::endl;
    }
}

} //end namespace wr8_parking