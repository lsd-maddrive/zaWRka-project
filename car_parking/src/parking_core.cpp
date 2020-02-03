#include "car_parking/parking_core.hpp"

namespace wr8_parking {

CarParking::CarParking(){
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
int CarParking::Process(
            const nav_msgs::OccupancyGrid::ConstPtr& grid,
            const car_parking::Polygons::ConstPtr& poly,
            car_parking::Statuses& statuses) const{
    ROS_INFO("Core: Process() was called.");

    if(poly != nullptr){
        for(size_t i = 0; i < poly->polygons.size(); i++){
            statuses.statuses.push_back(0);
        }
    }
    
    float left_bot_x, left_bot_y,
          left_top_x, left_top_y,
          right_bot_x, right_bot_y,
          right_top_x, right_top_y;
    if(grid != nullptr){
        left_bot_x = grid->info.origin.position.x;
        left_bot_y = grid->info.origin.position.y;
        left_top_x = grid->info.origin.position.x;
        left_top_y = grid->info.origin.position.y + grid->info.height * grid->info.resolution;
        right_bot_x = grid->info.origin.position.x + grid->info.width * grid->info.resolution;
        right_bot_y = grid->info.origin.position.y;
        right_top_x = grid->info.origin.position.x + grid->info.width * grid->info.resolution;
        right_top_y = grid->info.origin.position.y + grid->info.height * grid->info.resolution;

        //ROS_INFO("Core: grid->info.resolution = %f", grid->info.resolution);
        //ROS_INFO("Core: grid->info.width = %u", grid->info.width);
        //ROS_INFO("Core: grid->info.height = %u", grid->info.height);
        //ROS_INFO("Core: grid->info.origin.position.x = %f", grid->info.origin.position.x);
        //ROS_INFO("Core: grid->info.origin.position.y = %f", grid->info.origin.position.y);

        ROS_INFO("Core: left_bot = %f/%f", left_bot_x, left_bot_y);
        ROS_INFO("Core: left_top = %f/%f", left_top_x, left_top_y);
        ROS_INFO("Core: right_bot = %f/%f", right_bot_x, right_bot_y);
        ROS_INFO("Core: right_top = %f/%f", right_top_x, right_top_y);
    }

    if(grid == nullptr || poly == nullptr){
        ROS_WARN("Core: Grid or poly are nullptr.");
        return -1;
    }
    


    return 0;
}

} //end namespace wr8_parking
