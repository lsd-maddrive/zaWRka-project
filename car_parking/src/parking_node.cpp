#include <sstream>
#include <vector>

#include <geometry_msgs/Polygon.h>
#include <std_msgs/UInt8.h>

#include "car_parking/parking_core.hpp"

//namespace wr8_parking {

enum Cmd_t{
    STOP = 0,
    START = 1,
};


static ros::Publisher status_pub;
static ros::Subscriber cmd_sub;
static ros::Subscriber poly_sub;
static ros::Subscriber grid_sub;
static Cmd_t cmd = Cmd_t::STOP;
static wr8_parking::CarParking parking;
static nav_msgs::OccupancyGrid::ConstPtr grid = nullptr;
static car_parking::Polygons::ConstPtr polygons = nullptr;
static bool is_grid_updated = false;
static bool is_polygons_updated = false;


void grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& _grid){
    ROS_INFO("I heard grid.");
    grid = _grid;
    is_grid_updated = true;
}

void poly_cb(const car_parking::Polygons::ConstPtr& _poly){
    ROS_INFO("I heard polygons.");
    polygons = _poly;
    is_polygons_updated = true;
}

void cmd_cb(const std_msgs::UInt8 _cmd){
    ROS_INFO("I heard cmd.");
    cmd = static_cast<Cmd_t>(_cmd.data);
}

void process(){
    if(grid != nullptr && polygons != nullptr){
        ROS_INFO("Parking node: Process() was called.");
        car_parking::Statuses statuses;
        if(is_grid_updated){
            parking.UpdateGrid(grid);
        }
        if(is_polygons_updated){
            parking.UpdatePolygones(polygons);
        }
        if(is_grid_updated || is_polygons_updated){
            is_grid_updated = false;
            is_polygons_updated = false;
            parking.Process(statuses);
        }
        // only for test:
        else{
            parking.Process(statuses);
        }
        status_pub.publish(statuses);
    }else{
        ROS_WARN("Parking node: Can't call the process(). Start command is received, but there is no grid or polygons.");
    }
}

int main(int argc, char **argv)
{
	const char* NODE_NAME = "parking_node";
	const char* POLY_SUB_TOPIC = "/parking_areas_polygones";
	const char* STATUS_PUB_TOPIC = "/parking_areas_status";
	const char* CMD_SUB_TOPIC = "/parking_cmd";
    const char* GRID_SUB_TOPIC = "/move_base/local_costmap/costmap";

    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    status_pub = n.advertise<car_parking::Statuses>(STATUS_PUB_TOPIC, 10);
    poly_sub = n.subscribe(POLY_SUB_TOPIC, 2, poly_cb);
    cmd_sub = n.subscribe(CMD_SUB_TOPIC, 2, cmd_cb);
    grid_sub = n.subscribe(GRID_SUB_TOPIC, 2, grid_cb);

    ros::Rate loop_rate(1);
    while ( ros::ok() )
    {
        if(cmd == Cmd_t::START){
            ROS_INFO("cmd == START");
            process();
        }else{
            ROS_INFO("cmd != START");
        }
        ros::spinOnce(); // handle callbacks
        loop_rate.sleep();
    }

    return 0;
}

 //end namespace wr8_parking
