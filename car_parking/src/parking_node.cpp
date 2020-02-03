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

void grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& _grid){
    ROS_INFO("I heard grid.");
    grid = _grid;
}

void poly_cb(const car_parking::Polygons::ConstPtr& _poly){
    polygons = _poly;
    ROS_INFO("I heard polygons.");
}

void cmd_cb(const std_msgs::UInt8 _cmd){
    cmd = static_cast<Cmd_t>(_cmd.data);
    if(cmd == Cmd_t::START){
        ROS_INFO("Cmd: Start.");
    }else if(cmd == Cmd_t::STOP){
        ROS_INFO("Cmd: Stop.");
    }else{
        ROS_WARN("Cmd: Unrecognized.");
    }
}

void process(){
    car_parking::Statuses statuses;
    parking.Process(grid, polygons, statuses);
    status_pub.publish(statuses);
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
