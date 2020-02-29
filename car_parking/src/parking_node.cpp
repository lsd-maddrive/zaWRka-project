#include <std_msgs/UInt8.h>

#include "car_parking/parking_core.hpp"

enum Cmd_t{
    STOP = 0,
    START = 1,
};

static ros::NodeHandle* nh;
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

static const char* NODE_NAME = "parking_node";
static const char* POLY_SUB_TOPIC = "parking_polygones";
static const char* STATUS_PUB_TOPIC = "parking_status";
static const char* CMD_SUB_TOPIC = "parking_cmd";
static const char* GRID_SUB_TOPIC = "costmap";

void grid_cb(const nav_msgs::OccupancyGrid::ConstPtr& _grid){
    grid = _grid;
    is_grid_updated = true;
}

void poly_cb(const car_parking::Polygons::ConstPtr& _poly){
    polygons = _poly;
    is_polygons_updated = true;
}

void cmd_cb(const std_msgs::UInt8 _cmd){
    cmd = static_cast<Cmd_t>(_cmd.data);
}

void process(){
    if(grid != nullptr && polygons != nullptr){
        car_parking::Statuses statuses;
        if(is_grid_updated){
            ROS_DEBUG("UpdateGrid() is calling.");
            parking.UpdateGrid(grid);
        }
        if(is_polygons_updated){
            ROS_DEBUG("UpdatePolygons() is calling.");
            parking.UpdatePolygons(polygons);
        }
        if(is_grid_updated || is_polygons_updated){
            is_grid_updated = false;
            is_polygons_updated = false;
            ROS_DEBUG("Process() is calling.");
            parking.Process(statuses);
        }
        status_pub.publish(statuses);
    }else{
        ROS_WARN_THROTTLE(5, "Can't call the process(). Start is received, but there is no grid or polygons.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    nh = new ros::NodeHandle;
    status_pub = nh->advertise<car_parking::Statuses>(STATUS_PUB_TOPIC, 10);
    poly_sub = nh->subscribe(POLY_SUB_TOPIC, 2, poly_cb);
    cmd_sub = nh->subscribe(CMD_SUB_TOPIC, 2, cmd_cb);
    grid_sub = nh->subscribe(GRID_SUB_TOPIC, 2, grid_cb);

    ros::Rate loop_rate(1);
    while ( ros::ok() )
    {
        if(cmd == Cmd_t::START){
            process();
        }else{
            ROS_DEBUG_THROTTLE(5, "Waiting for START cmd...");
        }
        ros::spinOnce(); // handle callbacks
        loop_rate.sleep();
    }

    delete nh;
    return 0;
}
