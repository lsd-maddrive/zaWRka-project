#include "traffic_light.h"
#include <std_msgs/String.h>
#include <gazebo/physics/physics.hh>

using namespace std; 

static void cb(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

namespace gazebo
{
    GZ_REGISTER_VISUAL_PLUGIN(Wr8TrafficLightPlugin)

    Wr8TrafficLightPlugin::Wr8TrafficLightPlugin(): counter(0), sphere_type(NONE)
	{
		ROS_INFO("Traffic Light Plugin is created!"); 
	}

    void Wr8TrafficLightPlugin::Load(rendering::VisualPtr vis, sdf::ElementPtr)
    {
        // You can run this plugin without ROS, for example to debug
        // In this way you can't use constructor of ros::NodeHandle and other features
		if( !ros::isInitialized() )
		{
			ROS_INFO("ROS has not beed initialized!"); 
		}
        // Or you can run it using ROS, so we can create instances
        else
		{
			ROS_INFO("Traffic Light has beed initialized!"); 
		}
        this->model_ = vis;
        this->update_connection_ = event::Events::ConnectPreRender(
                           boost::bind(&Wr8TrafficLightPlugin::OnUpdate, this));

        std::string name = this->model_->Name();
        if(name.find("topSphere") != -1){
            ROS_INFO("topSphere plugin is loaded!");
            sphere_type = Sphere_t::TOP;
        }
        else if(name.find("bottomSphere") != -1)
        {
            ROS_INFO("bottomSphere plugin is loaded!");
            sphere_type = Sphere_t::BOT;
        }

    }

    void Wr8TrafficLightPlugin::OnUpdate()
    {
        counter++;
        if(counter % 400 == 200)
        {
            if(Sphere_t::TOP == sphere_type){
                this->model_->SetMaterial("Gazebo/Grey");
            }
            else if(Sphere_t::BOT == sphere_type){
                this->model_->SetMaterial("Gazebo/Green");
            }
        }
        else if(counter % 400 == 0)
        {
            if(Sphere_t::TOP == sphere_type){
                this->model_->SetMaterial("Gazebo/Red");
            }
            else if(Sphere_t::BOT == sphere_type){
                this->model_->SetMaterial("Gazebo/Grey");
            }
        }
    }

	void Wr8TrafficLightPlugin::OnCmdTL()
	{
        ROS_INFO("OnCmdTL() called");
	}

	void Wr8TrafficLightPlugin::Reset()
	{
        ROS_INFO("Reset() called");
	}

	Wr8TrafficLightPlugin::~Wr8TrafficLightPlugin()
	{
        //n.shutdown();
        ROS_INFO("Destructor called");
	}
}
