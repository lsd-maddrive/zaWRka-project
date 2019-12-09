#pragma once 

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>

namespace gazebo
{

class Wr8TrafficLightPlugin : public ModelPlugin {
public:
	Wr8TrafficLightPlugin();
	virtual ~Wr8TrafficLightPlugin();

protected:
	virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
	virtual void Reset(); 

private:
	void OnCmdTL();
	void OnUpdate(const common::UpdateInfo& info); 

	ros::NodeHandle n;
	ros::Subscriber sub_tl; 

	enum states { NO_COLORS, RED, GREEN };

	};



GZ_REGISTER_MODEL_PLUGIN(Wr8TrafficLightPlugin)
}