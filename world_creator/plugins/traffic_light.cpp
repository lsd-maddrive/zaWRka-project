#include "traffic_light.h"


using namespace std; 

namespace gazebo
{

	Wr8TrafficLightPlugin::Wr8TrafficLightPlugin()
	{
		
		cout << "Traffic Light Plugin is created!" << endl; 
	}

	void Wr8TrafficLightPlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
	{
		if( !ros::isInitialized() )
		{
			cout << "Traffic Light has not beed initialized, unable to load plugin" << cout; 
			return; 
		}

		// in case if you want several traffic lights 
		// you may want to give them names ('Traffic Light 1', etc.)
		if( sdf -> HasElement("tl_name"))
		{

		}

		n = ros::NodeHandle();
		sub_tl = n.subscribe("topik_name") 

	}

	void Wr8TrafficLightPlugin::OnUpdate(const common::UpdateInfo& info)
	{

	}

	void Wr8TrafficLightPlugin::OnCmdTL()
	{

	}

	void Wr8TrafficLightPlugin::Reset()
	{

	}

	Wr8TrafficLightPlugin::~Wr8TrafficLightPlugin()
	{
		n.shutdown();
	}
}
