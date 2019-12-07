#pragma once 

namespace gazebo
{

	class Wr8TrafficLightPlugin {
	public:
		Wr8TrafficLightPlugin();
		virtual ~Wr8TrafficLightPlugin();

	protected:
		virtual void Load();
		virtual void Reset(); 

	}
}