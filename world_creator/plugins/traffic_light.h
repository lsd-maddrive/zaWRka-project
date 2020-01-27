#ifndef MY_PLUGIN_H
#define MY_PLUGIN_H

#include <ros/ros.h>

#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{

class Wr8TrafficLightPlugin : public VisualPlugin
{
public:
	Wr8TrafficLightPlugin();
	virtual ~Wr8TrafficLightPlugin();

protected:
	void Load(rendering::VisualPtr _parent, sdf::ElementPtr sdf);
	virtual void Reset(); 
private:
    enum Sphere_t {NONE, TOP, BOT};
	enum states { NO_COLORS, RED, GREEN };

    void OnCmdTL();
    void OnUpdate();

    rendering::VisualPtr model_;
    event::ConnectionPtr update_connection_;

    //ros::NodeHandle n;
    //ros::Subscriber sub_tl;

    Sphere_t sphere_type;
    int counter;
};

} // end namespace Gazebo
#endif // MY_PLUGIN_H
