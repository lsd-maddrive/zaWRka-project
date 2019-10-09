#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace gazebo {

// Kinematics parameters
#define WR8_STEERING_RATIO      1       // Ratio between steering wheel angle and tire angle

// Drag parameters
#define ROLLING_RESISTANCE_COEFF  0.01
#define AERO_DRAG_COEFF           0.35
#define GRAVITY_ACCEL             9.81
#define VEHICLE_MASS              1700.0
#define WHEEL_RADIUS              0.36
#define MAX_BRAKE_TORQUE          3000.0

class Wr8InterfacePlugin : public ModelPlugin {
public:
    Wr8InterfacePlugin();
    virtual ~Wr8InterfacePlugin();

protected:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    virtual void Reset();

private:
    void onCmdVel(const geometry_msgs::Twist& command);

    void twistTimerCallback(const ros::TimerEvent& event);
    void tfTimerCallback(const ros::TimerEvent& event);
    void OnUpdate(const common::UpdateInfo& info);

    void twistStateUpdate();
    void driveUpdate();
    void steeringUpdate(const common::UpdateInfo& info);
    void dragUpdate();
    void stopWheels();
    void setAllWheelTorque(double torque);
    void setRearWheelTorque(double torque);

    ros::NodeHandle* n_;
    ros::Publisher pub_twist_;
    ros::Subscriber sub_vel_cmd_;
    ros::Timer twist_timer_;
    ros::Timer tf_timer_;

    tf::TransformBroadcaster br_;
    geometry_msgs::Twist twist_;
    bool rollover_;
    #if GAZEBO_MAJOR_VERSION >= 9
    ignition::math::Pose3d world_pose_;
    #else
    gazebo::math::Pose world_pose_;
    #endif
    event::ConnectionPtr update_connection_;
    physics::JointPtr steer_fl_joint_;
    physics::JointPtr steer_fr_joint_;
    physics::JointPtr wheel_rl_joint_;
    physics::JointPtr wheel_rr_joint_;
    physics::JointPtr wheel_fl_joint_;
    physics::JointPtr wheel_fr_joint_;
    physics::LinkPtr footprint_link_;
    common::Time last_update_time_;

    // SDF parameters
    std::string robot_name_;
    bool pub_tf_;
    double max_steer_rad_;
    double tf_freq_;
    double wheelbase_;
    double track_width_;

    // Steering values
    double right_angle_;
    double left_angle_;
    double target_angle_;
    double current_steering_angle_;

    // Brakes
    double brake_cmd_;
    ros::Time brake_stamp_;

    // Throttle
    double throttle_cmd_;
    ros::Time throttle_stamp_;
    };

    GZ_REGISTER_MODEL_PLUGIN(Wr8InterfacePlugin)
}
