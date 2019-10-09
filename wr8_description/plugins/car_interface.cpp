#include "car_interface.h"

using namespace std;

namespace gazebo
{

Wr8InterfacePlugin::Wr8InterfacePlugin()
{
    target_angle_ = 0.0;
    brake_cmd_ = 0.0;
    throttle_cmd_ = 0.0;
    current_steering_angle_ = 0.0;
    rollover_ = false;

    cout << "Wr8 plugin created!" << endl;
}

void Wr8InterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
    // Gazebo initialization
    steer_fl_joint_ = model->GetJoint("fl_steer_joint");
    steer_fr_joint_ = model->GetJoint("fr_steer_joint");
    wheel_rl_joint_ = model->GetJoint("rl_speed_joint");
    wheel_rr_joint_ = model->GetJoint("rr_speed_joint");
    wheel_fl_joint_ = model->GetJoint("fl_speed_joint");
    wheel_fr_joint_ = model->GetJoint("fr_speed_joint");
    footprint_link_ = model->GetLink("base_footprint");

    assert(steer_fl_joint_);
    assert(steer_fr_joint_);
    assert(wheel_rl_joint_);
    assert(wheel_rr_joint_);
    assert(wheel_fl_joint_);
    assert(wheel_fr_joint_);
    assert(footprint_link_);

    // Load SDF parameters
    if (sdf->HasElement("pubTf"))
    {
        sdf->GetElement("pubTf")->GetValue()->Get(pub_tf_);
    }
    else
    {
        pub_tf_ = false;
    }

    if (sdf->HasElement("robotName"))
    {
        sdf::ParamPtr sdf_robot_name = sdf->GetElement("robotName")->GetValue();
        if (sdf_robot_name)
        {
            sdf_robot_name->Get(robot_name_);
        }
        else
        {
            robot_name_ = std::string("");
        }
    }
    else
    {
        robot_name_ = std::string("");
    }

    // if (sdf->HasElement("pubTf"))
    // {
    //     sdf->GetElement("pubTf")->GetValue()->Get(pub_tf_);
    // }
    // else
    // {
    //     pub_tf_ = false;
    // }

    if (sdf->HasElement("maxSteerRad"))
    {
        sdf->GetElement("maxSteerRad")->GetValue()->Get(max_steer_rad_);
    }
    else
    {
        max_steer_rad_ = M_PI * 30 / 180;
    }

    if (sdf->HasElement("wheelbase"))
    {
        sdf->GetElement("wheelbase")->GetValue()->Get(wheelbase_);
    }
    else
    {
        wheelbase_ = 0.3;
    }

    if (sdf->HasElement("trackWidth"))
    {
        sdf->GetElement("trackWidth")->GetValue()->Get(track_width_);
    }
    else
    {
        track_width_ = 0.23;
    }

    if (sdf->HasElement("tfFreq"))
    {
        sdf->GetElement("tfFreq")->GetValue()->Get(tf_freq_);
    }
    else
    {
        tf_freq_ = 100.0;
    }

    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&Wr8InterfacePlugin::OnUpdate, this, _1));

    steer_fl_joint_->SetParam("fmax", 0, 99999.0);
    steer_fr_joint_->SetParam("fmax", 0, 99999.0);

    // ROS initialization
    n_ = new ros::NodeHandle(robot_name_);

    sub_vel_cmd_ = n_->subscribe("cmd_vel", 1, &Wr8InterfacePlugin::onCmdVel, this);

    cout << "Wr8 plugin loaded!" << endl;
}

void Wr8InterfacePlugin::OnUpdate(const common::UpdateInfo &info)
{
    if (last_update_time_ == common::Time(0))
    {
        last_update_time_ = info.simTime;
        return;
    }

    // twistStateUpdate();
    // driveUpdate();
    steeringUpdate(info);
    // dragUpdate();
}

void Wr8InterfacePlugin::twistStateUpdate()
{
#if GAZEBO_MAJOR_VERSION >= 9
    world_pose_ = footprint_link_->WorldPose();
    twist_.linear.x = footprint_link_->RelativeLinearVel().X();
    twist_.angular.z = footprint_link_->RelativeAngularVel().Z();
    rollover_ = (fabs(world_pose_.Rot().X()) > 0.2 || fabs(world_pose_.Rot().Y()) > 0.2);
#else
    world_pose_ = footprint_link_->GetWorldPose();
    twist_.linear.x = footprint_link_->GetRelativeLinearVel().x;
    twist_.angular.z = footprint_link_->GetRelativeAngularVel().z;
    rollover_ = (fabs(world_pose_.rot.x) > 0.2 || fabs(world_pose_.rot.y) > 0.2);
#endif
}

void Wr8InterfacePlugin::driveUpdate()
{
    // Stop wheels if vehicle is rolled over
    if (rollover_)
    {
        stopWheels();
        return;
    }

    // Brakes have precedence over throttle
    ros::Time current_stamp = ros::Time::now();
    if ((brake_cmd_ > 0) && ((current_stamp - brake_stamp_).toSec() < 0.25))
    {
        double brake_torque_factor = 1.0;
        if (twist_.linear.x < -0.1)
        {
            brake_torque_factor = -1.0;
        }
        else if (twist_.linear.x < 0.1)
        {
            brake_torque_factor = 1.0 + (twist_.linear.x - 0.1) / 0.1;
        }

        setAllWheelTorque(-brake_torque_factor * brake_cmd_);
    }
    else
    {
        if ((current_stamp - throttle_stamp_).toSec() < 0.25)
        {
            double max_throttle_torque = throttle_cmd_ * 4000.0 - 40.1 * twist_.linear.x;
            if (max_throttle_torque < 0.0)
            {
                max_throttle_torque = 0.0;
            }
            setRearWheelTorque(max_throttle_torque);
        }
    }
}

void Wr8InterfacePlugin::steeringUpdate(const common::UpdateInfo &info)
{
    double time_step = (info.simTime - last_update_time_).Double();
    last_update_time_ = info.simTime;

    // Arbitrarily set maximum steering rate to 800 deg/s
    const double max_rate = 800.0 * M_PI / 180.0 * WR8_STEERING_RATIO;
    double max_inc = time_step * max_rate;

    if ((target_angle_ - current_steering_angle_) > max_inc)
    {
        current_steering_angle_ += max_inc;
    }
    else if ((target_angle_ - current_steering_angle_) < -max_inc)
    {
        current_steering_angle_ -= max_inc;
    }

    // Compute Ackermann steering angles for each wheel
    double t_alph = tan(current_steering_angle_);
    double left_steer = atan(wheelbase_ * t_alph / (wheelbase_ - 0.5 * track_width_ * t_alph));
    double right_steer = atan(wheelbase_ * t_alph / (wheelbase_ + 0.5 * track_width_ * t_alph));

#if GAZEBO_MAJOR_VERSION >= 9
    steer_fl_joint_->SetParam("vel", 0, 100.0 * (left_steer - steer_fl_joint_->Position(0)));
    steer_fr_joint_->SetParam("vel", 0, 100.0 * (right_steer - steer_fr_joint_->Position(0)));
#else
    steer_fl_joint_->SetParam("vel", 0, 100.0 * (left_steer - steer_fl_joint_->GetAngle(0).Radian()));
    steer_fr_joint_->SetParam("vel", 0, 100.0 * (right_steer - steer_fr_joint_->GetAngle(0).Radian()));
#endif
}

void Wr8InterfacePlugin::dragUpdate()
{
    // Apply rolling resistance and aerodynamic drag forces
    double rolling_resistance_torque = ROLLING_RESISTANCE_COEFF * VEHICLE_MASS * GRAVITY_ACCEL;
    double drag_force = AERO_DRAG_COEFF * twist_.linear.x * twist_.linear.x;
    double drag_torque = drag_force * WHEEL_RADIUS; // Implement aerodynamic drag as a torque disturbance

    if (twist_.linear.x > 0.0)
    {
        setAllWheelTorque(-rolling_resistance_torque);
        setAllWheelTorque(-drag_torque);
    }
    else
    {
        setAllWheelTorque(rolling_resistance_torque);
        setAllWheelTorque(drag_torque);
    }
}

void Wr8InterfacePlugin::setAllWheelTorque(double torque)
{
    wheel_rl_joint_->SetForce(0, 0.25 * torque);
    wheel_rr_joint_->SetForce(0, 0.25 * torque);
    wheel_fl_joint_->SetForce(0, 0.25 * torque);
    wheel_fr_joint_->SetForce(0, 0.25 * torque);
}

void Wr8InterfacePlugin::onCmdVel(const geometry_msgs::Twist& command)
{
    target_angle_ = command.angular.z * WR8_STEERING_RATIO;
    if (target_angle_ > max_steer_rad_)
    {
        target_angle_ = max_steer_rad_;
    }
    else if (target_angle_ < -max_steer_rad_)
    {
        target_angle_ = -max_steer_rad_;
    }

    ROS_INFO_STREAM(target_angle_ << " / " << max_steer_rad_ );

    // wheel_rl_joint_->SetForce(0, 0.5 * torque);
    // wheel_rr_joint_->SetForce(0, 0.5 * torque);
}

void Wr8InterfacePlugin::stopWheels()
{
    wheel_fl_joint_->SetForce(0, -1000.0 * wheel_fl_joint_->GetVelocity(0));
    wheel_fr_joint_->SetForce(0, -1000.0 * wheel_fr_joint_->GetVelocity(0));
    wheel_rl_joint_->SetForce(0, -1000.0 * wheel_rl_joint_->GetVelocity(0));
    wheel_rr_joint_->SetForce(0, -1000.0 * wheel_rr_joint_->GetVelocity(0));
}

void Wr8InterfacePlugin::Reset()
{
}

Wr8InterfacePlugin::~Wr8InterfacePlugin()
{
    n_->shutdown();
    delete n_;
}

} // namespace gazebo
