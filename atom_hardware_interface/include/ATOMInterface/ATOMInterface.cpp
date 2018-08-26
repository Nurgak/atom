#include "ATOMInterface.h"

namespace ATOM_interface
{

ATOMInterface::ATOMInterface(ros::NodeHandle& nh) : nh_(nh)
{
    // Get joint names
    std::vector<std::string> joint_names_left_;
    std::vector<std::string> joint_names_right_;

    nh_.getParam("diff_drive_controller/left_wheel", joint_names_left_);
    joint_names_.insert(joint_names_.end(), joint_names_left_.begin(),
        joint_names_left_.end());

    nh_.getParam("diff_drive_controller/right_wheel", joint_names_right_);
    joint_names_.insert(joint_names_.end(), joint_names_right_.begin(),
        joint_names_right_.end());

    num_joints_ = joint_names_.size();

    if(num_joints_ == 0)
    {
        ROS_FATAL("Did not find joints");
        exit(1);
    }

    // Resize vectors
    joint_position_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);

    // Initialize Controller 
    for(int i = 0; i < num_joints_; ++i)
    {
        // Create joint state interface
        JointStateHandle jointStateHandle(
            joint_names_[i],
            &joint_position_[i],
            &joint_velocity_[i],
            &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create velocity joint interface
        JointHandle jointVelocityHandle(jointStateHandle,
                                        &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);
    }

    // Register the interfaces for the joint states and interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
}

void ATOMInterface::update()
{
    current_time_ = ros::Time::now();
    elapsed_time_ = ros::Duration(current_time_ - last_time_);
    last_time_ = current_time_;
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write();
}

} // namespace
