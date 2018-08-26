#pragma once

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

using namespace hardware_interface;

namespace ATOM_interface
{

// Abstract interface for ATOM platform
class ATOMInterface: public hardware_interface::RobotHW 
{
    public:
        ATOMInterface(ros::NodeHandle& nh);
        virtual ~ATOMInterface() {}

        // Pure virtual methods, need use specific definition
        virtual void read() = 0;
        virtual void write() = 0;

        // Update calls read, write and controller manager
        virtual void update();

    protected:
        // Interfaces
        hardware_interface::JointStateInterface joint_state_interface_;
        // ATOM uses diff_drive_controller which only works with velocity interface
        hardware_interface::VelocityJointInterface velocity_joint_interface_;

        // Joints
        int num_joints_;
        std::vector<std::string> joint_names_;
        std::vector<double> joint_position_;
        std::vector<double> joint_velocity_;
        std::vector<double> joint_effort_;
        std::vector<double> joint_velocity_command_;

        // Control loop variables
        ros::NodeHandle nh_;
        ros::Time current_time_;
        ros::Time last_time_;
        ros::Duration elapsed_time_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
}; // class

} // namespace
