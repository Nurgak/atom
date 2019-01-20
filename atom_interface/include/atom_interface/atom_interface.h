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
