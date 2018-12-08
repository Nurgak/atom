#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <ATOMInterface/ATOMInterface.h>

extern "C"
{
#define NON_MATLAB_PARSING 1
#define MAX_EXT_API_CONNECTIONS 255
#define DO_NOT_USE_SHARED_MEMORY 1
#include <programming/include/v_repConst.h>
#include <programming/remoteApi/extApi.h>
#include <programming/remoteApi/extApiInternal.h>
#include <programming/remoteApi/extApi.c>
#include <programming/remoteApi/extApiPlatform.c>
}

using namespace hardware_interface;

namespace ATOM_interface
{
// V-REP interface for ATOM platform
class ATOMVREPInterface: public ATOMInterface
{
    public:
        ATOMVREPInterface(ros::NodeHandle& nh);
        ~ATOMVREPInterface();
        virtual void read();
        virtual void write();
    protected:
        std::vector<int> joint_handle_;
        int clientID;
};

ATOMVREPInterface::ATOMVREPInterface(ros::NodeHandle& nh): ATOMInterface(nh)
{
    // Override the interface constructor

    // Get the simulation client ID
    // Port 19997, wait until connected (blocking), do not attemp reconnecting,
    // timeout of 5 seconds, packet exchange of 5ms
    clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 5000, 5);

    if(clientID == -1)
    {
        ROS_ERROR("Could not connect to V-REP");
        return;
    }
    ROS_INFO("Connected to V-REP");

    // Resize the V-REP handles for the joints
    joint_handle_.resize(num_joints_);

    // Initialize Controller
    int handleID;
    for(int i = 0; i < num_joints_; ++i)
    {
        // Get the joint handle from V-REP
        simxGetObjectHandle(clientID, joint_names_[i].c_str(), &handleID, simx_opmode_blocking);
        // Save the handle in the joint handle array
        joint_handle_[i] = handleID;
    }
}

ATOMVREPInterface::~ATOMVREPInterface()
{
    // Write command to the wheels
    for(int i = 0; i < num_joints_; i++)
    {
        joint_velocity_[i] = 0.0;
    }

    write();

    // Close connection to the simulator
    simxFinish(clientID);
}

void ATOMVREPInterface::read()
{
    // Read sensors (encoders, odometry...)
    float position;
    for (int i = 0; i < num_joints_; i++)
    {
        simxGetJointPosition(clientID, joint_handle_[i], &position, simx_opmode_buffer);
        joint_position_[i] = position;
    }
}

void ATOMVREPInterface::write()
{
    // Write command to the wheels
    for(int i = 0; i < num_joints_; i++)
    {
        // Set the joint target speed in V-REP
        simxSetJointTargetVelocity(clientID, joint_handle_[i], joint_velocity_command_[i], simx_opmode_oneshot);

        // As there are no sensors on the robot the joint_velocity_ shall be updated here instead of in read()
        joint_velocity_[i] = joint_velocity_command_[i];
    }
}

} // namespace

// Main node implementation
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ATOM_vrep_interface");
    ros::NodeHandle nh;

    // This is needed or the node will not subscribe to the cmd_vel messages
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ATOM instance
    ATOM_interface::ATOMVREPInterface ATOM_VREP_simulation(nh);

    // Call the control loop at 10Hz by default
    double loop_hz;
    nh.param("hardware_interface/loop_hz", loop_hz, 10.0);
    ROS_INFO("Update rate: %f", loop_hz);
    ros::Rate rate(loop_hz);
    while(ros::ok())
    {
        ATOM_VREP_simulation.update();
        rate.sleep();
    }

    return 0;
}
