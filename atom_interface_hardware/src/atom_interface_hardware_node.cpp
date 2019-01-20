#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <atom_interface/atom_interface.h>
#include <wiringPi.h>
#include <softPwm.h>

using namespace hardware_interface;

namespace ATOM_interface
{
// Hardware interface for ATOM platform
class ATOMHardwareInterface: public ATOMInterface
{
    public:
        ATOMHardwareInterface(ros::NodeHandle& nh);
        ~ATOMHardwareInterface();
        virtual void read();
        virtual void write();

    protected:
        int gpio_left_a;
        int gpio_left_b;
        int gpio_right_a;
        int gpio_right_b;
}; // class

ATOMHardwareInterface::ATOMHardwareInterface(ros::NodeHandle& nh) :
    ATOMInterface(nh)
{
    gpio_left_a = 5;
    gpio_left_b = 6;
    gpio_right_a = 13;
    gpio_right_b = 19;

    // Finally configure the hardware outputs
    wiringPiSetupGpio();
    pinMode(gpio_left_a, OUTPUT);
    pinMode(gpio_left_b, OUTPUT);
    pinMode(gpio_right_a, OUTPUT);
    pinMode(gpio_right_b, OUTPUT);

    // Hardware PWM only works on pin 18 (LED)
    // Set PWM range from 0 to 100 (percent)
    // Minimum pulse width is 100us
    // 100us * 100 = 10ms = 100Hz
    softPwmCreate(gpio_left_a, 0, 100);
    softPwmCreate(gpio_left_b, 0, 100);
    softPwmCreate(gpio_right_a, 0, 100);
    softPwmCreate(gpio_right_b, 0, 100);
}

ATOMHardwareInterface::~ATOMHardwareInterface()
{
    // Cleanup GPIO (also stops the motors)
    softPwmWrite(gpio_left_a, 0);
    softPwmWrite(gpio_left_b, 0);
    softPwmWrite(gpio_right_a, 0);
    softPwmWrite(gpio_right_b, 0);

    // Set GPIO back to inputs
    pinMode(gpio_left_a, INPUT);
    pinMode(gpio_left_b, INPUT);
    pinMode(gpio_right_a, INPUT);
    pinMode(gpio_right_b, INPUT);
}

void ATOMHardwareInterface::read()
{
    // Read sensors (encoders, odometry...)
    /*for (int i = 0; i < num_joints_; i++)
    {
        // Nothing to read from joints
    }*/
}

void ATOMHardwareInterface::write()
{
    int gpio_a;
    int gpio_b;

    // Write command to the drive motor joints
    for(int i = 0; i < num_joints_; i++)
    {
        // As there are no sensors on the robot the "joint_velocity_" vector
        // shall be updated here instead of in read()
        joint_velocity_[i] = joint_velocity_command_[i];

        // Dummy way to set wheel angle for proper rviz display
        joint_position_[i] = joint_position_[i] + joint_velocity_command_[i];

        if(joint_names_[i].find("left") > 0)
        {
            // Left wheels
            gpio_a = gpio_left_b;
            gpio_b = gpio_left_a;
        }
        else if(joint_names_[i].find("right") > 0)
        {
            // Right wheels
            gpio_a = gpio_right_a;
            gpio_b = gpio_right_b;
        }
        else
        {
            ROS_FATAL("Could not define joint side");
            exit(1);
        }

        // Set PWM based on joint_velocity_command_
        if(joint_velocity_command_[i] > 0)
        {
            // Forwards
            softPwmWrite(gpio_a, 30);
            softPwmWrite(gpio_b, 0);
        }
        else if(joint_velocity_command_[i] < 0)
        {
            // Backwards
            softPwmWrite(gpio_a, 0);
            softPwmWrite(gpio_b, 30);
        }
        else
        {
            // Stop
            softPwmWrite(gpio_a, 0);
            softPwmWrite(gpio_b, 0);
        }
    }
}

} // namespace

// Main node implementation
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ATOM_hardware_interface");
    ros::NodeHandle nh;

    // This is needed or the node will not subscribe to the cmd_vel messages
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // ATOM instance
    ATOM_interface::ATOMHardwareInterface ATOM_hardware_interface(nh);

    // Call the control loop, default rate: 10Hz
    double loop_hz;
    nh.param("diff_drive_controller/publish_rate", loop_hz, 10);
    ros::Rate rate(loop_hz);
    while(ros::ok())
    {
        ATOM_hardware_interface.update();
        rate.sleep();
    }

    return 0;
}
