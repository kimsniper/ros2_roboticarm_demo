#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "ros2_pca9685/srv/set_pwm.hpp"

#include <cmath>
#include <chrono>

using std::placeholders::_1;

class JointsProcessor : public rclcpp::Node
{
public:
    JointsProcessor(const std::string& name) 
        : Node(name)
    {
        sub_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", 
                                                                 10, 
                                                                 std::bind(&JointsProcessor::SubCallback, this, _1));

        threads_.push_back(std::thread(std::bind(&JointsProcessor::JointControl, this, base_plate_joint)));
        threads_.push_back(std::thread(std::bind(&JointsProcessor::JointControl, this, forward_drive_arm_joint)));
        threads_.push_back(std::thread(std::bind(&JointsProcessor::JointControl, this, horizontal_arm_joint)));
        threads_.push_back(std::thread(std::bind(&JointsProcessor::JointControl, this, claw_support_joint)));
        RCLCPP_INFO(get_logger(), "Joints control node started");
    }

private:

    void SubCallback(const sensor_msgs::msg::JointState &msg)
    {
        for ( int i = 0; i < static_cast<int>(joint_pose.size()); ++i )
        {
            joint_pose.at(i) = round((msg.position.at(i) + (PI_VAL / 2)) * (180 / PI_VAL));
            //RCLCPP_INFO_STREAM(get_logger(), msg.name.at(i) << ": " << joint_pose.at(i));
        }
    }

    void JointControl(std::uint8_t joint_num)
    {
        auto client = this->create_client<ros2_pca9685::srv::SetPwm>("/pca9685/set_pwm");

        RCLCPP_INFO(this->get_logger(), "Joint task created.");
        RCLCPP_INFO(this->get_logger(), "Waiting for server to be up. . .");

        while(!client->wait_for_service(std::chrono::seconds(1)));

        auto request = std::make_shared<ros2_pca9685::srv::SetPwm::Request>();

        request->channel_num = joint_num;
        request->target_position = joint_pose.at(joint_num);

        //auto future = client->async_send_request(request);

        // try
        // {
        //     auto response = future.get();
        //     RCLCPP_INFO_STREAM(this->get_logger(), response->response);
        // }
        // catch(const std::exception& e)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Service call error!");
        // }

        while (rclcpp::ok()) 
        {
            if(joint_pose.at(joint_num) > (request->target_position + ANGLE_STEP))
            {
                client->async_send_request(request);
                request->target_position += ANGLE_STEP;
            }
            else if(joint_pose.at(joint_num) < request->target_position)
            {
                client->async_send_request(request);
                request->target_position -= ANGLE_STEP;
            }
            else
            {
                /* do nothing */
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(ANGLE_STEP_DEL));
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    std::array<double, 4> joint_pose = {0.0, 0.0, 0.0, 0.0};
    std::vector<std::thread> threads_;

    /* joint names - this joint arrangement is also used for pca9685 channel assignment */
    static constexpr std::uint8_t base_plate_joint = 0;
    static constexpr std::uint8_t forward_drive_arm_joint = 1;
    static constexpr std::uint8_t horizontal_arm_joint = 2;
    static constexpr std::uint8_t claw_support_joint = 3;

    static constexpr double PI_VAL = 3.14159;

    /* Servo motor speed tuning */
    static constexpr double ANGLE_STEP = 0.2;   /* Angle increment/decrement value */
    static constexpr int64_t ANGLE_STEP_DEL = 1; /* Angle step delay */
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointsProcessor>("joints_processor");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
