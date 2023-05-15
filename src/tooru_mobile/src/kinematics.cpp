#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include "rclcpp/rclcpp.hpp"
#include <thread>


// Set up parameters
const std::string JOINTSTATE_TOPIC = "/joint_state";
const std::string TWIST_TOPIC = "/twist_cmd";

// bool convertTwistToJoint(const geometry_msgs::msg::TwistStamped& twist,
//                      std::unique_ptr<sensor_msgs::msg::JointState>& joint)
// {
//   // map buttons to twist commands
//   // twist->twist.linear.z = axes[RIGHT_STICK_Y];
//   // twist->twist.linear.y = axes[RIGHT_STICK_X];

//   // double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
//   // double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
//   // twist->twist.linear.x = lin_x_right + lin_x_left;

//   // twist->twist.angular.y = axes[LEFT_STICK_Y];
//   // twist->twist.angular.x = axes[LEFT_STICK_X];

//   // double roll_positive = buttons[RIGHT_BUMPER];
//   // double roll_negative = -1 * (buttons[LEFT_BUMPER]);
//   // twist->twist.angular.z = roll_positive + roll_negative;

//   twist->twist.linear.x = axes[LEFT_STICK_X];
//   twist->twist.linear.y = axes[LEFT_STICK_Y];
//   twist->twist.angular.z = axes[RIGHT_STICK_X];


//   return true;
// }

namespace tooru_mobile
{
    class TwistToJointPub : public rclcpp::Node
    {
        public:
            //TwistToJointPub(const rclcpp::NodeOptions& options) : Node("twist_to_joint_publisher", options)
            TwistToJointPub(const rclcpp::NodeOptions& options) : Node("twist_to_joint_publisher", options)
            {
                // Setup pub/sub
                twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS(),
                                                                    [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg) {
                                                                        return twistCB(msg);
                                                                    });

                joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(JOINTSTATE_TOPIC, rclcpp::SystemDefaultsQoS());
            }

            void twistCB(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg)
            {
               // Create the messages we might publish
                auto joint_msg = std::make_unique<sensor_msgs::msg::JointState>();

                joint_msg->velocity.resize(4);

                joint_msg->header.frame_id = "Mobile";
                joint_msg->velocity[0] = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
                joint_msg->velocity[1] = 1.0;
                joint_msg->velocity[2] = 1.0;
                joint_msg->velocity[3] = 1.0;
                joint_msg->header.stamp = now();

                joint_pub_->publish(std::move(joint_msg));
            }
        private:
            rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    };       
}

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<tooru_mobile::TwistToJointPub>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tooru_mobile::TwistToJointPub)