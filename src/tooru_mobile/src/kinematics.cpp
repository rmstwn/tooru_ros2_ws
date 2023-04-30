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
const std::string JOINTSTATE_TOPIC = "/joint_vel";
const std::string TWIST_TOPIC = "/twist_cmd";



namespace tooru_kinematics
{
    class TwistToJointPub : public rclcpp::Node
    {
        public:
            //TwistToJointPub(const rclcpp::NodeOptions& options) : Node("twist_to_joint_publisher", options)
            TwistToJointPub() : Node("twist_to_joint_publisher")
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

                joint_msg->header.frame_id = "Mobile";
                joint_msg->velocity[0] = 1;
                joint_msg->velocity[1] = 1;
                joint_msg->velocity[2] = 1;
                joint_msg->velocity[3] = 1;
                joint_msg->header.stamp = now();

                joint_pub_->publish(std::move(joint_msg));
            }
        private:
            rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    };       
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tooru_kinematics::TwistToJointPub>());
  rclcpp::shutdown();
  return 0;
}

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(tooru_kinematics::JointStatePub)