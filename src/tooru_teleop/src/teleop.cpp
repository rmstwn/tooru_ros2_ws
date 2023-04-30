#include <sensor_msgs/msg/joy.hpp>
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
const std::string JOY_TOPIC = "/joy";
const std::string TWIST_TOPIC = "/twist_cmd";

// Enums for button names -> axis/button array index
// For Logicool Gamepad F310 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5,
  D_PAD_X = 6,
  D_PAD_Y = 7
};
enum Button
{
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  BACK = 6,
  START = 7,
  HOME = 8,
  LEFT_STICK_CLICK = 9,
  RIGHT_STICK_CLICK = 10
};

// Some axes have offsets (e.g. the default trigger position is 1.0 not 0)
// Map the default values for the axes
const std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };

// To change controls or setup a new controller, all you should to do is change the above enums and the follow 2
// functions
/** \brief // This converts a joystick axes and buttons array to a TwistStamped or JointJog message
 * @param axes The vector of continuous controller joystick axes
 * @param buttons The vector of discrete controller button values
 * @param twist A TwistStamped message to update in prep for publishing
 * @param joint A JointJog message to update in prep for publishing
 * @return return true if you want to publish a Twist, false if you want to publish a JointJog
 */

bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist)
{
  // map buttons to twist commands
  // twist->twist.linear.z = axes[RIGHT_STICK_Y];
  // twist->twist.linear.y = axes[RIGHT_STICK_X];

  // double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  // double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  // twist->twist.linear.x = lin_x_right + lin_x_left;

  // twist->twist.angular.y = axes[LEFT_STICK_Y];
  // twist->twist.angular.x = axes[LEFT_STICK_X];

  // double roll_positive = buttons[RIGHT_BUMPER];
  // double roll_negative = -1 * (buttons[LEFT_BUMPER]);
  // twist->twist.angular.z = roll_positive + roll_negative;

  twist->twist.linear.x = axes[LEFT_STICK_X];
  twist->twist.linear.y = axes[LEFT_STICK_Y];
  twist->twist.angular.z = axes[RIGHT_STICK_X];


  return true;
}

namespace tooru_teleop
{
    class JoyToTwistPub : public rclcpp::Node
    {
        public:
            JoyToTwistPub(const rclcpp::NodeOptions& options) : Node("joy_to_twist_publisher", options)
            {
                // Setup pub/sub
                joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
                                                                    [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) {
                                                                        return joyCB(msg);
                                                                    });

                twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS());
            
            }

            void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
            {
                // Create the messages we might publish
                auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            
                // Convert the joystick message to Twist or JointJog and publish
                if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg))
                {
                // publish the TwistStamped
                twist_msg->header.frame_id = "Mobile";
                twist_msg->header.stamp = now();
                twist_pub_->publish(std::move(twist_msg));
                }
            }
        private:
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    };
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tooru_teleop::JoyToTwistPub)