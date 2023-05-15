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
#include <math.h>
#include <eigen3/Eigen/Dense> 

// Set up parameters
const std::string JOINTSTATE_TOPIC = "/joint_state";
const std::string TWIST_TOPIC = "/twist_cmd";

rclcpp::Clock system_clock(RCL_SYSTEM_TIME);

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
        // TwistToJointPub(const rclcpp::NodeOptions& options) : Node("twist_to_joint_publisher", options)
        TwistToJointPub(const rclcpp::NodeOptions &options) : Node("twist_to_joint_publisher", options)
        {
            // Setup pub/sub
            twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, rclcpp::SystemDefaultsQoS(),
                                                                               [this](const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg)
                                                                               {
                                                                                   return twistCB(msg);
                                                                               });

            joint_pub_ = create_publisher<sensor_msgs::msg::JointState>(JOINTSTATE_TOPIC, rclcpp::SystemDefaultsQoS());
        }

        void twistCB(const geometry_msgs::msg::TwistStamped::ConstSharedPtr &msg)
        {
            // Create the messages we might publish
            auto joint_msg = std::make_unique<sensor_msgs::msg::JointState>();

            joint_msg->velocity.resize(4);

            joint_msg->header.frame_id = "Mobile";
            joint_msg->velocity[0] = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
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

    class MobileInfo
    {
    public:
        double Odometry[3];
        double Position[3];
    };

    class JointState
    {
    public:
        double RPM[4];
    };

    double AxisLength = 0.16;
    double WheelDiameter = 0.1;
    double Delta = 45; // degree
    double Sin = sin(Delta * (M_PI / 180));
    double Cos = cos(Delta * (M_PI / 180));

    class Mobile
    {

        class Omnidirectional
        {
            /* Omnidirectional
               robot:	L: AxisLength

               ID=14  |###|       |###|  ID=11
                    |###|           |###|          y
                  |###| ############# |###|        ^
                        #############              ¦
                        #############     front    ¦
                        #############              ¦
                  |###| ############# |###|       -¦-----> x
                    |###|           |###|
               ID=13  |###|       |###|  ID=12
                              |   L   |
                              |<----->|
            */

            double odom[3] = {0, 0, 0};
            double cpose[3] = {0, 0, 0};

            double last_time;
            double last_odom[3];
            double dt;

            MobileInfo MobileOmni;
            JointState JointOmni;

            MobileInfo ForwardKinematics(double rpm[])
            {
                uint64_t current_time = system_clock.now().nanoseconds();

                double velocity[] = {0, 0, 0, 0};
                velocity[0] = (2 * M_PI * (WheelDiameter / 2)) * (rpm[0] / 60);
                velocity[1] = (2 * M_PI * (WheelDiameter / 2)) * (rpm[1] / 60);
                velocity[2] = (2 * M_PI * (WheelDiameter / 2)) * (rpm[2] / 60);
                velocity[3] = (2 * M_PI * (WheelDiameter / 2)) * (rpm[3] / 60);
                double move_vel_x = velocity[0] * Sin - velocity[2] * Sin - velocity[1] * Cos + velocity[3] * Cos;
                double move_vel_y = -velocity[0] * Sin + velocity[2] * Cos - velocity[1] * Sin + velocity[3] * Sin;
                double move_yawrate = -(velocity[0] + velocity[1] + velocity[2] + velocity[3]) / AxisLength;
                // positions:
                if (last_time > 0 || last_time < 0)
                {
                    dt = (current_time - last_time) / 1000;
                    if (dt > 0 && dt < 1)
                    {
                        // compute second order midpoint velocities
                        double vel_x_mid = 0.5 * (move_vel_x + last_odom[0]);
                        double vel_y_mid = 0.5 * (move_vel_y + last_odom[1]);
                        double yawrate_mid = 0.5 * (move_yawrate + last_odom[2]);
                        // compute midpoint yaw angle
                        double yaw_mid = cpose[2] + 0.5 * yawrate_mid * dt;
                        // integrate position using midpoint velocities and yaw angle
                        cpose[0] += vel_x_mid * dt * cos(yaw_mid) + vel_y_mid * dt * -sin(yaw_mid);
                        cpose[1] += vel_x_mid * dt * sin(yaw_mid) + vel_y_mid * dt * cos(yaw_mid);
                        // integrate yaw angle using midpoint yawrate
                        cpose[2] += yawrate_mid * dt;
                    }
                    else
                    {
                        // RCLCPP_INFO("invalid joint state delta time: " + dt + " sec");
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "invalid joint state delta time:: '%lu' sec", dt);
                    }
                }
                odom[0] = move_vel_x;
                odom[1] = move_vel_y;
                odom[2] = move_yawrate;

                last_time = current_time;

                last_odom[0] = odom[0];
                last_odom[1] = odom[1];
                last_odom[2] = odom[2];

                MobileOmni.Odometry[0] = odom[0];
                MobileOmni.Odometry[1] = odom[1];
                MobileOmni.Odometry[2] = odom[2];

                //MobileOmni = cpose;

                MobileOmni.Position[0] = cpose[0];
                MobileOmni.Position[1] = cpose[1];
                MobileOmni.Position[2] = cpose[2];

                return MobileOmni;
            }

            JointState InverseKinematics(double odom[])
            {
                Eigen::
                double J[3][4];
                double JJ[3][3];

                double JTranspose[4][3];
                double JInverse[3][3];
                double JPseudo[4][3];

                std::vector<double> Odom[3];
                std::vector<double> Result[4];

                double rpm[] = { 0, 0, 0, 0 };

                J[0][0] = (double)Sin;
                J[0][1] = -(double)Cos;
                J[0][2] = -(double)Sin;
                J[0][3] = (double)Cos;

                J[1][0] = -(double)Cos;
                J[1][1] = -(double)Sin;
                J[1][2] = (double)Cos;
                J[1][3] = (double)Sin;
                
                J[2][0] = 1;
                J[2][1] = 1;
                J[2][2] = 1;
                J[2][3] = 1;

                //J.ConsoleWrite();
                
                transpose(J, JTranspose);

                // JJ.Set = J.Times(JTranspose.Get);
                // //JJ.ConsoleWrite();

                // JInverse.Set = JJ.Inverse;
                // //JInverse.ConsoleWrite();

                // JPseudo.Set = JTranspose.Times(JInverse.Get);
                // //JPseudo.ConsoleWrite();

                // Odom[0] = (float)odom[0];
                // Odom[1] = (float)odom[1];
                // Odom[2] = (float)odom[2];

                // //Odom.ConsoleWrite();

                // Result.Set = JPseudo.Times(Odom.Get);
                // //Result.ConsoleWrite();

                // rpm[0] = (double)(Result[0] / (2 * Math.PI * (GlobalsOmnidirectional.WheelDiameter / 2)) * 60);
                // rpm[1] = (double)(Result[1] / (2 * Math.PI * (GlobalsOmnidirectional.WheelDiameter / 2)) * 60);
                // rpm[2] = (double)(Result[2] / (2 * Math.PI * (GlobalsOmnidirectional.WheelDiameter / 2)) * 60);
                // rpm[3] = (double)(Result[3] / (2 * Math.PI * (GlobalsOmnidirectional.WheelDiameter / 2)) * 60);

                // JointOmni.RPM = rpm;

                // //Console.WriteLine("RPM:{0}, RPM:{1}, RPM:{2}, RPM:{3}", rpm[0], rpm[1], rpm[2], rpm[3]);

                // return JointOmni;
            }

            void transpose(double A[3][4], double B[4][3])
            {
                int i, j;
                for (i = 0; i < 4; i++)
                    for (j = 0; j < 4; j++)
                        B[i][j] = A[j][i];
            }
            
        };
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