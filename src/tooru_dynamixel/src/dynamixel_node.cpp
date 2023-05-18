#include <cstdio>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "dynamixel_sdk_custom_interfaces/msg/set_velocity.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "tooru_dynamixel/dynamixel_node.hpp"

#define MOTOR_MAX_RPM 1023 * 0.229 // motor's maximum rpm
#define WHEEL_DIAMETER 0.1         // robot's wheel diameter expressed in meters
#define v_to_rpm 0.229             // 1 v = 0.229 RPM

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define ADDRESS_GOAL_VELOCITY 104
#define ADDRESS_PRESENT_VELOCITY 128
#define ADDRESS_VELOCITY_LIMIT 44

#define OPERATING_MODE_CURRENT 0
#define OPERATING_MODE_VELOCITY 1
#define OPERATING_MODE_POSITION 3

#define DXL_VELOCITY_LIMIT 1023

// Data Byte Length
#define LENGTH_GOAL_POSITION 4
#define LENGTH_GOAL_VELOCITY 4
#define LENGTH_PRESENT_POSITION 4
#define LENGTH_PRESENT_VELOCITY 4

// Protocol version
#define PROTOCOL_VERSION 2.0 // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID 11                 // Dynamixel#1 ID: 11
#define DXL2_ID 14                 // Dynamixel#2 ID: 14
#define DXL3_ID 12                 // Dynamixel#3 ID: 12
#define DXL4_ID 13                 // Dynamixel#4 ID: 13
#define BAUDRATE 1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyUSB0" // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDRESS_GOAL_VELOCITY, LENGTH_GOAL_VELOCITY);
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDRESS_PRESENT_VELOCITY, LENGTH_PRESENT_VELOCITY);

uint8_t dxl_id[4] = {DXL1_ID, DXL2_ID, DXL3_ID, DXL4_ID};
uint8_t dxl_error = 0;
uint32_t goal_position = 0;
// uint32_t dxl_goal_velocity[]; // Goal velocity
int dxl_comm_result = COMM_TX_FAIL;
bool dxl_addparam_result = false; // addParam result
bool dxl_getdata_result = false;  // GetParam result

uint8_t param_goal_velocity1[4];
uint8_t param_goal_velocity2[4];
uint8_t param_goal_velocity3[4];
uint8_t param_goal_velocity4[4];
int32_t dxl_present_position[4] = {0, 0, 0, 0}; // 4096
int32_t dxl_present_velocity[4] = {0, 0, 0, 0}; // 4096

DynamixelNode::DynamixelNode()
    : Node("dynamixel_node")
{
  RCLCPP_INFO(this->get_logger(), "Run Dynamixel node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
      rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // set_position_subscriber_ =
  //     this->create_subscription<SetPosition>(
  //         "set_position",
  //         QOS_RKL10V,
  //         [this](const SetPosition::SharedPtr msg) -> void
  //         {
  //           uint8_t dxl_error = 0;

  //           // Position Value of X series is 4 byte data.
  //           // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  //           uint32_t goal_position = (unsigned int)msg->position; // Convert int32 -> uint32

  //           // Write Goal Position (length : 4 bytes)
  //           // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  //           dxl_comm_result =
  //               packetHandler->write4ByteTxRx(
  //                   portHandler,
  //                   (uint8_t)msg->id,
  //                   ADDR_GOAL_POSITION,
  //                   goal_position,
  //                   &dxl_error);

  //           if (dxl_comm_result != COMM_SUCCESS)
  //           {
  //             RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
  //           }
  //           else if (dxl_error != 0)
  //           {
  //             RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
  //           }
  //           else
  //           {
  //             RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
  //           }
  //         });

  // auto get_present_position =
  //     [this](
  //         const std::shared_ptr<GetPosition::Request> request,
  //         std::shared_ptr<GetPosition::Response> response) -> void
  // {
  //   // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  //   // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  //   dxl_comm_result = packetHandler->read4ByteTxRx(
  //       portHandler,
  //       (uint8_t)request->id,
  //       ADDR_PRESENT_POSITION,
  //       reinterpret_cast<uint32_t *>(&present_position),
  //       &dxl_error);

  //   RCLCPP_INFO(
  //       this->get_logger(),
  //       "Get [ID: %d] [Present Position: %d]",
  //       request->id,
  //       present_position);

  //   response->position = present_position;
  // };

  // get_position_server_ = create_service<GetPosition>("get_position", get_present_position);

  // Velocity
  set_velocity_subscriber_ =
      this->create_subscription<SetVelocity>(
          "set_velocity",
          QOS_RKL10V,
          [this](const dynamixel_sdk_custom_interfaces::msg::SetVelocity::ConstSharedPtr &msg) -> void
          {
            uint8_t dxl_error = 0;

            // Position Value of X series is 4 byte data.
            // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
            auto dxl_goal_velocity = msg->velocity; // Convert int32 -> uint32

            auto dxl_id = msg->id;

            // Allocate goal velocity value into byte array
            param_goal_velocity1[0] = DXL_LOBYTE(DXL_LOWORD(round(dxl_goal_velocity[0] / 0.229)));
            param_goal_velocity1[1] = DXL_HIBYTE(DXL_LOWORD(round(dxl_goal_velocity[0] / 0.229)));
            param_goal_velocity1[2] = DXL_LOBYTE(DXL_HIWORD(round(dxl_goal_velocity[0] / 0.229)));
            param_goal_velocity1[3] = DXL_HIBYTE(DXL_HIWORD(round(dxl_goal_velocity[0] / 0.229)));

            // Allocate goal velocity value into byte array
            param_goal_velocity2[0] = DXL_LOBYTE(DXL_LOWORD(round(dxl_goal_velocity[1] / 0.229)));
            param_goal_velocity2[1] = DXL_HIBYTE(DXL_LOWORD(round(dxl_goal_velocity[1] / 0.229)));
            param_goal_velocity2[2] = DXL_LOBYTE(DXL_HIWORD(round(dxl_goal_velocity[1] / 0.229)));
            param_goal_velocity2[3] = DXL_HIBYTE(DXL_HIWORD(round(dxl_goal_velocity[1] / 0.229)));

            // Allocate goal velocity value into byte array
            param_goal_velocity3[0] = DXL_LOBYTE(DXL_LOWORD(round(dxl_goal_velocity[2] / 0.229)));
            param_goal_velocity3[1] = DXL_HIBYTE(DXL_LOWORD(round(dxl_goal_velocity[2] / 0.229)));
            param_goal_velocity3[2] = DXL_LOBYTE(DXL_HIWORD(round(dxl_goal_velocity[2] / 0.229)));
            param_goal_velocity3[3] = DXL_HIBYTE(DXL_HIWORD(round(dxl_goal_velocity[2] / 0.229)));

            // Allocate goal velocity value into byte array
            param_goal_velocity4[0] = DXL_LOBYTE(DXL_LOWORD(round(dxl_goal_velocity[3] / 0.229)));
            param_goal_velocity4[1] = DXL_HIBYTE(DXL_LOWORD(round(dxl_goal_velocity[3] / 0.229)));
            param_goal_velocity4[2] = DXL_LOBYTE(DXL_HIWORD(round(dxl_goal_velocity[3] / 0.229)));
            param_goal_velocity4[3] = DXL_HIBYTE(DXL_HIWORD(round(dxl_goal_velocity[3] / 0.229)));

            dxl_addparam_result = groupSyncWrite.addParam(dxl_id[0], param_goal_velocity1);
            dxl_addparam_result = groupSyncWrite.addParam(dxl_id[1], param_goal_velocity2);
            dxl_addparam_result = groupSyncWrite.addParam(dxl_id[2], param_goal_velocity3);
            dxl_addparam_result = groupSyncWrite.addParam(dxl_id[3], param_goal_velocity4);

            // Syncwrite goal velocity
            dxl_comm_result = groupSyncWrite.txPacket();
            if (dxl_comm_result != COMM_SUCCESS)
              packetHandler->getTxRxResult(dxl_comm_result);

            // Clear syncwrite parameter storage
            groupSyncWrite.clearParam();

            if (dxl_comm_result != COMM_SUCCESS)
            {
              RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
            }
            else if (dxl_error != 0)
            {
              RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
            }
            // else
            // {
            //   RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Velocity: %d]", msg->id, msg->velocity);
            // }
          });
}

DynamixelNode::~DynamixelNode()
{
}

// void setupDynamixel(uint8_t dxl_id[])
// {
//   // // Use Position Control Mode
//   // dxl_comm_result = packetHandler->write1ByteTxRx(
//   //     portHandler,
//   //     dxl_id,
//   //     ADDR_OPERATING_MODE,
//   //     OPERATING_MODE_POSITION,
//   //     &dxl_error);

//   // if (dxl_comm_result != COMM_SUCCESS)
//   // {
//   //   RCLCPP_ERROR(rclcpp::get_logger("dynamixel_node"), "Failed to set Position Control Mode.");
//   // }
//   // else
//   // {
//   //   RCLCPP_INFO(rclcpp::get_logger("dynamixel_node"), "Succeeded to set Position Control Mode.");
//   // }

//   for (int cnt = 0; cnt < 4; cnt++)
//   {
//     // Use Velocity Control Mode
//     dxl_comm_result = packetHandler->write1ByteTxRx(
//         portHandler,
//         dxl_id[cnt],
//         ADDR_OPERATING_MODE,
//         OPERATING_MODE_VELOCITY,
//         &dxl_error);

//     if (dxl_comm_result != COMM_SUCCESS)
//     {
//       RCLCPP_ERROR(rclcpp::get_logger("dynamixel_node"), "Failed to set Velocity Control Mode Dxl: [%d].", dxl_id[cnt]);
//     }
//     else
//     {
//       RCLCPP_INFO(rclcpp::get_logger("dynamixel_node"), "Succeeded to set Velocity Control Mode Dxl: [%d].", dxl_id[cnt]);
//     }
//   }

//   for (int cnt = 0; cnt < 4; cnt++)
//   {
//     // Enable Torque of DYNAMIXEL
//     dxl_comm_result = packetHandler->write1ByteTxRx(
//         portHandler,
//         dxl_id[cnt],
//         ADDR_TORQUE_ENABLE,
//         1,
//         &dxl_error);

//     if (dxl_comm_result != COMM_SUCCESS)
//     {
//       RCLCPP_ERROR(rclcpp::get_logger("dynamixel_node"), "Failed to enable torque Dxl: [%d].", dxl_id[cnt]);
//     }
//     else
//     {
//       RCLCPP_INFO(rclcpp::get_logger("dynamixel_node"), "Succeeded to enable torque Dxl: [%d].", dxl_id[cnt]);
//     }
//   }
// }

void setupDynamixel(uint8_t dxl_id)
{
  // Use Position Control Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_OPERATING_MODE,
    OPERATING_MODE_VELOCITY,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_node"), "Failed to set Velocity Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_node"), "Succeeded to set Velocity Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    dxl_id,
    ADDR_TORQUE_ENABLE,
    1,
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char *argv[])
{
  //portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  //packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false)
  {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_node"), "Failed to open the port!");
    return -1;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false)
  {
    RCLCPP_ERROR(rclcpp::get_logger("dynamixel_node"), "Failed to set the baudrate!");
    return -1;
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("dynamixel_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel(BROADCAST_ID);

  rclcpp::init(argc, argv);

  auto dynamixelnode = std::make_shared<DynamixelNode>();
  rclcpp::spin(dynamixelnode);
  rclcpp::shutdown();

  for (int cnt = 0; cnt < 4; cnt++)
  {
    // Disable Torque of DYNAMIXEL
    packetHandler->write1ByteTxRx(
        portHandler,
        BROADCAST_ID,
        ADDR_TORQUE_ENABLE,
        0,
        &dxl_error);
  }

  return 0;
}