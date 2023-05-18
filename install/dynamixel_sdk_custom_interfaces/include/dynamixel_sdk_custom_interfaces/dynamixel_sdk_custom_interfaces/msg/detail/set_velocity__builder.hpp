// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_sdk_custom_interfaces:msg/SetVelocity.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY__BUILDER_HPP_
#define DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dynamixel_sdk_custom_interfaces/msg/detail/set_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dynamixel_sdk_custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_SetVelocity_velocity
{
public:
  explicit Init_SetVelocity_velocity(::dynamixel_sdk_custom_interfaces::msg::SetVelocity & msg)
  : msg_(msg)
  {}
  ::dynamixel_sdk_custom_interfaces::msg::SetVelocity velocity(::dynamixel_sdk_custom_interfaces::msg::SetVelocity::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::msg::SetVelocity msg_;
};

class Init_SetVelocity_id
{
public:
  Init_SetVelocity_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetVelocity_velocity id(::dynamixel_sdk_custom_interfaces::msg::SetVelocity::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_SetVelocity_velocity(msg_);
  }

private:
  ::dynamixel_sdk_custom_interfaces::msg::SetVelocity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_sdk_custom_interfaces::msg::SetVelocity>()
{
  return dynamixel_sdk_custom_interfaces::msg::builder::Init_SetVelocity_id();
}

}  // namespace dynamixel_sdk_custom_interfaces

#endif  // DYNAMIXEL_SDK_CUSTOM_INTERFACES__MSG__DETAIL__SET_VELOCITY__BUILDER_HPP_
