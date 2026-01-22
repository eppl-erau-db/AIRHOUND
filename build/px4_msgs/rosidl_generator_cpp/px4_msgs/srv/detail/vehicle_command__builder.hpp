// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:srv/VehicleCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "px4_msgs/srv/vehicle_command.hpp"


#ifndef PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__BUILDER_HPP_
#define PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/srv/detail/vehicle_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace srv
{

namespace builder
{

class Init_VehicleCommand_Request_request
{
public:
  Init_VehicleCommand_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::px4_msgs::srv::VehicleCommand_Request request(::px4_msgs::srv::VehicleCommand_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::srv::VehicleCommand_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::srv::VehicleCommand_Request>()
{
  return px4_msgs::srv::builder::Init_VehicleCommand_Request_request();
}

}  // namespace px4_msgs


namespace px4_msgs
{

namespace srv
{

namespace builder
{

class Init_VehicleCommand_Response_reply
{
public:
  Init_VehicleCommand_Response_reply()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::px4_msgs::srv::VehicleCommand_Response reply(::px4_msgs::srv::VehicleCommand_Response::_reply_type arg)
  {
    msg_.reply = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::srv::VehicleCommand_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::srv::VehicleCommand_Response>()
{
  return px4_msgs::srv::builder::Init_VehicleCommand_Response_reply();
}

}  // namespace px4_msgs


namespace px4_msgs
{

namespace srv
{

namespace builder
{

class Init_VehicleCommand_Event_response
{
public:
  explicit Init_VehicleCommand_Event_response(::px4_msgs::srv::VehicleCommand_Event & msg)
  : msg_(msg)
  {}
  ::px4_msgs::srv::VehicleCommand_Event response(::px4_msgs::srv::VehicleCommand_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::srv::VehicleCommand_Event msg_;
};

class Init_VehicleCommand_Event_request
{
public:
  explicit Init_VehicleCommand_Event_request(::px4_msgs::srv::VehicleCommand_Event & msg)
  : msg_(msg)
  {}
  Init_VehicleCommand_Event_response request(::px4_msgs::srv::VehicleCommand_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_VehicleCommand_Event_response(msg_);
  }

private:
  ::px4_msgs::srv::VehicleCommand_Event msg_;
};

class Init_VehicleCommand_Event_info
{
public:
  Init_VehicleCommand_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleCommand_Event_request info(::px4_msgs::srv::VehicleCommand_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_VehicleCommand_Event_request(msg_);
  }

private:
  ::px4_msgs::srv::VehicleCommand_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::srv::VehicleCommand_Event>()
{
  return px4_msgs::srv::builder::Init_VehicleCommand_Event_info();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__SRV__DETAIL__VEHICLE_COMMAND__BUILDER_HPP_
