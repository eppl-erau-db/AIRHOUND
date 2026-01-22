// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/BatteryStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/battery_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/battery_status__struct.h"
#include "px4_msgs/msg/detail/battery_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _BatteryStatus__ros_msg_type = px4_msgs__msg__BatteryStatus;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_px4_msgs__msg__BatteryStatus(
  const px4_msgs__msg__BatteryStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: connected
  {
    cdr << (ros_message->connected ? true : false);
  }

  // Field name: voltage_v
  {
    cdr << ros_message->voltage_v;
  }

  // Field name: current_a
  {
    cdr << ros_message->current_a;
  }

  // Field name: current_average_a
  {
    cdr << ros_message->current_average_a;
  }

  // Field name: discharged_mah
  {
    cdr << ros_message->discharged_mah;
  }

  // Field name: remaining
  {
    cdr << ros_message->remaining;
  }

  // Field name: scale
  {
    cdr << ros_message->scale;
  }

  // Field name: time_remaining_s
  {
    cdr << ros_message->time_remaining_s;
  }

  // Field name: temperature
  {
    cdr << ros_message->temperature;
  }

  // Field name: cell_count
  {
    cdr << ros_message->cell_count;
  }

  // Field name: source
  {
    cdr << ros_message->source;
  }

  // Field name: priority
  {
    cdr << ros_message->priority;
  }

  // Field name: capacity
  {
    cdr << ros_message->capacity;
  }

  // Field name: cycle_count
  {
    cdr << ros_message->cycle_count;
  }

  // Field name: average_time_to_empty
  {
    cdr << ros_message->average_time_to_empty;
  }

  // Field name: manufacture_date
  {
    cdr << ros_message->manufacture_date;
  }

  // Field name: state_of_health
  {
    cdr << ros_message->state_of_health;
  }

  // Field name: max_error
  {
    cdr << ros_message->max_error;
  }

  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: interface_error
  {
    cdr << ros_message->interface_error;
  }

  // Field name: voltage_cell_v
  {
    size_t size = 14;
    auto array_ptr = ros_message->voltage_cell_v;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: max_cell_voltage_delta
  {
    cdr << ros_message->max_cell_voltage_delta;
  }

  // Field name: is_powering_off
  {
    cdr << (ros_message->is_powering_off ? true : false);
  }

  // Field name: is_required
  {
    cdr << (ros_message->is_required ? true : false);
  }

  // Field name: warning
  {
    cdr << ros_message->warning;
  }

  // Field name: faults
  {
    cdr << ros_message->faults;
  }

  // Field name: full_charge_capacity_wh
  {
    cdr << ros_message->full_charge_capacity_wh;
  }

  // Field name: remaining_capacity_wh
  {
    cdr << ros_message->remaining_capacity_wh;
  }

  // Field name: over_discharge_count
  {
    cdr << ros_message->over_discharge_count;
  }

  // Field name: nominal_voltage
  {
    cdr << ros_message->nominal_voltage;
  }

  // Field name: internal_resistance_estimate
  {
    cdr << ros_message->internal_resistance_estimate;
  }

  // Field name: ocv_estimate
  {
    cdr << ros_message->ocv_estimate;
  }

  // Field name: ocv_estimate_filtered
  {
    cdr << ros_message->ocv_estimate_filtered;
  }

  // Field name: volt_based_soc_estimate
  {
    cdr << ros_message->volt_based_soc_estimate;
  }

  // Field name: voltage_prediction
  {
    cdr << ros_message->voltage_prediction;
  }

  // Field name: prediction_error
  {
    cdr << ros_message->prediction_error;
  }

  // Field name: estimation_covariance_norm
  {
    cdr << ros_message->estimation_covariance_norm;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_deserialize_px4_msgs__msg__BatteryStatus(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs__msg__BatteryStatus * ros_message)
{
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: connected
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->connected = tmp ? true : false;
  }

  // Field name: voltage_v
  {
    cdr >> ros_message->voltage_v;
  }

  // Field name: current_a
  {
    cdr >> ros_message->current_a;
  }

  // Field name: current_average_a
  {
    cdr >> ros_message->current_average_a;
  }

  // Field name: discharged_mah
  {
    cdr >> ros_message->discharged_mah;
  }

  // Field name: remaining
  {
    cdr >> ros_message->remaining;
  }

  // Field name: scale
  {
    cdr >> ros_message->scale;
  }

  // Field name: time_remaining_s
  {
    cdr >> ros_message->time_remaining_s;
  }

  // Field name: temperature
  {
    cdr >> ros_message->temperature;
  }

  // Field name: cell_count
  {
    cdr >> ros_message->cell_count;
  }

  // Field name: source
  {
    cdr >> ros_message->source;
  }

  // Field name: priority
  {
    cdr >> ros_message->priority;
  }

  // Field name: capacity
  {
    cdr >> ros_message->capacity;
  }

  // Field name: cycle_count
  {
    cdr >> ros_message->cycle_count;
  }

  // Field name: average_time_to_empty
  {
    cdr >> ros_message->average_time_to_empty;
  }

  // Field name: manufacture_date
  {
    cdr >> ros_message->manufacture_date;
  }

  // Field name: state_of_health
  {
    cdr >> ros_message->state_of_health;
  }

  // Field name: max_error
  {
    cdr >> ros_message->max_error;
  }

  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: interface_error
  {
    cdr >> ros_message->interface_error;
  }

  // Field name: voltage_cell_v
  {
    size_t size = 14;
    auto array_ptr = ros_message->voltage_cell_v;
    cdr.deserialize_array(array_ptr, size);
  }

  // Field name: max_cell_voltage_delta
  {
    cdr >> ros_message->max_cell_voltage_delta;
  }

  // Field name: is_powering_off
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_powering_off = tmp ? true : false;
  }

  // Field name: is_required
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->is_required = tmp ? true : false;
  }

  // Field name: warning
  {
    cdr >> ros_message->warning;
  }

  // Field name: faults
  {
    cdr >> ros_message->faults;
  }

  // Field name: full_charge_capacity_wh
  {
    cdr >> ros_message->full_charge_capacity_wh;
  }

  // Field name: remaining_capacity_wh
  {
    cdr >> ros_message->remaining_capacity_wh;
  }

  // Field name: over_discharge_count
  {
    cdr >> ros_message->over_discharge_count;
  }

  // Field name: nominal_voltage
  {
    cdr >> ros_message->nominal_voltage;
  }

  // Field name: internal_resistance_estimate
  {
    cdr >> ros_message->internal_resistance_estimate;
  }

  // Field name: ocv_estimate
  {
    cdr >> ros_message->ocv_estimate;
  }

  // Field name: ocv_estimate_filtered
  {
    cdr >> ros_message->ocv_estimate_filtered;
  }

  // Field name: volt_based_soc_estimate
  {
    cdr >> ros_message->volt_based_soc_estimate;
  }

  // Field name: voltage_prediction
  {
    cdr >> ros_message->voltage_prediction;
  }

  // Field name: prediction_error
  {
    cdr >> ros_message->prediction_error;
  }

  // Field name: estimation_covariance_norm
  {
    cdr >> ros_message->estimation_covariance_norm;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__BatteryStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BatteryStatus__ros_msg_type * ros_message = static_cast<const _BatteryStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: connected
  {
    size_t item_size = sizeof(ros_message->connected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: voltage_v
  {
    size_t item_size = sizeof(ros_message->voltage_v);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: current_a
  {
    size_t item_size = sizeof(ros_message->current_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: current_average_a
  {
    size_t item_size = sizeof(ros_message->current_average_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: discharged_mah
  {
    size_t item_size = sizeof(ros_message->discharged_mah);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: remaining
  {
    size_t item_size = sizeof(ros_message->remaining);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: scale
  {
    size_t item_size = sizeof(ros_message->scale);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: time_remaining_s
  {
    size_t item_size = sizeof(ros_message->time_remaining_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: temperature
  {
    size_t item_size = sizeof(ros_message->temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cell_count
  {
    size_t item_size = sizeof(ros_message->cell_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: source
  {
    size_t item_size = sizeof(ros_message->source);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: priority
  {
    size_t item_size = sizeof(ros_message->priority);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: capacity
  {
    size_t item_size = sizeof(ros_message->capacity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cycle_count
  {
    size_t item_size = sizeof(ros_message->cycle_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: average_time_to_empty
  {
    size_t item_size = sizeof(ros_message->average_time_to_empty);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: manufacture_date
  {
    size_t item_size = sizeof(ros_message->manufacture_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: state_of_health
  {
    size_t item_size = sizeof(ros_message->state_of_health);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_error
  {
    size_t item_size = sizeof(ros_message->max_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: interface_error
  {
    size_t item_size = sizeof(ros_message->interface_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: voltage_cell_v
  {
    size_t array_size = 14;
    auto array_ptr = ros_message->voltage_cell_v;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_cell_voltage_delta
  {
    size_t item_size = sizeof(ros_message->max_cell_voltage_delta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: is_powering_off
  {
    size_t item_size = sizeof(ros_message->is_powering_off);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: is_required
  {
    size_t item_size = sizeof(ros_message->is_required);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: warning
  {
    size_t item_size = sizeof(ros_message->warning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: faults
  {
    size_t item_size = sizeof(ros_message->faults);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: full_charge_capacity_wh
  {
    size_t item_size = sizeof(ros_message->full_charge_capacity_wh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: remaining_capacity_wh
  {
    size_t item_size = sizeof(ros_message->remaining_capacity_wh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: over_discharge_count
  {
    size_t item_size = sizeof(ros_message->over_discharge_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: nominal_voltage
  {
    size_t item_size = sizeof(ros_message->nominal_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: internal_resistance_estimate
  {
    size_t item_size = sizeof(ros_message->internal_resistance_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ocv_estimate
  {
    size_t item_size = sizeof(ros_message->ocv_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ocv_estimate_filtered
  {
    size_t item_size = sizeof(ros_message->ocv_estimate_filtered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: volt_based_soc_estimate
  {
    size_t item_size = sizeof(ros_message->volt_based_soc_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: voltage_prediction
  {
    size_t item_size = sizeof(ros_message->voltage_prediction);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: prediction_error
  {
    size_t item_size = sizeof(ros_message->prediction_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: estimation_covariance_norm
  {
    size_t item_size = sizeof(ros_message->estimation_covariance_norm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__BatteryStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: timestamp
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: connected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: voltage_v
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: current_a
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: current_average_a
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: discharged_mah
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: remaining
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: scale
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: time_remaining_s
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: cell_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: source
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: priority
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: capacity
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: cycle_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: average_time_to_empty
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: manufacture_date
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: state_of_health
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: max_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: interface_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: voltage_cell_v
  {
    size_t array_size = 14;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: max_cell_voltage_delta
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: is_powering_off
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: is_required
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: warning
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: faults
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: full_charge_capacity_wh
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: remaining_capacity_wh
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: over_discharge_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: nominal_voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: internal_resistance_estimate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ocv_estimate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ocv_estimate_filtered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: volt_based_soc_estimate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: voltage_prediction
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: prediction_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: estimation_covariance_norm
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__BatteryStatus;
    is_plain =
      (
      offsetof(DataType, estimation_covariance_norm) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
bool cdr_serialize_key_px4_msgs__msg__BatteryStatus(
  const px4_msgs__msg__BatteryStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: connected
  {
    cdr << (ros_message->connected ? true : false);
  }

  // Field name: voltage_v
  {
    cdr << ros_message->voltage_v;
  }

  // Field name: current_a
  {
    cdr << ros_message->current_a;
  }

  // Field name: current_average_a
  {
    cdr << ros_message->current_average_a;
  }

  // Field name: discharged_mah
  {
    cdr << ros_message->discharged_mah;
  }

  // Field name: remaining
  {
    cdr << ros_message->remaining;
  }

  // Field name: scale
  {
    cdr << ros_message->scale;
  }

  // Field name: time_remaining_s
  {
    cdr << ros_message->time_remaining_s;
  }

  // Field name: temperature
  {
    cdr << ros_message->temperature;
  }

  // Field name: cell_count
  {
    cdr << ros_message->cell_count;
  }

  // Field name: source
  {
    cdr << ros_message->source;
  }

  // Field name: priority
  {
    cdr << ros_message->priority;
  }

  // Field name: capacity
  {
    cdr << ros_message->capacity;
  }

  // Field name: cycle_count
  {
    cdr << ros_message->cycle_count;
  }

  // Field name: average_time_to_empty
  {
    cdr << ros_message->average_time_to_empty;
  }

  // Field name: manufacture_date
  {
    cdr << ros_message->manufacture_date;
  }

  // Field name: state_of_health
  {
    cdr << ros_message->state_of_health;
  }

  // Field name: max_error
  {
    cdr << ros_message->max_error;
  }

  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: interface_error
  {
    cdr << ros_message->interface_error;
  }

  // Field name: voltage_cell_v
  {
    size_t size = 14;
    auto array_ptr = ros_message->voltage_cell_v;
    cdr.serialize_array(array_ptr, size);
  }

  // Field name: max_cell_voltage_delta
  {
    cdr << ros_message->max_cell_voltage_delta;
  }

  // Field name: is_powering_off
  {
    cdr << (ros_message->is_powering_off ? true : false);
  }

  // Field name: is_required
  {
    cdr << (ros_message->is_required ? true : false);
  }

  // Field name: warning
  {
    cdr << ros_message->warning;
  }

  // Field name: faults
  {
    cdr << ros_message->faults;
  }

  // Field name: full_charge_capacity_wh
  {
    cdr << ros_message->full_charge_capacity_wh;
  }

  // Field name: remaining_capacity_wh
  {
    cdr << ros_message->remaining_capacity_wh;
  }

  // Field name: over_discharge_count
  {
    cdr << ros_message->over_discharge_count;
  }

  // Field name: nominal_voltage
  {
    cdr << ros_message->nominal_voltage;
  }

  // Field name: internal_resistance_estimate
  {
    cdr << ros_message->internal_resistance_estimate;
  }

  // Field name: ocv_estimate
  {
    cdr << ros_message->ocv_estimate;
  }

  // Field name: ocv_estimate_filtered
  {
    cdr << ros_message->ocv_estimate_filtered;
  }

  // Field name: volt_based_soc_estimate
  {
    cdr << ros_message->volt_based_soc_estimate;
  }

  // Field name: voltage_prediction
  {
    cdr << ros_message->voltage_prediction;
  }

  // Field name: prediction_error
  {
    cdr << ros_message->prediction_error;
  }

  // Field name: estimation_covariance_norm
  {
    cdr << ros_message->estimation_covariance_norm;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_key_px4_msgs__msg__BatteryStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BatteryStatus__ros_msg_type * ros_message = static_cast<const _BatteryStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: connected
  {
    size_t item_size = sizeof(ros_message->connected);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: voltage_v
  {
    size_t item_size = sizeof(ros_message->voltage_v);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: current_a
  {
    size_t item_size = sizeof(ros_message->current_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: current_average_a
  {
    size_t item_size = sizeof(ros_message->current_average_a);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: discharged_mah
  {
    size_t item_size = sizeof(ros_message->discharged_mah);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: remaining
  {
    size_t item_size = sizeof(ros_message->remaining);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: scale
  {
    size_t item_size = sizeof(ros_message->scale);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: time_remaining_s
  {
    size_t item_size = sizeof(ros_message->time_remaining_s);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: temperature
  {
    size_t item_size = sizeof(ros_message->temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cell_count
  {
    size_t item_size = sizeof(ros_message->cell_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: source
  {
    size_t item_size = sizeof(ros_message->source);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: priority
  {
    size_t item_size = sizeof(ros_message->priority);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: capacity
  {
    size_t item_size = sizeof(ros_message->capacity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: cycle_count
  {
    size_t item_size = sizeof(ros_message->cycle_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: average_time_to_empty
  {
    size_t item_size = sizeof(ros_message->average_time_to_empty);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: manufacture_date
  {
    size_t item_size = sizeof(ros_message->manufacture_date);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: state_of_health
  {
    size_t item_size = sizeof(ros_message->state_of_health);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_error
  {
    size_t item_size = sizeof(ros_message->max_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: interface_error
  {
    size_t item_size = sizeof(ros_message->interface_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: voltage_cell_v
  {
    size_t array_size = 14;
    auto array_ptr = ros_message->voltage_cell_v;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: max_cell_voltage_delta
  {
    size_t item_size = sizeof(ros_message->max_cell_voltage_delta);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: is_powering_off
  {
    size_t item_size = sizeof(ros_message->is_powering_off);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: is_required
  {
    size_t item_size = sizeof(ros_message->is_required);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: warning
  {
    size_t item_size = sizeof(ros_message->warning);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: faults
  {
    size_t item_size = sizeof(ros_message->faults);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: full_charge_capacity_wh
  {
    size_t item_size = sizeof(ros_message->full_charge_capacity_wh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: remaining_capacity_wh
  {
    size_t item_size = sizeof(ros_message->remaining_capacity_wh);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: over_discharge_count
  {
    size_t item_size = sizeof(ros_message->over_discharge_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: nominal_voltage
  {
    size_t item_size = sizeof(ros_message->nominal_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: internal_resistance_estimate
  {
    size_t item_size = sizeof(ros_message->internal_resistance_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ocv_estimate
  {
    size_t item_size = sizeof(ros_message->ocv_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: ocv_estimate_filtered
  {
    size_t item_size = sizeof(ros_message->ocv_estimate_filtered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: volt_based_soc_estimate
  {
    size_t item_size = sizeof(ros_message->volt_based_soc_estimate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: voltage_prediction
  {
    size_t item_size = sizeof(ros_message->voltage_prediction);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: prediction_error
  {
    size_t item_size = sizeof(ros_message->prediction_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: estimation_covariance_norm
  {
    size_t item_size = sizeof(ros_message->estimation_covariance_norm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_key_px4_msgs__msg__BatteryStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: timestamp
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: connected
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: voltage_v
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: current_a
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: current_average_a
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: discharged_mah
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: remaining
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: scale
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: time_remaining_s
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: temperature
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: cell_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: source
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: priority
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: capacity
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: cycle_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: average_time_to_empty
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: manufacture_date
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: state_of_health
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: max_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: id
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: interface_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: voltage_cell_v
  {
    size_t array_size = 14;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: max_cell_voltage_delta
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: is_powering_off
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: is_required
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: warning
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: faults
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: full_charge_capacity_wh
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: remaining_capacity_wh
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: over_discharge_count
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Field name: nominal_voltage
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: internal_resistance_estimate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ocv_estimate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: ocv_estimate_filtered
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: volt_based_soc_estimate
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: voltage_prediction
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: prediction_error
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: estimation_covariance_norm
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__BatteryStatus;
    is_plain =
      (
      offsetof(DataType, estimation_covariance_norm) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _BatteryStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const px4_msgs__msg__BatteryStatus * ros_message = static_cast<const px4_msgs__msg__BatteryStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_px4_msgs__msg__BatteryStatus(ros_message, cdr);
}

static bool _BatteryStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  px4_msgs__msg__BatteryStatus * ros_message = static_cast<px4_msgs__msg__BatteryStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_px4_msgs__msg__BatteryStatus(cdr, ros_message);
}

static uint32_t _BatteryStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__BatteryStatus(
      untyped_ros_message, 0));
}

static size_t _BatteryStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__BatteryStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_BatteryStatus = {
  "px4_msgs::msg",
  "BatteryStatus",
  _BatteryStatus__cdr_serialize,
  _BatteryStatus__cdr_deserialize,
  _BatteryStatus__get_serialized_size,
  _BatteryStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _BatteryStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BatteryStatus,
  get_message_typesupport_handle_function,
  &px4_msgs__msg__BatteryStatus__get_type_hash,
  &px4_msgs__msg__BatteryStatus__get_type_description,
  &px4_msgs__msg__BatteryStatus__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, BatteryStatus)() {
  return &_BatteryStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
