// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleTorqueSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_torque_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleTorqueSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd7, 0xb6, 0x56, 0x09, 0x9f, 0x58, 0xe3, 0x35,
      0x81, 0x99, 0x2e, 0x49, 0x35, 0x81, 0x8b, 0xaf,
      0x4a, 0xe9, 0xd7, 0xb0, 0xb2, 0xd2, 0x8e, 0x03,
      0xe9, 0xa6, 0x7a, 0x21, 0xc8, 0xe1, 0xcb, 0x0e,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleTorqueSetpoint__TYPE_NAME[] = "px4_msgs/msg/VehicleTorqueSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleTorqueSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleTorqueSetpoint__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleTorqueSetpoint__FIELD_NAME__xyz[] = "xyz";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleTorqueSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleTorqueSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleTorqueSetpoint__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleTorqueSetpoint__FIELD_NAME__xyz, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__VehicleTorqueSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleTorqueSetpoint__TYPE_NAME, 34, 34},
      {px4_msgs__msg__VehicleTorqueSetpoint__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "\n"
  "uint64 timestamp        # time since system start (microseconds)\n"
  "uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)\n"
  "\n"
  "float32[3] xyz          # torque setpoint about X, Y, Z body axis (normalized)\n"
  "\n"
  "# TOPICS vehicle_torque_setpoint\n"
  "# TOPICS vehicle_torque_setpoint_virtual_fw vehicle_torque_setpoint_virtual_mc";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleTorqueSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleTorqueSetpoint__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 360, 360},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleTorqueSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleTorqueSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
