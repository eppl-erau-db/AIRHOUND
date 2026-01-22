// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleAttitudeSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_attitude_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleAttitudeSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x1b, 0xf3, 0x4f, 0x1b, 0x39, 0x33, 0x5d, 0xf0,
      0xe6, 0x76, 0x78, 0x88, 0x5f, 0xfd, 0x21, 0xbf,
      0xb0, 0x8b, 0xb2, 0x6f, 0x97, 0xea, 0xf3, 0x6a,
      0x94, 0xe8, 0xfc, 0x50, 0x83, 0xd5, 0xc1, 0x63,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleAttitudeSetpoint__TYPE_NAME[] = "px4_msgs/msg/VehicleAttitudeSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleAttitudeSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleAttitudeSetpoint__FIELD_NAME__yaw_sp_move_rate[] = "yaw_sp_move_rate";
static char px4_msgs__msg__VehicleAttitudeSetpoint__FIELD_NAME__q_d[] = "q_d";
static char px4_msgs__msg__VehicleAttitudeSetpoint__FIELD_NAME__thrust_body[] = "thrust_body";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleAttitudeSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleAttitudeSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAttitudeSetpoint__FIELD_NAME__yaw_sp_move_rate, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAttitudeSetpoint__FIELD_NAME__q_d, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      4,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAttitudeSetpoint__FIELD_NAME__thrust_body, 11, 11},
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
px4_msgs__msg__VehicleAttitudeSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleAttitudeSetpoint__TYPE_NAME, 36, 36},
      {px4_msgs__msg__VehicleAttitudeSetpoint__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 MESSAGE_VERSION = 1\n"
  "\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "float32 yaw_sp_move_rate\\t# rad/s (commanded by user)\n"
  "\n"
  "# For quaternion-based attitude control\n"
  "float32[4] q_d\\t\\t\\t# Desired quaternion for quaternion control\n"
  "\n"
  "# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.\n"
  "# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.\n"
  "float32[3] thrust_body\\t\\t# Normalized thrust command in body FRD frame [-1,1]\n"
  "\n"
  "# TOPICS vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleAttitudeSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleAttitudeSetpoint__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 638, 638},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleAttitudeSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleAttitudeSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
