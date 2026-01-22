// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleRatesSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_rates_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleRatesSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdd, 0x56, 0x00, 0x09, 0x4f, 0x52, 0x29, 0x66,
      0xef, 0x85, 0x0e, 0x22, 0x84, 0x7b, 0x52, 0xfa,
      0x6a, 0xdd, 0xb6, 0x67, 0x95, 0xb4, 0x63, 0x1c,
      0x69, 0xa2, 0xce, 0x0a, 0x8e, 0xfe, 0x40, 0x63,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleRatesSetpoint__TYPE_NAME[] = "px4_msgs/msg/VehicleRatesSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__roll[] = "roll";
static char px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__pitch[] = "pitch";
static char px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__yaw[] = "yaw";
static char px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__thrust_body[] = "thrust_body";
static char px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__reset_integral[] = "reset_integral";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleRatesSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__roll, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__pitch, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__thrust_body, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleRatesSetpoint__FIELD_NAME__reset_integral, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__VehicleRatesSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleRatesSetpoint__TYPE_NAME, 33, 33},
      {px4_msgs__msg__VehicleRatesSetpoint__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 MESSAGE_VERSION = 0\n"
  "\n"
  "uint64 timestamp\\t# time since system start (microseconds)\n"
  "\n"
  "# body angular rates in FRD frame\n"
  "float32 roll\\t\\t# [rad/s] roll rate setpoint\n"
  "float32 pitch\\t\\t# [rad/s] pitch rate setpoint\n"
  "float32 yaw\\t\\t# [rad/s] yaw rate setpoint\n"
  "\n"
  "# For clarification: For multicopters thrust_body[0] and thrust[1] are usually 0 and thrust[2] is the negative throttle demand.\n"
  "# For fixed wings thrust_x is the throttle demand and thrust_y, thrust_z will usually be zero.\n"
  "float32[3] thrust_body\\t# Normalized thrust command in body NED frame [-1,1]\n"
  "\n"
  "bool reset_integral # Reset roll/pitch/yaw integrals (navigation logic change)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleRatesSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleRatesSetpoint__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 630, 630},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleRatesSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleRatesSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
