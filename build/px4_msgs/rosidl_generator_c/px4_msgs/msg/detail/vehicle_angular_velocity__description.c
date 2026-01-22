// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleAngularVelocity.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_angular_velocity__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleAngularVelocity__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa9, 0x08, 0x5b, 0x1e, 0xd1, 0xb4, 0x28, 0xc4,
      0xe7, 0xa6, 0xc7, 0xf5, 0x56, 0x4f, 0x3c, 0x0b,
      0x02, 0x6d, 0xe1, 0xc5, 0x79, 0x1c, 0x33, 0x62,
      0xdd, 0x19, 0x82, 0x09, 0x2a, 0x76, 0x99, 0xda,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleAngularVelocity__TYPE_NAME[] = "px4_msgs/msg/VehicleAngularVelocity";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleAngularVelocity__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleAngularVelocity__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleAngularVelocity__FIELD_NAME__xyz[] = "xyz";
static char px4_msgs__msg__VehicleAngularVelocity__FIELD_NAME__xyz_derivative[] = "xyz_derivative";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleAngularVelocity__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleAngularVelocity__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAngularVelocity__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAngularVelocity__FIELD_NAME__xyz, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAngularVelocity__FIELD_NAME__xyz_derivative, 14, 14},
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
px4_msgs__msg__VehicleAngularVelocity__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleAngularVelocity__TYPE_NAME, 35, 35},
      {px4_msgs__msg__VehicleAngularVelocity__FIELDS, 4, 4},
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
  "uint64 timestamp          # time since system start (microseconds)\n"
  "uint64 timestamp_sample   # timestamp of the data sample on which this message is based (microseconds)\n"
  "\n"
  "float32[3] xyz\\t\\t  # Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s\n"
  "\n"
  "float32[3] xyz_derivative # angular acceleration about the FRD body frame XYZ-axis in rad/s^2\n"
  "\n"
  "# TOPICS vehicle_angular_velocity vehicle_angular_velocity_groundtruth";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleAngularVelocity__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleAngularVelocity__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 461, 461},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleAngularVelocity__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleAngularVelocity__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
