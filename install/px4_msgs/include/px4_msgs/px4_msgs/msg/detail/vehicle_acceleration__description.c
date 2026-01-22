// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/VehicleAcceleration.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/vehicle_acceleration__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__VehicleAcceleration__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x76, 0xb7, 0x11, 0xac, 0x73, 0xde, 0x8f, 0x5e,
      0x5c, 0x87, 0x77, 0xf2, 0xc2, 0xa2, 0x7d, 0x8f,
      0x29, 0xe3, 0xb2, 0xb3, 0x9d, 0x7a, 0xd2, 0x0d,
      0xe9, 0x82, 0x76, 0xe2, 0x59, 0x5f, 0x4f, 0x11,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__VehicleAcceleration__TYPE_NAME[] = "px4_msgs/msg/VehicleAcceleration";

// Define type names, field names, and default values
static char px4_msgs__msg__VehicleAcceleration__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__VehicleAcceleration__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__VehicleAcceleration__FIELD_NAME__xyz[] = "xyz";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__VehicleAcceleration__FIELDS[] = {
  {
    {px4_msgs__msg__VehicleAcceleration__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAcceleration__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__VehicleAcceleration__FIELD_NAME__xyz, 3, 3},
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
px4_msgs__msg__VehicleAcceleration__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__VehicleAcceleration__TYPE_NAME, 32, 32},
      {px4_msgs__msg__VehicleAcceleration__FIELDS, 3, 3},
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
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "\n"
  "uint64 timestamp_sample\\t\\t# the timestamp of the raw data (microseconds)\n"
  "\n"
  "float32[3] xyz\\t\\t\\t# Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__VehicleAcceleration__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__VehicleAcceleration__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 241, 241},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__VehicleAcceleration__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__VehicleAcceleration__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
