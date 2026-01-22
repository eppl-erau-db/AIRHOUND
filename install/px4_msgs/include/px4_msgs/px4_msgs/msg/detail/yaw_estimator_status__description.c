// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/YawEstimatorStatus.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/yaw_estimator_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__YawEstimatorStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfb, 0xae, 0x18, 0x49, 0x00, 0x31, 0xcf, 0x35,
      0xc7, 0x06, 0xf0, 0xa2, 0x12, 0x3a, 0x7a, 0xbc,
      0x1c, 0x64, 0x76, 0xb1, 0x54, 0xfe, 0x4b, 0x74,
      0x56, 0x62, 0x0d, 0x0b, 0xb5, 0x89, 0x79, 0x4c,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__YawEstimatorStatus__TYPE_NAME[] = "px4_msgs/msg/YawEstimatorStatus";

// Define type names, field names, and default values
static char px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__yaw_composite[] = "yaw_composite";
static char px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__yaw_variance[] = "yaw_variance";
static char px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__yaw_composite_valid[] = "yaw_composite_valid";
static char px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__yaw[] = "yaw";
static char px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__innov_vn[] = "innov_vn";
static char px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__innov_ve[] = "innov_ve";
static char px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__weight[] = "weight";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__YawEstimatorStatus__FIELDS[] = {
  {
    {px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__yaw_composite, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__yaw_variance, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__yaw_composite_valid, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      5,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__innov_vn, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      5,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__innov_ve, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      5,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__YawEstimatorStatus__FIELD_NAME__weight, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      5,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__YawEstimatorStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__YawEstimatorStatus__TYPE_NAME, 31, 31},
      {px4_msgs__msg__YawEstimatorStatus__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "uint64 timestamp_sample         # the timestamp of the raw data (microseconds)\n"
  "\n"
  "float32 yaw_composite\\t# composite yaw from GSF (rad)\n"
  "float32 yaw_variance\\t# composite yaw variance from GSF (rad^2)\n"
  "bool yaw_composite_valid\n"
  "\n"
  "float32[5] yaw\\t\\t# yaw estimate for each model in the filter bank (rad)\n"
  "float32[5] innov_vn\\t# North velocity innovation for each model in the filter bank (m/s)\n"
  "float32[5] innov_ve\\t# East velocity innovation for each model in the filter bank (m/s)\n"
  "float32[5] weight\\t# weighting for each model in the filter bank";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__YawEstimatorStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__YawEstimatorStatus__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 591, 591},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__YawEstimatorStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__YawEstimatorStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
