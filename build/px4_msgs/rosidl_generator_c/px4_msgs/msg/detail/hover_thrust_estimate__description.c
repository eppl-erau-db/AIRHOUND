// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/HoverThrustEstimate.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/hover_thrust_estimate__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__HoverThrustEstimate__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6d, 0x75, 0x21, 0x4d, 0xe4, 0x82, 0xd2, 0x7e,
      0x40, 0x1f, 0x26, 0xc7, 0xaa, 0x87, 0x21, 0x99,
      0xb8, 0xdc, 0x47, 0x6e, 0x4a, 0xd0, 0x19, 0xea,
      0x9d, 0xa7, 0x2c, 0x4e, 0xaf, 0x24, 0x1d, 0x28,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__HoverThrustEstimate__TYPE_NAME[] = "px4_msgs/msg/HoverThrustEstimate";

// Define type names, field names, and default values
static char px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__timestamp_sample[] = "timestamp_sample";
static char px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__hover_thrust[] = "hover_thrust";
static char px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__hover_thrust_var[] = "hover_thrust_var";
static char px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__accel_innov[] = "accel_innov";
static char px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__accel_innov_var[] = "accel_innov_var";
static char px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__accel_innov_test_ratio[] = "accel_innov_test_ratio";
static char px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__accel_noise_var[] = "accel_noise_var";
static char px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__valid[] = "valid";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__HoverThrustEstimate__FIELDS[] = {
  {
    {px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__timestamp_sample, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__hover_thrust, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__hover_thrust_var, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__accel_innov, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__accel_innov_var, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__accel_innov_test_ratio, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__accel_noise_var, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__HoverThrustEstimate__FIELD_NAME__valid, 5, 5},
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
px4_msgs__msg__HoverThrustEstimate__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__HoverThrustEstimate__TYPE_NAME, 32, 32},
      {px4_msgs__msg__HoverThrustEstimate__FIELDS, 9, 9},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp                # time since system start (microseconds)\n"
  "uint64 timestamp_sample         # time of corresponding sensor data last used for this estimate\n"
  "\n"
  "float32 hover_thrust\\t\\t# estimated hover thrust [0.1, 0.9]\n"
  "float32 hover_thrust_var\\t# estimated hover thrust variance\n"
  "\n"
  "float32 accel_innov\\t\\t# innovation of the last acceleration fusion\n"
  "float32 accel_innov_var\\t\\t# innovation variance of the last acceleration fusion\n"
  "float32 accel_innov_test_ratio\\t# normalized innovation squared test ratio\n"
  "\n"
  "float32 accel_noise_var\\t\\t# vertical acceleration noise variance estimated form innovation residual\n"
  "\n"
  "bool valid";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__HoverThrustEstimate__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__HoverThrustEstimate__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 619, 619},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__HoverThrustEstimate__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__HoverThrustEstimate__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
