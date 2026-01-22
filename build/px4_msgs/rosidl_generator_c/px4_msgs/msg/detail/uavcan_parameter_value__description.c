// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/UavcanParameterValue.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/uavcan_parameter_value__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__UavcanParameterValue__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x55, 0x2e, 0xc7, 0xd2, 0x58, 0xfd, 0xcd, 0x2a,
      0x23, 0x4f, 0x45, 0x4a, 0x91, 0xbb, 0x3e, 0x79,
      0xbc, 0xbb, 0xd3, 0xeb, 0x37, 0xd3, 0xc2, 0xb2,
      0xfb, 0xae, 0xda, 0xf5, 0xc4, 0x3d, 0xd9, 0x7f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__UavcanParameterValue__TYPE_NAME[] = "px4_msgs/msg/UavcanParameterValue";

// Define type names, field names, and default values
static char px4_msgs__msg__UavcanParameterValue__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__UavcanParameterValue__FIELD_NAME__node_id[] = "node_id";
static char px4_msgs__msg__UavcanParameterValue__FIELD_NAME__param_id[] = "param_id";
static char px4_msgs__msg__UavcanParameterValue__FIELD_NAME__param_index[] = "param_index";
static char px4_msgs__msg__UavcanParameterValue__FIELD_NAME__param_count[] = "param_count";
static char px4_msgs__msg__UavcanParameterValue__FIELD_NAME__param_type[] = "param_type";
static char px4_msgs__msg__UavcanParameterValue__FIELD_NAME__int_value[] = "int_value";
static char px4_msgs__msg__UavcanParameterValue__FIELD_NAME__real_value[] = "real_value";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__UavcanParameterValue__FIELDS[] = {
  {
    {px4_msgs__msg__UavcanParameterValue__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UavcanParameterValue__FIELD_NAME__node_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UavcanParameterValue__FIELD_NAME__param_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      17,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UavcanParameterValue__FIELD_NAME__param_index, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UavcanParameterValue__FIELD_NAME__param_count, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UavcanParameterValue__FIELD_NAME__param_type, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UavcanParameterValue__FIELD_NAME__int_value, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__UavcanParameterValue__FIELD_NAME__real_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__UavcanParameterValue__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__UavcanParameterValue__TYPE_NAME, 33, 33},
      {px4_msgs__msg__UavcanParameterValue__FIELDS, 8, 8},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# UAVCAN-MAVLink parameter bridge response type\n"
  "uint64 timestamp\\t\\t# time since system start (microseconds)\n"
  "uint8 node_id\\t\\t\\t# UAVCAN node ID mapped from MAVLink component ID\n"
  "char[17] param_id\\t\\t# MAVLink/UAVCAN parameter name\n"
  "int16 param_index\\t\\t# parameter index, if known\n"
  "uint16 param_count\\t\\t# number of parameters exposed by the node\n"
  "uint8 param_type\\t\\t# MAVLink parameter type\n"
  "int64 int_value\\t\\t\\t# current value if param_type is int-like\n"
  "float32 real_value\\t\\t# current value if param_type is float-like";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__UavcanParameterValue__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__UavcanParameterValue__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 501, 501},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__UavcanParameterValue__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__UavcanParameterValue__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
