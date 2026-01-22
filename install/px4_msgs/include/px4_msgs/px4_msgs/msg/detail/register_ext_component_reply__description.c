// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RegisterExtComponentReply.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/register_ext_component_reply__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RegisterExtComponentReply__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xcc, 0x8f, 0xe5, 0xe1, 0x2b, 0x0c, 0xc5, 0x8c,
      0x2c, 0x97, 0x6a, 0x5b, 0xcd, 0xfa, 0xc5, 0xaf,
      0x3f, 0xa3, 0x9f, 0x1b, 0x0b, 0x74, 0xc8, 0x07,
      0x32, 0x85, 0xdc, 0xbe, 0x6e, 0x62, 0xed, 0x90,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RegisterExtComponentReply__TYPE_NAME[] = "px4_msgs/msg/RegisterExtComponentReply";

// Define type names, field names, and default values
static char px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__request_id[] = "request_id";
static char px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__name[] = "name";
static char px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__px4_ros2_api_version[] = "px4_ros2_api_version";
static char px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__success[] = "success";
static char px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__arming_check_id[] = "arming_check_id";
static char px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__mode_id[] = "mode_id";
static char px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__mode_executor_id[] = "mode_executor_id";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RegisterExtComponentReply__FIELDS[] = {
  {
    {px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__request_id, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__name, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_ARRAY,
      25,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__px4_ros2_api_version, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__arming_check_id, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__mode_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RegisterExtComponentReply__FIELD_NAME__mode_executor_id, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__RegisterExtComponentReply__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RegisterExtComponentReply__TYPE_NAME, 38, 38},
      {px4_msgs__msg__RegisterExtComponentReply__FIELDS, 8, 8},
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
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "uint64 request_id          # ID from the request\n"
  "char[25] name              # name from the request\n"
  "\n"
  "uint16 px4_ros2_api_version\n"
  "\n"
  "bool success\n"
  "int8 arming_check_id      # arming check registration ID (-1 if invalid)\n"
  "int8 mode_id              # assigned mode ID (-1 if invalid)\n"
  "int8 mode_executor_id     # assigned mode executor ID (-1 if invalid)\n"
  "\n"
  "uint8 ORB_QUEUE_LENGTH = 2";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RegisterExtComponentReply__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RegisterExtComponentReply__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 462, 462},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RegisterExtComponentReply__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RegisterExtComponentReply__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
